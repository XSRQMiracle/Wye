//! macOS adapter: Core Audio HAL IOProcs + Process Tap (macOS 14.4+).
//!
//! * Capture: a global stereo Process Tap that excludes this process, inside a private aggregate
//!   device that contains only the tap (`tapautostart = 0`, sub-tap drift compensation off).
//! * Output / calibration input: one raw HAL IOProc per device (C function pointer variant).
//! * Events: HAL property listeners on a private serial dispatch queue.
//!
//! Every IOProc converts the HAL buffers into pre-allocated scratch buffers (4096 frames × 2
//! channels) and never allocates, locks, logs or panics.
#![cfg_attr(not(target_os = "macos"), allow(unused))]
// Core Audio constants keep their SDK names (they are matched on in listener callbacks).
#![allow(non_upper_case_globals)]

#[cfg(target_os = "macos")]
mod devices;
#[cfg(target_os = "macos")]
mod hal;
#[cfg(target_os = "macos")]
mod hfp;
#[cfg(target_os = "macos")]
mod input;
#[cfg(target_os = "macos")]
mod output;
#[cfg(target_os = "macos")]
mod permission;
#[cfg(target_os = "macos")]
mod rt;
#[cfg(target_os = "macos")]
mod tap;
#[cfg(target_os = "macos")]
mod volume;
#[cfg(target_os = "macos")]
mod watch;

#[cfg(target_os = "macos")]
pub use platform::MacPlatform;
#[cfg(target_os = "macos")]
pub use tap::CaptureProbe;

// The lib's unit tests check the IOProcs with assert_no_alloc, which needs its allocator.
#[cfg(all(test, target_os = "macos"))]
#[global_allocator]
static ALLOC: assert_no_alloc::AllocDisabler = assert_no_alloc::AllocDisabler;

#[cfg(target_os = "macos")]
use std::sync::{Arc, Mutex};

/// State shared by the platform object, its streams and its watcher.
#[cfg(target_os = "macos")]
pub(crate) struct Shared {
    pub tb: hal::Timebase,
    pub queue: Arc<hal::QueueHandle>,
    sink: Mutex<Option<Arc<wye_platform::EventSink>>>,
}

#[cfg(target_os = "macos")]
impl Shared {
    /// Sends an event to the active watcher's sink, if any (non-real-time threads only).
    pub fn emit(&self, ev: wye_platform::PlatformEvent) {
        let s = self.sink.lock().unwrap_or_else(|e| e.into_inner()).clone();
        if let Some(s) = s {
            s(ev);
        }
    }

    pub fn set_sink(&self, s: Arc<wye_platform::EventSink>) {
        *self.sink.lock().unwrap_or_else(|e| e.into_inner()) = Some(s);
    }

    /// Clears the sink if it is still `s` (a newer watcher keeps its own).
    pub fn clear_sink(&self, s: &Arc<wye_platform::EventSink>) {
        let mut g = self.sink.lock().unwrap_or_else(|e| e.into_inner());
        if g.as_ref().is_some_and(|cur| Arc::ptr_eq(cur, s)) {
            *g = None;
        }
    }
}

#[cfg(target_os = "macos")]
mod platform {
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Arc, Mutex};
    use std::time::Duration;

    use objc2::runtime::AnyClass;
    use wye_platform::{
        Capabilities, CaptureConfig, CaptureHandler, DeviceId, DeviceInfo, EventSink, HostNanos,
        InputHandler, OutputConfig, PermissionState, Platform, PlatformError, RenderHandler,
        Stream, WatchGuard,
    };

    use crate::hal::{self, device_by_uid, device_uid, kAudioObjectSystemObject};
    use crate::tap::CaptureProbe;
    use crate::{devices, input, output, permission, tap, volume, watch, Shared};

    /// The macOS implementation of [`wye_platform::Platform`].
    pub struct MacPlatform {
        shared: Arc<Shared>,
        show_virtual: AtomicBool,
        last_probe: Mutex<Option<CaptureProbe>>,
    }

    fn env_show_virtual() -> bool {
        std::env::var("WYE_SHOW_VIRTUAL").is_ok_and(|v| v == "1")
    }

    /// `kern.osproductversion` as (major, minor).
    fn os_version() -> Option<(u32, u32)> {
        extern "C" {
            fn sysctlbyname(
                name: *const std::ffi::c_char,
                oldp: *mut std::ffi::c_void,
                oldlenp: *mut usize,
                newp: *mut std::ffi::c_void,
                newlen: usize,
            ) -> i32;
        }
        let mut buf = [0u8; 64];
        let mut len = buf.len();
        // SAFETY: valid NUL-terminated name and buffer/length pair.
        let r = unsafe {
            sysctlbyname(
                c"kern.osproductversion".as_ptr(),
                buf.as_mut_ptr().cast(),
                &mut len,
                std::ptr::null_mut(),
                0,
            )
        };
        if r != 0 {
            return None;
        }
        let s = std::str::from_utf8(&buf[..len.min(buf.len())]).ok()?;
        let s = s.trim_end_matches('\0');
        let mut it = s.split('.').map(|p| p.parse::<u32>().ok());
        Some((it.next()??, it.next().flatten().unwrap_or(0)))
    }

    impl MacPlatform {
        /// Creates the adapter. Fails on macOS older than 14.4 (no Process Tap).
        pub fn new() -> Result<Self, PlatformError> {
            match os_version() {
                Some(v) if v >= (14, 4) => {}
                Some((a, b)) => {
                    return Err(PlatformError::UnsupportedOs(format!(
                        "macOS {a}.{b} (14.4 or later required)"
                    )))
                }
                None => {}
            }
            if AnyClass::get(c"CATapDescription").is_none() {
                return Err(PlatformError::UnsupportedOs(
                    "CATapDescription is not available".into(),
                ));
            }
            Ok(MacPlatform {
                shared: Arc::new(Shared {
                    tb: hal::Timebase::new(),
                    queue: hal::QueueHandle::new("io.github.xsrqmiracle.wye.hal"),
                    sink: Mutex::new(None),
                }),
                show_virtual: AtomicBool::new(env_show_virtual()),
                last_probe: Mutex::new(None),
            })
        }

        /// Shows virtual devices (BlackHole, Loopback, …) in the device lists. Also enabled by
        /// the environment variable `WYE_SHOW_VIRTUAL=1`.
        pub fn set_show_virtual(&self, show: bool) {
            self.show_virtual
                .store(show || env_show_virtual(), Ordering::Relaxed);
        }

        fn show_virtual(&self) -> bool {
            self.show_virtual.load(Ordering::Relaxed)
        }

        /// Result of the permission probe of the most recent `open_capture` (diagnostics).
        pub fn last_capture_probe(&self) -> Option<CaptureProbe> {
            self.last_probe
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .clone()
        }
    }

    fn find(id: &DeviceId) -> Result<hal::AudioObjectID, PlatformError> {
        device_by_uid(&id.0).ok_or_else(|| PlatformError::DeviceNotFound(id.clone()))
    }

    impl Platform for MacPlatform {
        fn capabilities(&self) -> Capabilities {
            Capabilities {
                can_mute_capture: true,
                can_set_default_output: true,
                device_timestamps: true,
            }
        }

        fn output_devices(&self) -> Result<Vec<DeviceInfo>, PlatformError> {
            Ok(devices::output_devices(self.show_virtual())
                .into_iter()
                .map(|(_, i)| i)
                .collect())
        }

        fn default_output(&self) -> Result<Option<DeviceId>, PlatformError> {
            let d = hal::default_output_id();
            if d == hal::kAudioObjectUnknown {
                return Ok(None);
            }
            Ok(device_uid(d).map(DeviceId))
        }

        fn set_default_output(&self, id: &DeviceId) -> Result<(), PlatformError> {
            let d = find(id)?;
            hal::set(
                kAudioObjectSystemObject,
                hal::kAudioHardwarePropertyDefaultOutputDevice,
                hal::SCOPE_GLOBAL,
                &d,
            )
            .map_err(|st| output::os("set default output device", st))
        }

        fn calibration_inputs(&self) -> Result<Vec<DeviceInfo>, PlatformError> {
            Ok(devices::calibration_inputs(self.show_virtual())
                .into_iter()
                .map(|(_, i)| i)
                .collect())
        }

        fn watch(&self, sink: EventSink) -> Result<Box<dyn WatchGuard>, PlatformError> {
            Ok(watch::watch(&self.shared, sink))
        }

        fn capture_permission(&self) -> PermissionState {
            // There is no public query API; the capture probe and the all-zero watchdog decide.
            PermissionState::Unknown
        }

        fn open_permission_settings(&self) -> Result<(), PlatformError> {
            permission::open_url(permission::AUDIO_CAPTURE_SETTINGS)
        }

        fn open_microphone_settings(&self) -> Result<(), PlatformError> {
            permission::open_url(permission::MICROPHONE_SETTINGS)
        }

        fn open_sound_settings(&self) -> Result<(), PlatformError> {
            permission::open_url(permission::SOUND_SETTINGS)
        }

        fn open_capture(
            &self,
            cfg: CaptureConfig,
            handler: Box<dyn CaptureHandler>,
        ) -> Result<Box<dyn Stream>, PlatformError> {
            match tap::open_capture(&self.shared, cfg, handler) {
                Ok((s, probe)) => {
                    *self.last_probe.lock().unwrap_or_else(|e| e.into_inner()) = Some(probe);
                    Ok(s)
                }
                Err(e) => Err(e),
            }
        }

        fn open_output(
            &self,
            id: &DeviceId,
            cfg: OutputConfig,
            handler: Box<dyn RenderHandler>,
        ) -> Result<Box<dyn Stream>, PlatformError> {
            output::open_output(&self.shared, id, cfg, handler)
        }

        fn open_input(
            &self,
            id: &DeviceId,
            handler: Box<dyn InputHandler>,
        ) -> Result<Box<dyn Stream>, PlatformError> {
            // Only devices that are safe for calibration (never Bluetooth, never hidden ones).
            let dev = devices::calibration_inputs(self.show_virtual())
                .into_iter()
                .find(|(_, i)| &i.id == id)
                .map(|(d, _)| d)
                .ok_or_else(|| PlatformError::DeviceNotFound(id.clone()))?;
            input::open_input(&self.shared, id, dev, handler)
        }

        fn input_latency(&self, id: &DeviceId) -> Result<Duration, PlatformError> {
            Ok(input::input_latency(find(id)?))
        }

        fn output_volume(&self, id: &DeviceId) -> Option<(f32, bool)> {
            volume::output_volume(find(id).ok()?)
        }

        fn microphone_permission(&self) -> PermissionState {
            permission::microphone_permission()
        }

        fn now(&self) -> HostNanos {
            self.shared.tb.now_ns()
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn os_version_parses() {
            let v = os_version().expect("kern.osproductversion");
            assert!(v.0 >= 10);
        }

        #[test]
        fn now_is_monotonic_nanoseconds() {
            let p = MacPlatform::new().expect("platform");
            let a = p.now();
            std::thread::sleep(Duration::from_millis(5));
            let b = p.now();
            assert!(b > a);
            let d = b - a;
            assert!((4_000_000..500_000_000).contains(&d), "{d}");
        }

        #[test]
        fn enumeration_is_consistent() {
            // Read-only: lists devices and checks basic invariants of whatever is attached.
            let p = MacPlatform::new().expect("platform");
            let outs = p.output_devices().unwrap();
            for d in &outs {
                assert!(d.output_channels > 0);
                assert!(!d.id.0.is_empty());
                assert_ne!(d.transport, wye_platform::Transport::Aggregate);
                assert!(d.reported_latency.total >= d.reported_latency.after_timestamp);
            }
            for d in p.calibration_inputs().unwrap() {
                assert!(!matches!(
                    d.transport,
                    wye_platform::Transport::Bluetooth | wye_platform::Transport::BluetoothLe
                ));
            }
            if let Some(def) = p.default_output().unwrap() {
                let _ = p.output_volume(&def);
            }
            assert_eq!(p.capture_permission(), PermissionState::Unknown);
        }

        #[test]
        fn unknown_device_is_reported() {
            let p = MacPlatform::new().expect("platform");
            let id = DeviceId("no-such-device-uid".into());
            assert!(matches!(
                p.input_latency(&id),
                Err(PlatformError::DeviceNotFound(_))
            ));
            assert!(p.output_volume(&id).is_none());
        }
    }
}
