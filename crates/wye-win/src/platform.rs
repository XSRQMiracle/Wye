//! `wye_platform::Platform` implementation for Windows.

use std::os::windows::process::CommandExt;
use std::process::Command;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use wye_platform::{
    Capabilities, CaptureConfig, CaptureHandler, DeviceId, DeviceInfo, EventSink, HostNanos,
    InputHandler, OutputConfig, PermissionState, Platform, PlatformError, RenderHandler, Stream,
    StreamKind, WatchGuard,
};

use crate::capture::CaptureStream;
use crate::classify::is_remote_session_name;
use crate::com::{enumerator, spawn_stream, ComScope, Qpc, SinkSlot};
use crate::devices::{
    default_output_id, device_info, endpoint_volume, get_device, visible_outputs,
};
use crate::input::{self, InputStream, MicPermission, MIC_DENIED, MIC_GRANTED};
use crate::render::RenderStream;

/// `CREATE_NO_WINDOW`: do not flash a console window when launching the settings app.
const CREATE_NO_WINDOW: u32 = 0x0800_0000;

/// The Windows platform adapter (WASAPI).
pub struct WinPlatform {
    sink: SinkSlot,
    mic_permission: MicPermission,
    qpc: Qpc,
}

impl WinPlatform {
    /// Creates the adapter and checks that the audio device enumerator is available.
    pub fn new() -> Result<Self, PlatformError> {
        let _com = ComScope::enter()?;
        enumerator()?;
        Ok(WinPlatform {
            sink: Arc::new(Mutex::new(None)),
            mic_permission: Arc::new(AtomicU8::new(input::MIC_UNKNOWN)),
            qpc: Qpc::new(),
        })
    }

    /// Remote Desktop sessions only see the client's "Remote Audio" endpoint.
    fn remote_session() -> bool {
        is_remote_session_name(std::env::var("SESSIONNAME").ok().as_deref())
    }
}

fn open_uri(uri: &str) -> Result<(), PlatformError> {
    Command::new("cmd")
        .args(["/C", "start", "", uri])
        .creation_flags(CREATE_NO_WINDOW)
        .spawn()
        .map(|_| ())
        .map_err(|e| PlatformError::Os {
            call: "start ms-settings",
            code: e.raw_os_error().unwrap_or(-1) as i64,
        })
}

impl Platform for WinPlatform {
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            can_mute_capture: false,
            can_set_default_output: false,
            device_timestamps: true,
        }
    }

    fn output_devices(&self) -> Result<Vec<DeviceInfo>, PlatformError> {
        let _com = ComScope::enter()?;
        let en = enumerator()?;
        Ok(visible_outputs(&en)?.iter().map(device_info).collect())
    }

    fn default_output(&self) -> Result<Option<DeviceId>, PlatformError> {
        let _com = ComScope::enter()?;
        let en = enumerator()?;
        Ok(default_output_id(&en).map(DeviceId))
    }

    fn set_default_output(&self, _id: &DeviceId) -> Result<(), PlatformError> {
        Err(PlatformError::Unsupported(
            "changing the default output device (open the sound settings instead)",
        ))
    }

    fn calibration_inputs(&self) -> Result<Vec<DeviceInfo>, PlatformError> {
        let _com = ComScope::enter()?;
        let en = enumerator()?;
        input::calibration_inputs(&en)
    }

    fn watch(&self, sink: EventSink) -> Result<Box<dyn WatchGuard>, PlatformError> {
        crate::watch::start(sink, self.sink.clone())
    }

    fn capture_permission(&self) -> PermissionState {
        // Endpoint loopback needs no permission on Windows.
        PermissionState::Granted
    }

    fn open_permission_settings(&self) -> Result<(), PlatformError> {
        Err(PlatformError::Unsupported(
            "system audio capture needs no permission on Windows",
        ))
    }

    fn open_microphone_settings(&self) -> Result<(), PlatformError> {
        open_uri("ms-settings:privacy-microphone")
    }

    fn open_sound_settings(&self) -> Result<(), PlatformError> {
        open_uri("ms-settings:sound")
    }

    fn open_capture(
        &self,
        cfg: CaptureConfig,
        handler: Box<dyn CaptureHandler>,
    ) -> Result<Box<dyn Stream>, PlatformError> {
        if cfg.mute_original {
            return Err(PlatformError::Unsupported(
                "muting the captured output (Windows has no capture-and-mute)",
            ));
        }
        if Self::remote_session() {
            return Err(PlatformError::Unsupported("rdp"));
        }
        spawn_stream(
            "wye-capture",
            StreamKind::Capture,
            self.sink.clone(),
            move || CaptureStream::open(cfg, handler),
        )
    }

    fn open_output(
        &self,
        id: &DeviceId,
        cfg: OutputConfig,
        handler: Box<dyn RenderHandler>,
    ) -> Result<Box<dyn Stream>, PlatformError> {
        let id = id.clone();
        let kind = StreamKind::Output(id.clone());
        spawn_stream("wye-render", kind, self.sink.clone(), move || {
            RenderStream::open(&id, cfg, handler)
        })
    }

    fn open_input(
        &self,
        id: &DeviceId,
        handler: Box<dyn InputHandler>,
    ) -> Result<Box<dyn Stream>, PlatformError> {
        let id = id.clone();
        let kind = StreamKind::Input(id.clone());
        let permission = self.mic_permission.clone();
        spawn_stream("wye-input", kind, self.sink.clone(), move || {
            InputStream::open(&id, handler, &permission)
        })
    }

    fn input_latency(&self, id: &DeviceId) -> Result<Duration, PlatformError> {
        let _com = ComScope::enter()?;
        let en = enumerator()?;
        get_device(&en, id)?;
        // Packet timestamps mark the first captured sample; calibration measures relative
        // values, so no input latency is applied.
        Ok(Duration::ZERO)
    }

    fn output_volume(&self, id: &DeviceId) -> Option<(f32, bool)> {
        let _com = ComScope::enter().ok()?;
        let en = enumerator().ok()?;
        endpoint_volume(&get_device(&en, id).ok()?)
    }

    fn microphone_permission(&self) -> PermissionState {
        match self.mic_permission.load(Ordering::Acquire) {
            MIC_GRANTED => PermissionState::Granted,
            MIC_DENIED => PermissionState::Denied,
            _ => PermissionState::Unknown,
        }
    }

    fn now(&self) -> HostNanos {
        self.qpc.now_ns()
    }
}
