//! Device notifications. The `IMMNotificationClient` (and the volume / session callbacks) only
//! forward plain messages to a dedicated event thread — they never block, call back into the
//! enumerator or release MMDevice objects. The event thread queries properties, de-duplicates,
//! runs the hands-free detection and calls the [`EventSink`].

use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use windows::Win32::Devices::FunctionDiscovery::PKEY_Device_FriendlyName;
use windows::Win32::Foundation::PROPERTYKEY;
use windows::Win32::Media::Audio::Endpoints::{
    IAudioEndpointVolume, IAudioEndpointVolumeCallback, IAudioEndpointVolumeCallback_Impl,
    IAudioMeterInformation,
};
use windows::Win32::Media::Audio::{
    EDataFlow, ERole, IMMDeviceEnumerator, IMMNotificationClient, IMMNotificationClient_Impl,
    PKEY_AudioEngine_DeviceFormat, AUDIO_VOLUME_NOTIFICATION_DATA, DEVICE_STATE,
};
use windows::Win32::System::Com::CLSCTX_ALL;
use windows_core::{implement, PCWSTR};
use wye_platform::{DeviceId, EventSink, PlatformError, PlatformEvent, WatchGuard};

use crate::classify::{is_tracked_default_change, DefaultTracker};
use crate::com::{enumerator, os_err, ComScope, SinkSlot};
use crate::devices::{default_output_id, get_device};
use crate::hfp::HfpMonitor;

/// Messages from the OS callbacks (and the guard) to the event thread.
pub(crate) enum Note {
    Stop,
    /// Endpoint added / removed / state or name changed.
    DevicesChanged,
    /// `eRender`/`eConsole` default changed (`None`: no default output any more).
    Default(Option<String>),
    /// `PKEY_AudioEngine_DeviceFormat` of an endpoint changed.
    Format(String),
    Volume {
        id: String,
        scalar: f32,
        muted: bool,
    },
    /// Audio session activity on a watched hands-free endpoint.
    Sessions,
}

fn pcwstr_string(p: &PCWSTR) -> Option<String> {
    if p.is_null() {
        return None;
    }
    // SAFETY: the OS passes a valid NUL-terminated string for the duration of the callback.
    unsafe { p.to_string().ok() }
}

fn same_key(a: &PROPERTYKEY, b: &PROPERTYKEY) -> bool {
    a.fmtid == b.fmtid && a.pid == b.pid
}

#[implement(IMMNotificationClient)]
struct Notifier {
    tx: Sender<Note>,
}

impl IMMNotificationClient_Impl for Notifier_Impl {
    fn OnDeviceStateChanged(&self, _: &PCWSTR, _: DEVICE_STATE) -> windows_core::Result<()> {
        let _ = self.tx.send(Note::DevicesChanged);
        Ok(())
    }
    fn OnDeviceAdded(&self, _: &PCWSTR) -> windows_core::Result<()> {
        let _ = self.tx.send(Note::DevicesChanged);
        Ok(())
    }
    fn OnDeviceRemoved(&self, _: &PCWSTR) -> windows_core::Result<()> {
        let _ = self.tx.send(Note::DevicesChanged);
        Ok(())
    }
    fn OnDefaultDeviceChanged(
        &self,
        flow: EDataFlow,
        role: ERole,
        id: &PCWSTR,
    ) -> windows_core::Result<()> {
        if is_tracked_default_change(flow.0, role.0) {
            let _ = self.tx.send(Note::Default(pcwstr_string(id)));
        }
        Ok(())
    }
    fn OnPropertyValueChanged(&self, id: &PCWSTR, key: &PROPERTYKEY) -> windows_core::Result<()> {
        if same_key(key, &PKEY_AudioEngine_DeviceFormat) {
            if let Some(id) = pcwstr_string(id) {
                let _ = self.tx.send(Note::Format(id));
            }
        } else if same_key(key, &PKEY_Device_FriendlyName) {
            let _ = self.tx.send(Note::DevicesChanged);
        }
        Ok(())
    }
}

#[implement(IAudioEndpointVolumeCallback)]
struct VolumeNotifier {
    id: String,
    tx: Sender<Note>,
}

impl IAudioEndpointVolumeCallback_Impl for VolumeNotifier_Impl {
    fn OnNotify(&self, data: *mut AUDIO_VOLUME_NOTIFICATION_DATA) -> windows_core::Result<()> {
        if data.is_null() {
            return Ok(());
        }
        // SAFETY: the OS passes a valid notification record for the duration of the callback.
        let (scalar, muted) = unsafe { ((*data).fMasterVolume, (*data).bMuted.as_bool()) };
        let _ = self.tx.send(Note::Volume {
            id: self.id.clone(),
            scalar,
            muted,
        });
        Ok(())
    }
}

/// Volume callback and peak meter on the current default output.
struct DefaultWatch {
    id: Option<String>,
    volume: Option<(IAudioEndpointVolume, IAudioEndpointVolumeCallback)>,
    meter: Option<IAudioMeterInformation>,
}

impl DefaultWatch {
    fn attach(en: &IMMDeviceEnumerator, id: Option<String>, tx: &Sender<Note>) -> DefaultWatch {
        let mut w = DefaultWatch {
            id: id.clone(),
            volume: None,
            meter: None,
        };
        let Some(id) = id else {
            return w;
        };
        let Ok(device) = get_device(en, &DeviceId(id.clone())) else {
            return w;
        };
        // SAFETY: plain COM calls; the callback stays alive until unregistered in `Drop`.
        unsafe {
            if let Ok(vol) = device.Activate::<IAudioEndpointVolume>(CLSCTX_ALL, None) {
                let cb: IAudioEndpointVolumeCallback = VolumeNotifier { id, tx: tx.clone() }.into();
                if vol.RegisterControlChangeNotify(&cb).is_ok() {
                    w.volume = Some((vol, cb));
                }
            }
            w.meter = device
                .Activate::<IAudioMeterInformation>(CLSCTX_ALL, None)
                .ok();
        }
        w
    }

    fn peak(&self) -> Option<f32> {
        // SAFETY: plain COM call.
        self.meter
            .as_ref()
            .and_then(|m| unsafe { m.GetPeakValue() }.ok())
    }
}

impl Drop for DefaultWatch {
    fn drop(&mut self) {
        if let Some((vol, cb)) = self.volume.take() {
            // SAFETY: unregisters the callback registered in `attach`.
            let _ = unsafe { vol.UnregisterControlChangeNotify(&cb) };
        }
    }
}

/// Keeps the event thread alive; dropping it stops the thread, which unregisters everything.
struct WinWatch {
    tx: Sender<Note>,
    join: Option<JoinHandle<()>>,
    slot: SinkSlot,
    sink: Arc<EventSink>,
}

impl WatchGuard for WinWatch {}

impl Drop for WinWatch {
    fn drop(&mut self) {
        let _ = self.tx.send(Note::Stop);
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
        if let Ok(mut slot) = self.slot.lock() {
            if slot.as_ref().is_some_and(|s| Arc::ptr_eq(s, &self.sink)) {
                *slot = None;
            }
        }
    }
}

/// Starts the event thread and registers the endpoint notification callback.
pub(crate) fn start(sink: EventSink, slot: SinkSlot) -> Result<Box<dyn WatchGuard>, PlatformError> {
    let sink = Arc::new(sink);
    let (tx, rx) = mpsc::channel::<Note>();
    let (ready_tx, ready_rx) = mpsc::sync_channel::<Result<(), PlatformError>>(1);
    let thread_sink = sink.clone();
    let thread_tx = tx.clone();
    let join = std::thread::Builder::new()
        .name("wye-win-events".into())
        .spawn(move || event_thread(thread_sink, thread_tx, rx, ready_tx))
        .map_err(|e| PlatformError::Os {
            call: "spawn",
            code: e.raw_os_error().unwrap_or(-1) as i64,
        })?;
    match ready_rx.recv() {
        Ok(Ok(())) => {
            if let Ok(mut s) = slot.lock() {
                *s = Some(sink.clone());
            }
            Ok(Box::new(WinWatch {
                tx,
                join: Some(join),
                slot,
                sink,
            }))
        }
        Ok(Err(e)) => {
            let _ = join.join();
            Err(e)
        }
        Err(_) => {
            let _ = join.join();
            Err(PlatformError::Os {
                call: "event thread",
                code: -1,
            })
        }
    }
}

/// How often hands-free sessions and system activity are re-checked without notifications.
const POLL_INTERVAL: Duration = Duration::from_secs(1);

/// Peak level above which the default output counts as playing.
const ACTIVE_PEAK: f32 = 1e-5;

fn event_thread(
    sink: Arc<EventSink>,
    tx: Sender<Note>,
    rx: Receiver<Note>,
    ready: mpsc::SyncSender<Result<(), PlatformError>>,
) {
    let com = match ComScope::enter() {
        Ok(c) => c,
        Err(e) => {
            let _ = ready.send(Err(e));
            return;
        }
    };
    let en = match enumerator() {
        Ok(en) => en,
        Err(e) => {
            let _ = ready.send(Err(e));
            return;
        }
    };
    let client: IMMNotificationClient = Notifier { tx: tx.clone() }.into();
    // SAFETY: `client` stays alive until it is unregistered below.
    if let Err(e) = unsafe { en.RegisterEndpointNotificationCallback(&client) } {
        let _ = ready.send(Err(os_err("RegisterEndpointNotificationCallback", &e)));
        return;
    }
    let _ = ready.send(Ok(()));
    drop(ready);

    let sink: &EventSink = &sink;
    let mut defaults = DefaultTracker::default();
    let current = default_output_id(&en);
    defaults.prime(current.as_deref());
    let mut default_watch = DefaultWatch::attach(&en, current, &tx);
    let mut hfp = HfpMonitor::new(tx.clone());
    hfp.rebuild(&en, sink);
    let mut audio_active: Option<bool> = None;
    let mut last_poll = Instant::now();

    'outer: loop {
        let first = match rx.recv_timeout(POLL_INTERVAL) {
            Ok(n) => Some(n),
            Err(RecvTimeoutError::Timeout) => None,
            Err(RecvTimeoutError::Disconnected) => break,
        };
        // Coalesce bursts (the OS often sends several notifications for one user action).
        let mut notes: Vec<Note> = first.into_iter().collect();
        notes.extend(rx.try_iter());
        let mut stop = false;
        let mut devices_changed = false;
        let mut sessions = false;
        let mut new_default: Option<Option<String>> = None;
        for note in notes {
            match note {
                Note::Stop => stop = true,
                Note::DevicesChanged => devices_changed = true,
                Note::Default(id) => {
                    if let Some(changed) = defaults.observe(id.as_deref()) {
                        new_default = Some(changed);
                    }
                }
                Note::Format(id) => {
                    sink(PlatformEvent::DeviceFormatChanged(DeviceId(id.clone())));
                    hfp.format_changed(&en, &id, sink);
                }
                Note::Volume { id, scalar, muted } => {
                    if default_watch.id.as_deref() == Some(id.as_str()) {
                        sink(PlatformEvent::VolumeChanged {
                            id: DeviceId(id),
                            scalar: Some(scalar),
                            muted: Some(muted),
                        });
                    }
                }
                Note::Sessions => sessions = true,
            }
        }
        if stop {
            break 'outer;
        }
        if devices_changed {
            sink(PlatformEvent::DevicesChanged);
            // The default may vanish without an OnDefaultDeviceChanged callback.
            if let Some(changed) = defaults.observe(default_output_id(&en).as_deref()) {
                new_default = Some(changed);
            }
            hfp.rebuild(&en, sink);
        }
        if let Some(id) = new_default {
            drop(std::mem::replace(
                &mut default_watch,
                DefaultWatch::attach(&en, id.clone(), &tx),
            ));
            sink(PlatformEvent::DefaultOutputChanged(id.map(DeviceId)));
        }
        if sessions {
            hfp.sessions_changed(sink);
        }
        if last_poll.elapsed() >= POLL_INTERVAL {
            last_poll = Instant::now();
            hfp.poll(sink);
            if let Some(peak) = default_watch.peak() {
                let active = peak > ACTIVE_PEAK;
                if audio_active != Some(active) {
                    audio_active = Some(active);
                    sink(PlatformEvent::SystemAudioActive(active));
                }
            }
        }
    }

    drop(default_watch);
    hfp.release();
    drop(hfp);
    // SAFETY: unregisters the callback registered above (not from inside a callback).
    let _ = unsafe { en.UnregisterEndpointNotificationCallback(&client) };
    drop(client);
    drop(en);
    drop(com);
}
