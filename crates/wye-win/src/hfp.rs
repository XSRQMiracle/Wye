//! Bluetooth hands-free (HFP) detection.
//!
//! - Windows 10 exposes a headset as a "Stereo" (A2DP) endpoint and "Hands-Free" render/capture
//!   endpoints in the same container. When another application uses a hands-free endpoint the
//!   headset switches to HFP: the Stereo endpoint stays active and its streams report no error,
//!   but their audio is discarded. Detected by watching the audio sessions on the hands-free
//!   endpoints (`IAudioSessionNotification` + per-session state events, plus a periodic scan): an
//!   active session of another process means hands-free mode, its process name is the occupant.
//! - Windows 11 merges both profiles into one endpoint whose device format drops to 8/16/24 kHz
//!   (usually mono) while the microphone is in use: checked whenever the device format changes.

use std::collections::HashMap;
use std::sync::mpsc::Sender;

use windows::Win32::Foundation::CloseHandle;
use windows::Win32::Media::Audio::{
    eCapture, eRender, AudioSessionDisconnectReason, AudioSessionState, AudioSessionStateActive,
    IAudioSessionControl, IAudioSessionControl2, IAudioSessionEvents, IAudioSessionEvents_Impl,
    IAudioSessionManager2, IAudioSessionNotification, IAudioSessionNotification_Impl,
    IMMDeviceEnumerator, DEVICE_STATE_ACTIVE,
};
use windows::Win32::System::Com::CLSCTX_ALL;
use windows::Win32::System::Threading::{
    OpenProcess, QueryFullProcessImageNameW, PROCESS_NAME_WIN32, PROCESS_QUERY_LIMITED_INFORMATION,
};
use windows_core::{implement, Interface, Ref, BOOL, GUID, PCWSTR, PWSTR};
use wye_platform::{DeviceId, DeviceMode, EventSink, PlatformEvent};

use crate::classify::{is_hands_free_format, process_display_name};
use crate::devices::{describe, device_format, list_endpoints, Endpoint};
use crate::format::WaveFormat;
use crate::watch::Note;

/// Forwards "a session was created" to the event thread.
#[implement(IAudioSessionNotification)]
struct SessionCreated {
    tx: Sender<Note>,
}

impl IAudioSessionNotification_Impl for SessionCreated_Impl {
    fn OnSessionCreated(&self, _session: Ref<IAudioSessionControl>) -> windows_core::Result<()> {
        let _ = self.tx.send(Note::Sessions);
        Ok(())
    }
}

/// Forwards session state changes to the event thread.
#[implement(IAudioSessionEvents)]
struct SessionState {
    tx: Sender<Note>,
}

impl IAudioSessionEvents_Impl for SessionState_Impl {
    fn OnDisplayNameChanged(&self, _: &PCWSTR, _: *const GUID) -> windows_core::Result<()> {
        Ok(())
    }
    fn OnIconPathChanged(&self, _: &PCWSTR, _: *const GUID) -> windows_core::Result<()> {
        Ok(())
    }
    fn OnSimpleVolumeChanged(&self, _: f32, _: BOOL, _: *const GUID) -> windows_core::Result<()> {
        Ok(())
    }
    fn OnChannelVolumeChanged(
        &self,
        _: u32,
        _: *const f32,
        _: u32,
        _: *const GUID,
    ) -> windows_core::Result<()> {
        Ok(())
    }
    fn OnGroupingParamChanged(&self, _: *const GUID, _: *const GUID) -> windows_core::Result<()> {
        Ok(())
    }
    fn OnStateChanged(&self, _: AudioSessionState) -> windows_core::Result<()> {
        let _ = self.tx.send(Note::Sessions);
        Ok(())
    }
    fn OnSessionDisconnected(&self, _: AudioSessionDisconnectReason) -> windows_core::Result<()> {
        let _ = self.tx.send(Note::Sessions);
        Ok(())
    }
}

/// One Windows 10 headset: its Stereo endpoint and the session managers of its hands-free
/// endpoints.
struct HandsFreeGroup {
    stereo_id: String,
    managers: Vec<(IAudioSessionManager2, IAudioSessionNotification)>,
    hooks: Vec<(IAudioSessionControl, IAudioSessionEvents)>,
    mode: DeviceMode,
    occupant: Option<String>,
}

impl HandsFreeGroup {
    fn unhook_sessions(&mut self) {
        for (control, events) in self.hooks.drain(..) {
            // SAFETY: unregisters a callback registered by `hook_sessions`.
            let _ = unsafe { control.UnregisterAudioSessionNotification(&events) };
        }
    }

    fn hook_sessions(&mut self, tx: &Sender<Note>) {
        self.unhook_sessions();
        for (manager, _) in &self.managers {
            for control in sessions(manager) {
                let events: IAudioSessionEvents = SessionState { tx: tx.clone() }.into();
                // SAFETY: plain COM call; the pair is kept alive until unregistered.
                if unsafe { control.RegisterAudioSessionNotification(&events) }.is_ok() {
                    self.hooks.push((control, events));
                }
            }
        }
    }

    /// Name of the first other process with an active session on a hands-free endpoint.
    fn active_occupant(&self, own_pid: u32) -> Option<Option<String>> {
        for (manager, _) in &self.managers {
            for control in sessions(manager) {
                // SAFETY: plain COM calls.
                let active =
                    unsafe { control.GetState() }.is_ok_and(|s| s == AudioSessionStateActive);
                if !active {
                    continue;
                }
                let Ok(c2) = control.cast::<IAudioSessionControl2>() else {
                    return Some(None);
                };
                // SAFETY: S_OK means the system-sounds session.
                if unsafe { c2.IsSystemSoundsSession() }.0 == 0 {
                    continue;
                }
                // SAFETY: plain COM call.
                let pid = unsafe { c2.GetProcessId() }.unwrap_or(0);
                if pid == own_pid {
                    continue;
                }
                return Some(process_name(pid));
            }
        }
        None
    }

    fn release(&mut self) {
        self.unhook_sessions();
        for (manager, notification) in self.managers.drain(..) {
            // SAFETY: unregisters the notification registered in `HfpMonitor::rebuild`.
            let _ = unsafe { manager.UnregisterSessionNotification(&notification) };
        }
    }
}

fn sessions(manager: &IAudioSessionManager2) -> Vec<IAudioSessionControl> {
    // SAFETY: plain COM calls.
    unsafe {
        let Ok(list) = manager.GetSessionEnumerator() else {
            return Vec::new();
        };
        let count = list.GetCount().unwrap_or(0).max(0);
        (0..count).filter_map(|i| list.GetSession(i).ok()).collect()
    }
}

fn process_name(pid: u32) -> Option<String> {
    if pid == 0 {
        return None;
    }
    // SAFETY: query-only process handle, closed below; buffer and length are valid.
    unsafe {
        let handle = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, false, pid).ok()?;
        let mut buf = [0u16; 520];
        let mut len = buf.len() as u32;
        let ok = QueryFullProcessImageNameW(
            handle,
            PROCESS_NAME_WIN32,
            PWSTR(buf.as_mut_ptr()),
            &mut len,
        )
        .is_ok();
        let _ = CloseHandle(handle);
        if !ok {
            return None;
        }
        let path = String::from_utf16_lossy(buf.get(..len as usize)?);
        process_display_name(&path)
    }
}

/// Lives on the event thread; all methods are called there.
pub(crate) struct HfpMonitor {
    tx: Sender<Note>,
    own_pid: u32,
    groups: Vec<HandsFreeGroup>,
    /// Last mode derived from the device format, per Bluetooth endpoint (Windows 11 rule).
    format_modes: HashMap<String, DeviceMode>,
}

impl HfpMonitor {
    pub(crate) fn new(tx: Sender<Note>) -> HfpMonitor {
        HfpMonitor {
            tx,
            own_pid: std::process::id(),
            groups: Vec::new(),
            format_modes: HashMap::new(),
        }
    }

    /// Re-discovers headsets with separate hands-free endpoints and (re)registers the session
    /// watches. Previous modes are kept so unchanged state is not reported again.
    pub(crate) fn rebuild(&mut self, en: &IMMDeviceEnumerator, sink: &EventSink) {
        let previous: HashMap<String, (DeviceMode, Option<String>)> = self
            .groups
            .drain(..)
            .map(|mut g| {
                g.release();
                (g.stereo_id.clone(), (g.mode, g.occupant.clone()))
            })
            .collect();
        let outputs = list_endpoints(en, eRender, DEVICE_STATE_ACTIVE).unwrap_or_default();
        let inputs = list_endpoints(en, eCapture, DEVICE_STATE_ACTIVE).unwrap_or_default();
        for out in outputs.iter().filter(|e| e.is_bluetooth()) {
            if !out.class.hands_free {
                self.format_modes
                    .entry(out.id.clone())
                    .or_insert_with(|| format_mode(out).unwrap_or(DeviceMode::Normal));
            }
            let Some(group) = out.group.as_deref() else {
                continue;
            };
            if out.class.hands_free {
                continue;
            }
            let hands_free: Vec<&Endpoint> = outputs
                .iter()
                .chain(inputs.iter())
                .filter(|e| e.class.hands_free && e.group.as_deref() == Some(group))
                .collect();
            if hands_free.is_empty() {
                continue;
            }
            let (mode, occupant) = previous
                .get(&out.id)
                .cloned()
                .unwrap_or((DeviceMode::Normal, None));
            let mut g = HandsFreeGroup {
                stereo_id: out.id.clone(),
                managers: Vec::new(),
                hooks: Vec::new(),
                mode,
                occupant,
            };
            for hf in hands_free {
                // SAFETY: plain COM calls; activating the session manager opens no stream.
                unsafe {
                    let Ok(manager) = hf
                        .device
                        .Activate::<IAudioSessionManager2>(CLSCTX_ALL, None)
                    else {
                        continue;
                    };
                    // The enumerator must be requested once for session notifications to flow.
                    let _ = manager.GetSessionEnumerator();
                    let notification: IAudioSessionNotification = SessionCreated {
                        tx: self.tx.clone(),
                    }
                    .into();
                    if manager.RegisterSessionNotification(&notification).is_ok() {
                        g.managers.push((manager, notification));
                    }
                }
            }
            g.hook_sessions(&self.tx);
            self.groups.push(g);
        }
        self.poll(sink);
    }

    /// A session was created or changed state: re-hook new sessions, then rescan.
    pub(crate) fn sessions_changed(&mut self, sink: &EventSink) {
        for g in &mut self.groups {
            g.hook_sessions(&self.tx);
        }
        self.poll(sink);
    }

    /// Rescans the hands-free sessions and reports mode changes of the Stereo endpoints.
    pub(crate) fn poll(&mut self, sink: &EventSink) {
        for g in &mut self.groups {
            let (mode, occupant) = match g.active_occupant(self.own_pid) {
                Some(occupant) => (DeviceMode::BluetoothHandsFree, occupant),
                None => (DeviceMode::Normal, None),
            };
            if mode != g.mode || occupant != g.occupant {
                g.mode = mode;
                g.occupant = occupant.clone();
                sink(PlatformEvent::DeviceModeChanged {
                    id: DeviceId(g.stereo_id.clone()),
                    mode,
                    occupant,
                });
            }
        }
    }

    /// The device format of `id` changed: apply the Windows 11 rule to Bluetooth endpoints.
    pub(crate) fn format_changed(&mut self, en: &IMMDeviceEnumerator, id: &str, sink: &EventSink) {
        let Ok(device) = crate::devices::get_device(en, &DeviceId(id.to_owned())) else {
            return;
        };
        let Ok(ep) = describe(en, device) else {
            return;
        };
        if !ep.is_bluetooth() || ep.class.hands_free {
            return;
        }
        let Some(mode) = format_mode(&ep) else {
            return;
        };
        let previous = self
            .format_modes
            .insert(ep.id.clone(), mode)
            .unwrap_or(DeviceMode::Normal);
        if previous != mode {
            sink(PlatformEvent::DeviceModeChanged {
                id: DeviceId(ep.id),
                mode,
                occupant: None,
            });
        }
    }

    pub(crate) fn release(&mut self) {
        for g in &mut self.groups {
            g.release();
        }
        self.groups.clear();
    }
}

impl Drop for HfpMonitor {
    fn drop(&mut self) {
        self.release();
    }
}

/// Mode implied by the endpoint's current device format.
fn format_mode(ep: &Endpoint) -> Option<DeviceMode> {
    let raw = device_format(&ep.device)?;
    let f = WaveFormat::parse(&raw).map(|f| (f.rate, f.channels));
    let (rate, channels) = f.unwrap_or((raw.rate, raw.channels));
    Some(if is_hands_free_format(rate, channels) {
        DeviceMode::BluetoothHandsFree
    } else {
        DeviceMode::Normal
    })
}
