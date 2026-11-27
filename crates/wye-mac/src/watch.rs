//! HAL property listeners mapped to `PlatformEvent`s. All listeners run on the adapter's private
//! serial dispatch queue; callbacks only read properties and call the event sink.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};

use wye_platform::{DeviceId, DeviceMode, EventSink, PlatformEvent, WatchGuard};

use crate::devices::{self, format_suggests_hands_free, is_bluetooth, transport};
use crate::hal::*;
use crate::{hfp, volume, Shared};
use objc2_core_audio::AudioObjectPropertyAddress;

/// Device properties whose change means "re-read the format / latency and reopen the stream".
const FORMAT_SELECTORS: [(u32, u32); 9] = [
    (kAudioDevicePropertyNominalSampleRate, SCOPE_GLOBAL),
    (kAudioDevicePropertyStreams, SCOPE_OUTPUT),
    (kAudioDevicePropertyStreams, SCOPE_INPUT),
    (kAudioDevicePropertyStreamConfiguration, SCOPE_OUTPUT),
    (kAudioDevicePropertyLatency, SCOPE_OUTPUT),
    (kAudioDevicePropertySafetyOffset, SCOPE_OUTPUT),
    (kAudioDevicePropertyDeviceHasChanged, SCOPE_GLOBAL),
    (kAudioDevicePropertyDataSource, SCOPE_OUTPUT),
    (kAudioDevicePropertyPreferredChannelsForStereo, SCOPE_OUTPUT),
];

#[derive(Default)]
struct Inner {
    system: Vec<Listener>,
    devices: Vec<Listener>,
    processes: Vec<Listener>,
    volume: Vec<Listener>,
    default_out: AudioObjectID,
    modes: HashMap<String, (DeviceMode, Option<String>)>,
    audio_active: bool,
}

pub struct Watcher {
    me: Weak<Watcher>,
    shared: Arc<Shared>,
    sink: Arc<EventSink>,
    alive: AtomicBool,
    inner: Mutex<Inner>,
}

pub struct Guard(Arc<Watcher>);

impl WatchGuard for Guard {}

impl Drop for Guard {
    fn drop(&mut self) {
        let w = &self.0;
        w.alive.store(false, Ordering::SeqCst);
        w.shared.clear_sink(&w.sink);
        // Take the listeners out first and unregister them without holding the lock, so a
        // callback that is waiting for the lock can finish.
        let taken = {
            let mut g = w.inner.lock().unwrap_or_else(|e| e.into_inner());
            (
                std::mem::take(&mut g.system),
                std::mem::take(&mut g.devices),
                std::mem::take(&mut g.processes),
                std::mem::take(&mut g.volume),
            )
        };
        drop(taken);
    }
}

pub fn watch(shared: &Arc<Shared>, sink: EventSink) -> Box<dyn WatchGuard> {
    let sink: Arc<EventSink> = Arc::new(sink);
    shared.set_sink(sink.clone());
    let w = Arc::new_cyclic(|me| Watcher {
        me: me.clone(),
        shared: shared.clone(),
        sink,
        alive: AtomicBool::new(true),
        inner: Mutex::new(Inner::default()),
    });
    {
        let weak = Arc::downgrade(&w);
        let mut sys = Vec::new();
        for sel in [
            kAudioHardwarePropertyDevices,
            kAudioHardwarePropertyDefaultOutputDevice,
            kAudioHardwarePropertyServiceRestarted,
            kAudioHardwarePropertyProcessObjectList,
        ] {
            let wk = weak.clone();
            if let Ok(l) = Listener::add(
                &shared.queue,
                kAudioObjectSystemObject,
                addr(sel, SCOPE_GLOBAL),
                move |s| with(&wk, |w| w.on_system(s)),
            ) {
                sys.push(l);
            }
        }
        let mut g = w.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.system = sys;
        g.default_out = default_output_id();
    }
    // Initial listener sets and state (no events for the initial state).
    w.resync_devices();
    w.resync_processes();
    w.resync_volume();
    let (modes, active) = (current_modes(), hfp::other_process_running_output());
    {
        let mut g = w.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.modes = modes;
        g.audio_active = active;
    }
    Box::new(Guard(w))
}

fn with(w: &Weak<Watcher>, f: impl FnOnce(&Watcher)) {
    if let Some(w) = w.upgrade() {
        if w.alive.load(Ordering::SeqCst) {
            f(&w);
        }
    }
}

/// Current mode of every Bluetooth output: hands-free if another process holds its microphone
/// or its format looks like the hands-free profile.
fn current_modes() -> HashMap<String, (DeviceMode, Option<String>)> {
    let occ = hfp::bluetooth_occupancy();
    let mut out = HashMap::new();
    for d in system_devices() {
        let t = transport(d);
        if !is_bluetooth(t) || channels(d, SCOPE_OUTPUT) == 0 {
            continue;
        }
        let Some(uid) = device_uid(d) else { continue };
        let rate: f64 = get(d, kAudioDevicePropertyNominalSampleRate, SCOPE_GLOBAL).unwrap_or(0.0);
        let occupant = occ.get(&d).cloned();
        let hf =
            occupant.is_some() || format_suggests_hands_free(t, rate, channels(d, SCOPE_OUTPUT));
        let mode = if hf {
            DeviceMode::BluetoothHandsFree
        } else {
            DeviceMode::Normal
        };
        out.insert(uid, (mode, occupant));
    }
    out
}

impl Watcher {
    fn emit(&self, ev: PlatformEvent) {
        if self.alive.load(Ordering::SeqCst) {
            (self.sink)(ev);
        }
    }

    fn self_weak(&self) -> Weak<Watcher> {
        self.me.clone()
    }

    fn on_system(&self, sel: u32) {
        match sel {
            kAudioHardwarePropertyDevices => {
                self.emit(PlatformEvent::DevicesChanged);
                self.resync_devices();
                self.reevaluate();
            }
            kAudioHardwarePropertyDefaultOutputDevice => {
                let d = default_output_id();
                self.inner
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .default_out = d;
                self.emit(PlatformEvent::DefaultOutputChanged(
                    device_uid(d).map(DeviceId),
                ));
                self.resync_volume();
                self.emit_volume(d);
            }
            kAudioHardwarePropertyServiceRestarted => {
                // Every object id may have changed: rebuild everything.
                let d = default_output_id();
                self.inner
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .default_out = d;
                self.emit(PlatformEvent::DevicesChanged);
                self.emit(PlatformEvent::DefaultOutputChanged(
                    device_uid(d).map(DeviceId),
                ));
                self.resync_devices();
                self.resync_processes();
                self.resync_volume();
                self.reevaluate();
            }
            kAudioHardwarePropertyProcessObjectList => {
                self.resync_processes();
                self.reevaluate();
            }
            _ => {}
        }
    }

    fn on_device(&self, uid: &str, sel: u32) {
        match sel {
            kAudioDevicePropertyDeviceIsAlive | kAudioDevicePropertyHogMode => {
                self.emit(PlatformEvent::DevicesChanged);
            }
            kAudioDevicePropertyDeviceIsRunningSomewhere => self.reevaluate(),
            _ => {
                self.emit(PlatformEvent::DeviceFormatChanged(DeviceId(
                    uid.to_string(),
                )));
                if sel == kAudioDevicePropertyStreams {
                    self.resync_devices();
                }
                self.reevaluate();
            }
        }
    }

    fn emit_volume(&self, dev: AudioObjectID) {
        let Some(uid) = device_uid(dev) else { return };
        self.emit(PlatformEvent::VolumeChanged {
            id: DeviceId(uid),
            scalar: volume::scalar(dev),
            muted: volume::muted(dev),
        });
    }

    /// Recomputes hands-free modes and "system audio active"; emits changes.
    fn reevaluate(&self) {
        let modes = current_modes();
        let active = hfp::other_process_running_output();
        let mut events = Vec::new();
        {
            let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
            for (uid, (mode, occ)) in &modes {
                let prev = g.modes.get(uid);
                let changed = match prev {
                    Some((pm, po)) => {
                        pm != mode || (*mode == DeviceMode::BluetoothHandsFree && po != occ)
                    }
                    None => *mode == DeviceMode::BluetoothHandsFree,
                };
                if changed {
                    events.push(PlatformEvent::DeviceModeChanged {
                        id: DeviceId(uid.clone()),
                        mode: *mode,
                        occupant: occ.clone(),
                    });
                }
            }
            g.modes = modes;
            if g.audio_active != active {
                g.audio_active = active;
                events.push(PlatformEvent::SystemAudioActive(active));
            }
        }
        for e in events {
            self.emit(e);
        }
    }
}

impl Watcher {
    /// Re-registers the per-device listeners for every output-capable device.
    fn resync_devices(&self) {
        let weak = self.self_weak();
        let mut new = Vec::new();
        for d in system_devices() {
            let fi_ok = devices::filter(&devices::filter_input(d, SCOPE_OUTPUT), true).is_none();
            if !fi_ok {
                continue;
            }
            let Some(uid) = device_uid(d) else { continue };
            let mut addrs: Vec<(AudioObjectID, AudioObjectPropertyAddress)> = FORMAT_SELECTORS
                .iter()
                .map(|&(s, sc)| (d, addr(s, sc)))
                .collect();
            addrs.push((d, addr(kAudioDevicePropertyDeviceIsAlive, SCOPE_GLOBAL)));
            addrs.push((d, addr(kAudioDevicePropertyHogMode, SCOPE_GLOBAL)));
            addrs.push((
                d,
                addr(kAudioDevicePropertyDeviceIsRunningSomewhere, SCOPE_GLOBAL),
            ));
            for scope in [SCOPE_OUTPUT, SCOPE_INPUT] {
                for s in get_vec::<AudioObjectID>(d, kAudioDevicePropertyStreams, scope)
                    .unwrap_or_default()
                {
                    addrs.push((s, addr(kAudioStreamPropertyVirtualFormat, SCOPE_GLOBAL)));
                    addrs.push((s, addr(kAudioStreamPropertyLatency, SCOPE_GLOBAL)));
                }
            }
            for (o, a) in addrs {
                let wk = weak.clone();
                let u = uid.clone();
                if let Ok(l) = Listener::add(&self.shared.queue, o, a, move |s| {
                    with(&wk, |w| w.on_device(&u, s))
                }) {
                    new.push(l);
                }
            }
        }
        let old = std::mem::replace(
            &mut self.inner.lock().unwrap_or_else(|e| e.into_inner()).devices,
            new,
        );
        drop(old);
    }

    /// Re-registers IsRunningInput / input Devices / IsRunningOutput on every process object.
    fn resync_processes(&self) {
        let weak = self.self_weak();
        let mut new = Vec::new();
        for p in hfp::process_objects() {
            for (sel, scope) in [
                (kAudioProcessPropertyIsRunningInput, SCOPE_GLOBAL),
                (kAudioProcessPropertyDevices, SCOPE_INPUT),
                (kAudioProcessPropertyIsRunningOutput, SCOPE_GLOBAL),
            ] {
                let wk = weak.clone();
                if let Ok(l) = Listener::add(&self.shared.queue, p, addr(sel, scope), move |_| {
                    with(&wk, |w| w.reevaluate())
                }) {
                    new.push(l);
                }
            }
        }
        let old = std::mem::replace(
            &mut self
                .inner
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .processes,
            new,
        );
        drop(old);
    }

    /// Volume / mute listeners on the current default output only.
    fn resync_volume(&self) {
        let weak = self.self_weak();
        let d = self
            .inner
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .default_out;
        let mut new = Vec::new();
        if d != kAudioObjectUnknown {
            for sel in [kAudioDevicePropertyVolumeScalar, kAudioDevicePropertyMute] {
                for el in volume::ELEMENTS {
                    if !has(d, sel, SCOPE_OUTPUT, el) {
                        continue;
                    }
                    let wk = weak.clone();
                    if let Ok(l) = Listener::add(
                        &self.shared.queue,
                        d,
                        addr_el(sel, SCOPE_OUTPUT, el),
                        move |_| {
                            with(&wk, |w| {
                                let cur = w
                                    .inner
                                    .lock()
                                    .unwrap_or_else(|e| e.into_inner())
                                    .default_out;
                                if cur == d {
                                    w.emit_volume(d);
                                }
                            })
                        },
                    ) {
                        new.push(l);
                    }
                }
            }
        }
        let old = std::mem::replace(
            &mut self.inner.lock().unwrap_or_else(|e| e.into_inner()).volume,
            new,
        );
        drop(old);
    }
}
