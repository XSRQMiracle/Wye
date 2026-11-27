//! Bluetooth hands-free (HFP/HSP) detection through Core Audio process objects.
//!
//! A Bluetooth headset switches to the hands-free profile when some process opens its
//! microphone. The HAL exposes, per client process, whether it runs input IO and which devices it
//! uses for input; a non-Wye process running input on a Bluetooth device therefore marks that
//! device as hands-free. `DeviceIsRunningSomewhere` cannot be used: it ignores the scope.

use std::collections::HashMap;

use objc2::msg_send;
use objc2::rc::Retained;
use objc2::runtime::{AnyClass, AnyObject};
use objc2_foundation::NSString;

use crate::devices::{is_bluetooth, transport};
use crate::hal::*;

#[link(name = "AppKit", kind = "framework")]
extern "C" {}

pub fn process_objects() -> Vec<AudioObjectID> {
    get_vec(
        kAudioObjectSystemObject,
        kAudioHardwarePropertyProcessObjectList,
        SCOPE_GLOBAL,
    )
    .unwrap_or_default()
}

fn process_pid(p: AudioObjectID) -> Option<i32> {
    get(p, kAudioProcessPropertyPID, SCOPE_GLOBAL).ok()
}

fn is_own(p: AudioObjectID) -> bool {
    process_pid(p) == Some(std::process::id() as i32)
}

fn running_input(p: AudioObjectID) -> bool {
    get::<u32>(p, kAudioProcessPropertyIsRunningInput, SCOPE_GLOBAL).unwrap_or(0) != 0
}

fn running_output(p: AudioObjectID) -> bool {
    get::<u32>(p, kAudioProcessPropertyIsRunningOutput, SCOPE_GLOBAL).unwrap_or(0) != 0
}

fn input_devices(p: AudioObjectID) -> Vec<AudioObjectID> {
    get_vec(p, kAudioProcessPropertyDevices, SCOPE_INPUT).unwrap_or_default()
}

/// Display name of the app behind a process object: `NSRunningApplication.localizedName`, else
/// the bundle id, else the pid.
pub fn process_display_name(p: AudioObjectID) -> String {
    let pid = process_pid(p);
    if let Some(pid) = pid {
        if let Some(name) = running_app_name(pid) {
            return name;
        }
    }
    if let Ok(b) = get_string(p, kAudioProcessPropertyBundleID, SCOPE_GLOBAL) {
        if !b.is_empty() {
            return b;
        }
    }
    match pid {
        Some(pid) => format!("pid {pid}"),
        None => String::from("?"),
    }
}

fn running_app_name(pid: i32) -> Option<String> {
    let cls = AnyClass::get(c"NSRunningApplication")?;
    // SAFETY: documented class method / property with these signatures.
    unsafe {
        let app: Option<Retained<AnyObject>> =
            msg_send![cls, runningApplicationWithProcessIdentifier: pid];
        let app = app?;
        let name: Option<Retained<NSString>> = msg_send![&*app, localizedName];
        name.map(|n| n.to_string()).filter(|s| !s.is_empty())
    }
}

/// Bluetooth devices whose microphone is held by another process → occupant display name.
pub fn bluetooth_occupancy() -> HashMap<AudioObjectID, String> {
    let mut out = HashMap::new();
    for p in process_objects() {
        if is_own(p) || !running_input(p) {
            continue;
        }
        for d in input_devices(p) {
            if is_bluetooth(transport(d)) && !out.contains_key(&d) {
                out.insert(d, process_display_name(p));
            }
        }
    }
    out
}

/// Occupant of one device's microphone (another process running input on it), if Bluetooth.
pub fn occupant_of(dev: AudioObjectID) -> Option<String> {
    if !is_bluetooth(transport(dev)) {
        return None;
    }
    process_objects()
        .into_iter()
        .filter(|&p| !is_own(p) && running_input(p))
        .find(|&p| input_devices(p).contains(&dev))
        .map(process_display_name)
}

/// Whether any process other than Wye currently runs output IO (best effort).
pub fn other_process_running_output() -> bool {
    process_objects()
        .into_iter()
        .any(|p| !is_own(p) && running_output(p))
}
