//! Device enumeration, filtering and `DeviceInfo` construction.

use std::time::Duration;

use wye_platform::{DeviceId, DeviceInfo, DeviceMode, ReportedLatency, Transport};

use crate::hal::*;
use crate::hfp;

/// Sample rates at or below this value on a Bluetooth output indicate the hands-free profile.
pub const HFP_MAX_RATE: f64 = 24_000.0;

pub fn transport_from_code(code: u32) -> Transport {
    match &code.to_be_bytes() {
        b"bltn" => Transport::BuiltIn,
        b"usb " => Transport::Usb,
        b"blue" => Transport::Bluetooth,
        b"blea" => Transport::BluetoothLe,
        b"hdmi" => Transport::Hdmi,
        b"dprt" => Transport::DisplayPort,
        b"airp" => Transport::AirPlay,
        b"thun" => Transport::Thunderbolt,
        b"pci " => Transport::Pci,
        b"virt" => Transport::Virtual,
        b"grup" | b"fgrp" => Transport::Aggregate,
        _ => Transport::Other,
    }
}

pub fn transport_code(dev: AudioObjectID) -> u32 {
    get(dev, kAudioDevicePropertyTransportType, SCOPE_GLOBAL).unwrap_or(0)
}

pub fn transport(dev: AudioObjectID) -> Transport {
    transport_from_code(transport_code(dev))
}

pub fn is_bluetooth(t: Transport) -> bool {
    matches!(t, Transport::Bluetooth | Transport::BluetoothLe)
}

/// Hands-free heuristic from the format alone: a Bluetooth output running at ≤ 24 kHz or with a
/// single output channel.
pub fn format_suggests_hands_free(t: Transport, rate: f64, out_channels: u32) -> bool {
    is_bluetooth(t) && ((rate > 0.0 && rate <= HFP_MAX_RATE) || out_channels == 1)
}

/// Why a device is not offered (first matching filter, in the specified order).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Skip {
    Aggregate,
    Hidden,
    NoChannels,
    NotAlive,
    Virtual,
}

/// Raw attributes used by the filters (kept separate so the order is unit-testable).
#[derive(Clone, Copy, Debug)]
pub struct FilterInput {
    pub class: u32,
    pub transport: u32,
    pub hidden: bool,
    pub channels: u32,
    pub alive: bool,
}

pub fn filter(f: &FilterInput, show_virtual: bool) -> Option<Skip> {
    let tr = transport_from_code(f.transport);
    if f.class == kAudioAggregateDeviceClassID || tr == Transport::Aggregate {
        return Some(Skip::Aggregate);
    }
    if f.hidden {
        return Some(Skip::Hidden);
    }
    if f.channels == 0 {
        return Some(Skip::NoChannels);
    }
    if !f.alive {
        return Some(Skip::NotAlive);
    }
    if tr == Transport::Virtual && !show_virtual {
        return Some(Skip::Virtual);
    }
    None
}

pub fn filter_input(dev: AudioObjectID, scope: u32) -> FilterInput {
    FilterInput {
        class: get(dev, kAudioObjectPropertyClass, SCOPE_GLOBAL).unwrap_or(0),
        transport: transport_code(dev),
        hidden: get::<u32>(dev, kAudioDevicePropertyIsHidden, SCOPE_GLOBAL).unwrap_or(0) != 0,
        channels: channels(dev, scope),
        alive: get::<u32>(dev, kAudioDevicePropertyDeviceIsAlive, SCOPE_GLOBAL).unwrap_or(0) != 0,
    }
}

/// True if another process holds the device in hog (exclusive) mode.
pub fn hogged_by_other(dev: AudioObjectID) -> bool {
    match get::<i32>(dev, kAudioDevicePropertyHogMode, SCOPE_GLOBAL) {
        Ok(pid) => pid != -1 && pid != std::process::id() as i32,
        Err(_) => false,
    }
}

fn duration_s(s: f64) -> Duration {
    Duration::from_secs_f64(s.max(0.0))
}

/// Builds the `DeviceInfo` of a device for `scope` (output for mirrors, input for microphones;
/// for inputs `output_channels` carries the input channel count and the latency is the input
/// latency).
pub fn device_info(dev: AudioObjectID, scope: u32) -> Option<DeviceInfo> {
    let uid = device_uid(dev)?;
    let name = get_string(dev, kAudioObjectPropertyName, SCOPE_GLOBAL).unwrap_or_default();
    let t = transport(dev);
    let lat = Latency::read(dev, scope);
    let ch = channels(dev, scope);
    let can_be_default =
        get::<u32>(dev, kAudioDevicePropertyDeviceCanBeDefaultDevice, scope).unwrap_or(0) != 0;
    let mode = if scope == SCOPE_OUTPUT
        && (format_suggests_hands_free(t, lat.rate, ch) || hfp::occupant_of(dev).is_some())
    {
        DeviceMode::BluetoothHandsFree
    } else {
        DeviceMode::Normal
    };
    Some(DeviceInfo {
        id: DeviceId(uid),
        name,
        transport: t,
        nominal_rate: lat.rate,
        output_channels: ch.min(u16::MAX as u32) as u16,
        mode,
        reported_latency: ReportedLatency {
            after_timestamp: duration_s(lat.after_timestamp_s()),
            total: duration_s(lat.total_s()),
        },
        physical_group: None,
        can_be_default,
        buffer_frames: lat.buffer,
        busy: hogged_by_other(dev),
    })
}

/// Output devices usable by Wye, with their object ids.
pub fn output_devices(show_virtual: bool) -> Vec<(AudioObjectID, DeviceInfo)> {
    system_devices()
        .into_iter()
        .filter(|&d| filter(&filter_input(d, SCOPE_OUTPUT), show_virtual).is_none())
        .filter_map(|d| device_info(d, SCOPE_OUTPUT).map(|i| (d, i)))
        .collect()
}

fn calibration_rank(t: Transport) -> u8 {
    match t {
        Transport::BuiltIn => 0,
        Transport::Usb => 1,
        _ => 2,
    }
}

/// Input devices that are safe for calibration: never Bluetooth (opening a headset microphone
/// switches it to the hands-free profile), built-in first, then USB, then the rest.
pub fn calibration_inputs(show_virtual: bool) -> Vec<(AudioObjectID, DeviceInfo)> {
    let mut v: Vec<(AudioObjectID, DeviceInfo)> = system_devices()
        .into_iter()
        .filter(|&d| filter(&filter_input(d, SCOPE_INPUT), show_virtual).is_none())
        // Never a Bluetooth input (this also excludes a Bluetooth default input).
        .filter(|&d| !is_bluetooth(transport(d)))
        .filter_map(|d| device_info(d, SCOPE_INPUT).map(|i| (d, i)))
        .collect();
    v.sort_by_key(|(_, i)| calibration_rank(i.transport));
    v
}

/// Current output channel layout used to render stereo: (left, right) 1-based device channels.
pub fn preferred_stereo(dev: AudioObjectID) -> (u32, u32) {
    get::<[u32; 2]>(
        dev,
        kAudioDevicePropertyPreferredChannelsForStereo,
        SCOPE_OUTPUT,
    )
    .ok()
    .filter(|c| c[0] > 0 && c[1] > 0)
    .map(|c| (c[0], c[1]))
    .unwrap_or((1, 2))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fi(class: &[u8; 4], tr: &[u8; 4], hidden: bool, ch: u32, alive: bool) -> FilterInput {
        FilterInput {
            class: fcc(class),
            transport: fcc(tr),
            hidden,
            channels: ch,
            alive,
        }
    }

    #[test]
    fn filter_order_matches_spec() {
        // Aggregate wins over everything else.
        assert_eq!(
            filter(&fi(b"aagg", b"bltn", true, 0, false), true),
            Some(Skip::Aggregate)
        );
        assert_eq!(
            filter(&fi(b"adev", b"grup", true, 0, false), true),
            Some(Skip::Aggregate)
        );
        assert_eq!(
            filter(&fi(b"adev", b"usb ", true, 0, false), true),
            Some(Skip::Hidden)
        );
        assert_eq!(
            filter(&fi(b"adev", b"usb ", false, 0, false), true),
            Some(Skip::NoChannels)
        );
        assert_eq!(
            filter(&fi(b"adev", b"usb ", false, 2, false), true),
            Some(Skip::NotAlive)
        );
        assert_eq!(
            filter(&fi(b"adev", b"virt", false, 2, true), false),
            Some(Skip::Virtual)
        );
        assert_eq!(filter(&fi(b"adev", b"virt", false, 2, true), true), None);
        assert_eq!(filter(&fi(b"adev", b"blue", false, 2, true), false), None);
    }

    #[test]
    fn transport_codes() {
        assert_eq!(transport_from_code(fcc(b"bltn")), Transport::BuiltIn);
        assert_eq!(transport_from_code(fcc(b"blea")), Transport::BluetoothLe);
        assert_eq!(transport_from_code(fcc(b"dprt")), Transport::DisplayPort);
        assert_eq!(transport_from_code(fcc(b"pci ")), Transport::Pci);
        assert_eq!(transport_from_code(fcc(b"thun")), Transport::Thunderbolt);
        assert_eq!(transport_from_code(fcc(b"grup")), Transport::Aggregate);
        assert_eq!(transport_from_code(fcc(b"xxxx")), Transport::Other);
    }

    #[test]
    fn hands_free_heuristic() {
        assert!(format_suggests_hands_free(
            Transport::Bluetooth,
            16_000.0,
            1
        ));
        assert!(format_suggests_hands_free(
            Transport::Bluetooth,
            24_000.0,
            2
        ));
        assert!(format_suggests_hands_free(
            Transport::BluetoothLe,
            48_000.0,
            1
        ));
        assert!(!format_suggests_hands_free(
            Transport::Bluetooth,
            48_000.0,
            2
        ));
        assert!(!format_suggests_hands_free(Transport::Usb, 16_000.0, 1));
    }

    #[test]
    fn calibration_rank_order() {
        let mut v = [Transport::Usb, Transport::Other, Transport::BuiltIn];
        v.sort_by_key(|&t| calibration_rank(t));
        assert_eq!(v, [Transport::BuiltIn, Transport::Usb, Transport::Other]);
    }
}
