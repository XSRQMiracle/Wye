//! System volume and mute of an output device (output scope; main element, else channels 1/2).

use crate::hal::*;

/// Elements whose volume / mute controls are consulted, in order of preference.
pub const ELEMENTS: [u32; 3] = [ELEMENT_MAIN, 1, 2];

fn read_f32(dev: AudioObjectID, sel: u32, el: u32) -> Option<f32> {
    if !has(dev, sel, SCOPE_OUTPUT, el) {
        return None;
    }
    get_el::<f32>(dev, sel, SCOPE_OUTPUT, el).ok()
}

fn read_u32(dev: AudioObjectID, sel: u32, el: u32) -> Option<u32> {
    if !has(dev, sel, SCOPE_OUTPUT, el) {
        return None;
    }
    get_el::<u32>(dev, sel, SCOPE_OUTPUT, el).ok()
}

/// Combines per-element readings: the main element if present, else the average of channels 1/2.
pub fn combine_scalar(main: Option<f32>, ch1: Option<f32>, ch2: Option<f32>) -> Option<f32> {
    if let Some(m) = main {
        return Some(m.clamp(0.0, 1.0));
    }
    match (ch1, ch2) {
        (Some(a), Some(b)) => Some(((a + b) * 0.5).clamp(0.0, 1.0)),
        (Some(a), None) | (None, Some(a)) => Some(a.clamp(0.0, 1.0)),
        (None, None) => None,
    }
}

/// Mute: main element if present, else muted only if every present channel is muted.
pub fn combine_mute(main: Option<u32>, ch1: Option<u32>, ch2: Option<u32>) -> Option<bool> {
    if let Some(m) = main {
        return Some(m != 0);
    }
    match (ch1, ch2) {
        (Some(a), Some(b)) => Some(a != 0 && b != 0),
        (Some(a), None) | (None, Some(a)) => Some(a != 0),
        (None, None) => None,
    }
}

pub fn scalar(dev: AudioObjectID) -> Option<f32> {
    combine_scalar(
        read_f32(dev, kAudioDevicePropertyVolumeScalar, ELEMENT_MAIN),
        read_f32(dev, kAudioDevicePropertyVolumeScalar, 1),
        read_f32(dev, kAudioDevicePropertyVolumeScalar, 2),
    )
}

pub fn muted(dev: AudioObjectID) -> Option<bool> {
    combine_mute(
        read_u32(dev, kAudioDevicePropertyMute, ELEMENT_MAIN),
        read_u32(dev, kAudioDevicePropertyMute, 1),
        read_u32(dev, kAudioDevicePropertyMute, 2),
    )
}

/// `(scalar, muted)` if the device has a volume control (mute defaults to false when absent).
pub fn output_volume(dev: AudioObjectID) -> Option<(f32, bool)> {
    let s = scalar(dev)?;
    Some((s, muted(dev).unwrap_or(false)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scalar_prefers_main_then_averages() {
        assert_eq!(combine_scalar(Some(0.3), Some(1.0), Some(1.0)), Some(0.3));
        assert_eq!(combine_scalar(None, Some(0.2), Some(0.6)), Some(0.4));
        assert_eq!(combine_scalar(None, Some(0.2), None), Some(0.2));
        assert_eq!(combine_scalar(None, None, None), None);
        assert_eq!(combine_scalar(Some(1.5), None, None), Some(1.0));
    }

    #[test]
    fn mute_rules() {
        assert_eq!(combine_mute(Some(1), Some(0), Some(0)), Some(true));
        assert_eq!(combine_mute(None, Some(1), Some(0)), Some(false));
        assert_eq!(combine_mute(None, Some(1), Some(1)), Some(true));
        assert_eq!(combine_mute(None, None, None), None);
    }
}
