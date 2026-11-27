//! Platform-independent endpoint rules: transport inference from device instance IDs, physical
//! grouping by container ID, which endpoints to show, calibration microphone selection, the
//! hands-free format heuristic, default-device change de-duplication, and error mapping.

use wye_platform::{StreamError, Transport};

/// `EndpointFormFactor::Microphone`.
pub const FORM_FACTOR_MICROPHONE: i32 = 4;
/// `EndpointFormFactor::DigitalAudioDisplayDevice` (HDMI / DisplayPort).
pub const FORM_FACTOR_DIGITAL_DISPLAY: i32 = 9;

/// The "Computer" container shared by every on-board device; it does not identify a product.
pub const COMPUTER_CONTAINER: u128 = 0x00000000_0000_0000_FFFF_FFFFFFFFFFFF;

/// HRESULT values used for error mapping (as `i32`, the representation of `HRESULT`).
pub mod hr {
    pub const AUDCLNT_E_DEVICE_INVALIDATED: i32 = 0x8889_0004_u32 as i32;
    pub const AUDCLNT_E_RESOURCES_INVALIDATED: i32 = 0x8889_0026_u32 as i32;
    pub const AUDCLNT_E_DEVICE_IN_USE: i32 = 0x8889_000A_u32 as i32;
    pub const AUDCLNT_E_EXCLUSIVE_MODE_NOT_ALLOWED: i32 = 0x8889_000E_u32 as i32;
    pub const AUDCLNT_E_UNSUPPORTED_FORMAT: i32 = 0x8889_0008_u32 as i32;
    pub const E_ACCESSDENIED: i32 = 0x8007_0005_u32 as i32;
    /// `HRESULT_FROM_WIN32(ERROR_NOT_FOUND)`, returned by `GetDevice` for unknown IDs.
    pub const E_NOTFOUND: i32 = 0x8007_0490_u32 as i32;
}

/// What the device instance ID says about an endpoint.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EndpointClass {
    pub transport: Transport,
    /// Windows 10 Bluetooth hands-free (HFP) endpoint (`BTHHFENUM\`).
    pub hands_free: bool,
}

/// Infers the transport from the first recognisable candidate: the instance ID of the device the
/// endpoint's topology connects to (e.g. `USB\VID_…`, `BTHENUM\{…}`), or the endpoint's
/// enumerator name (`USB`, `BTHENUM`, …).
///
/// `BTHENUM` → Bluetooth (A2DP), `BTHHFENUM` → Bluetooth hands-free, `BTHLEDevice` → Bluetooth LE,
/// `USB` → USB, `HDAUDIO` with the digital-display form factor → HDMI, everything else → Other.
pub fn classify_endpoint(candidates: &[&str], form_factor: i32) -> EndpointClass {
    for c in candidates {
        let bus = c.split('\\').next().unwrap_or("").trim();
        let class = if bus.eq_ignore_ascii_case("BTHHFENUM") {
            Some((Transport::Bluetooth, true))
        } else if bus.eq_ignore_ascii_case("BTHENUM") {
            Some((Transport::Bluetooth, false))
        } else if bus.eq_ignore_ascii_case("BTHLEDEVICE") {
            Some((Transport::BluetoothLe, false))
        } else if bus.eq_ignore_ascii_case("USB") {
            Some((Transport::Usb, false))
        } else if bus.eq_ignore_ascii_case("HDAUDIO") {
            Some((
                if form_factor == FORM_FACTOR_DIGITAL_DISPLAY {
                    Transport::Hdmi
                } else {
                    Transport::Other
                },
                false,
            ))
        } else {
            None
        };
        if let Some((transport, hands_free)) = class {
            return EndpointClass {
                transport,
                hands_free,
            };
        }
    }
    EndpointClass {
        transport: Transport::Other,
        hands_free: false,
    }
}

/// Formats a GUID given as `u128` in registry form: `{XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX}`.
pub fn guid_string(g: u128) -> String {
    format!(
        "{{{:08X}-{:04X}-{:04X}-{:04X}-{:012X}}}",
        (g >> 96) as u32,
        (g >> 80) as u16,
        (g >> 64) as u16,
        (g >> 48) as u16,
        g & 0xFFFF_FFFF_FFFF
    )
}

/// `DeviceInfo::physical_group` for a container ID: `None` for the shared Computer container
/// (and the null GUID), otherwise the GUID string.
pub fn physical_group(container: Option<u128>) -> Option<String> {
    match container {
        None | Some(0) | Some(COMPUTER_CONTAINER) => None,
        Some(g) => Some(guid_string(g)),
    }
}

/// Minimal view of an endpoint for the filtering rules below.
#[derive(Clone, Debug)]
pub struct EndpointView<'a> {
    pub class: EndpointClass,
    pub group: Option<&'a str>,
}

/// Which render endpoints are listed. Windows 10 exposes a Bluetooth headset as a "Stereo"
/// (A2DP) and a "Hands-Free" endpoint in the same container; only the Stereo one is listed.
/// A hands-free endpoint is hidden only when such a sibling exists, so a unified Windows 11
/// endpoint is never hidden by this rule.
pub fn output_visible(all: &[EndpointView<'_>], index: usize) -> bool {
    let Some(me) = all.get(index) else {
        return false;
    };
    if !me.class.hands_free {
        return true;
    }
    let Some(group) = me.group else {
        return true;
    };
    !all.iter().enumerate().any(|(i, other)| {
        i != index
            && !other.class.hands_free
            && other.group == Some(group)
            && matches!(
                other.class.transport,
                Transport::Bluetooth | Transport::BluetoothLe
            )
    })
}

/// Bluetooth headsets switch to the hands-free profile at 8/16/24 kHz, usually mono.
pub fn is_hands_free_format(rate: u32, channels: u16) -> bool {
    rate <= 24_000 || channels == 1
}

/// A capture endpoint candidate for calibration.
#[derive(Clone, Debug)]
pub struct InputCandidate<'a> {
    pub class: EndpointClass,
    pub group: Option<&'a str>,
    pub form_factor: i32,
    /// The endpoint belongs to the Computer container (on-board).
    pub onboard: bool,
}

/// Returns the indices of `inputs` usable for calibration, best first. Bluetooth microphones and
/// any microphone sharing a container with a Bluetooth output are excluded, because opening them
/// switches the headset to the hands-free profile. On-board microphones come first, then other
/// microphones, then everything else; ties keep enumeration order.
pub fn rank_calibration_inputs(
    inputs: &[InputCandidate<'_>],
    bluetooth_output_groups: &[&str],
) -> Vec<usize> {
    let mut keep: Vec<usize> = inputs
        .iter()
        .enumerate()
        .filter(|(_, c)| {
            !matches!(
                c.class.transport,
                Transport::Bluetooth | Transport::BluetoothLe
            ) && !c.class.hands_free
                && !c
                    .group
                    .is_some_and(|g| bluetooth_output_groups.contains(&g))
        })
        .map(|(i, _)| i)
        .collect();
    keep.sort_by_key(|&i| {
        let c = &inputs[i];
        let mic = c.form_factor == FORM_FACTOR_MICROPHONE;
        match (mic && c.onboard, mic, c.onboard) {
            (true, _, _) => 0,
            (false, true, _) => 1,
            (false, false, true) => 2,
            _ => 3,
        }
    });
    keep
}

/// `OnDefaultDeviceChanged` arrives once per role; Wye follows only `eRender`/`eConsole`
/// (`EDataFlow` 0, `ERole` 0).
pub fn is_tracked_default_change(flow: i32, role: i32) -> bool {
    flow == 0 && role == 0
}

/// De-duplicates default-output notifications (the default can also be re-queried after
/// add/remove/state changes, which may or may not have produced a callback).
#[derive(Clone, Debug, Default)]
pub struct DefaultTracker {
    last: Option<Option<String>>,
}

impl DefaultTracker {
    /// Records the current default without reporting it.
    pub fn prime(&mut self, current: Option<&str>) {
        self.last = Some(current.map(str::to_owned));
    }

    /// Returns `Some(new)` if the default differs from the last reported/primed one.
    pub fn observe(&mut self, current: Option<&str>) -> Option<Option<String>> {
        let current = current.map(str::to_owned);
        if self.last.as_ref() == Some(&current) {
            return None;
        }
        self.last = Some(current.clone());
        Some(current)
    }
}

/// Maps a failed WASAPI call to a stream error.
pub fn stream_error_from_hresult(code: i32) -> StreamError {
    match code {
        hr::AUDCLNT_E_DEVICE_INVALIDATED | hr::AUDCLNT_E_RESOURCES_INVALIDATED | hr::E_NOTFOUND => {
            StreamError::DeviceGone
        }
        hr::AUDCLNT_E_DEVICE_IN_USE | hr::AUDCLNT_E_EXCLUSIVE_MODE_NOT_ALLOWED => {
            StreamError::DeviceBusy
        }
        other => StreamError::Os(other as i64),
    }
}

/// Maps an `AudioSessionDisconnectReason` to a stream error.
pub fn stream_error_from_disconnect(reason: i32) -> StreamError {
    match reason {
        // DeviceRemoval, SessionLogoff, SessionDisconnected (remote desktop)
        0 | 3 | 4 => StreamError::DeviceGone,
        // FormatChanged
        2 => StreamError::FormatChanged,
        // ExclusiveModeOverride
        5 => StreamError::DeviceBusy,
        // ServerShutdown and anything new
        other => StreamError::Os(other as i64),
    }
}

/// True for a Remote Desktop session (`SESSIONNAME` is `RDP-Tcp#N`; the console is `Console`).
/// Audio there only reaches the client's "Remote Audio" endpoint, so multi-device sync is moot.
pub fn is_remote_session_name(session_name: Option<&str>) -> bool {
    session_name.is_some_and(|s| {
        s.get(..4)
            .is_some_and(|prefix| prefix.eq_ignore_ascii_case("RDP-"))
    })
}

/// File stem of an executable path (`C:\\…\\Teams.exe` → `Teams`), for session occupant names.
pub fn process_display_name(path: &str) -> Option<String> {
    let file = path.rsplit(['\\', '/']).next()?;
    let stem = match file.rfind('.') {
        Some(dot) if dot > 0 => &file[..dot],
        _ => file,
    };
    (!stem.is_empty()).then(|| stem.to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn class(t: Transport, hf: bool) -> EndpointClass {
        EndpointClass {
            transport: t,
            hands_free: hf,
        }
    }

    #[test]
    fn classifies_instance_ids() {
        let c = |s: &str, ff| classify_endpoint(&[s], ff);
        assert_eq!(
            c(r"USB\VID_046D&PID_09A6&MI_02\7&1", 1),
            class(Transport::Usb, false)
        );
        assert_eq!(
            c(
                r"BTHENUM\{0000110b-0000-1000-8000-00805f9b34fb}_VID&0002",
                3
            ),
            class(Transport::Bluetooth, false)
        );
        assert_eq!(
            c(r"BTHHFENUM\BthHFPAudio\8&2", 5),
            class(Transport::Bluetooth, true)
        );
        assert_eq!(
            c(r"BTHLEDevice\{0000184e}", 3),
            class(Transport::BluetoothLe, false)
        );
        assert_eq!(
            c(r"HDAUDIO\FUNC_01&VEN_10DE", 9),
            class(Transport::Hdmi, false)
        );
        assert_eq!(
            c(r"HDAUDIO\FUNC_01&VEN_10EC", 1),
            class(Transport::Other, false)
        );
        assert_eq!(
            c(r"SWD\MMDEVAPI\{0.0.0}", 1),
            class(Transport::Other, false)
        );
        assert_eq!(c("", 1), class(Transport::Other, false));
    }

    #[test]
    fn falls_back_to_later_candidates() {
        assert_eq!(
            classify_endpoint(&[r"INTELAUDIO\FUNC_01", "bthenum"], 3),
            class(Transport::Bluetooth, false)
        );
        assert_eq!(
            classify_endpoint(&["USB", r"BTHENUM\x"], 1),
            class(Transport::Usb, false)
        );
    }

    #[test]
    fn groups_by_container_except_computer() {
        assert_eq!(physical_group(Some(COMPUTER_CONTAINER)), None);
        assert_eq!(physical_group(None), None);
        assert_eq!(physical_group(Some(0)), None);
        assert_eq!(
            physical_group(Some(0x12345678_9abc_def0_1122_334455667788)).as_deref(),
            Some("{12345678-9ABC-DEF0-1122-334455667788}")
        );
        assert_eq!(
            guid_string(COMPUTER_CONTAINER),
            "{00000000-0000-0000-FFFF-FFFFFFFFFFFF}"
        );
    }

    #[test]
    fn hides_hands_free_endpoint_with_stereo_sibling() {
        let g = "{A}";
        let all = [
            EndpointView {
                class: class(Transport::Bluetooth, false),
                group: Some(g),
            },
            EndpointView {
                class: class(Transport::Bluetooth, true),
                group: Some(g),
            },
            EndpointView {
                class: class(Transport::Bluetooth, true),
                group: Some("{B}"),
            },
            EndpointView {
                class: class(Transport::Other, false),
                group: None,
            },
        ];
        assert!(output_visible(&all, 0));
        assert!(!output_visible(&all, 1));
        assert!(
            output_visible(&all, 2),
            "no stereo sibling (unified endpoint)"
        );
        assert!(output_visible(&all, 3));
        assert!(!output_visible(&all, 9));
    }

    #[test]
    fn hands_free_format_heuristic() {
        assert!(is_hands_free_format(16_000, 1));
        assert!(is_hands_free_format(24_000, 2));
        assert!(is_hands_free_format(48_000, 1));
        assert!(!is_hands_free_format(48_000, 2));
        assert!(!is_hands_free_format(44_100, 2));
    }

    #[test]
    fn calibration_inputs_exclude_bluetooth_and_rank_onboard_mic_first() {
        let bt_group = "{HEADSET}";
        let inputs = [
            InputCandidate {
                class: class(Transport::Usb, false),
                group: Some("{USBMIC}"),
                form_factor: FORM_FACTOR_MICROPHONE,
                onboard: false,
            },
            InputCandidate {
                class: class(Transport::Bluetooth, true),
                group: Some(bt_group),
                form_factor: 5,
                onboard: false,
            },
            InputCandidate {
                // DSP-masked instance ID but same container as the BT headset output.
                class: class(Transport::Other, false),
                group: Some(bt_group),
                form_factor: 5,
                onboard: false,
            },
            InputCandidate {
                class: class(Transport::Other, false),
                group: None,
                form_factor: 2, // line in
                onboard: true,
            },
            InputCandidate {
                class: class(Transport::Other, false),
                group: None,
                form_factor: FORM_FACTOR_MICROPHONE,
                onboard: true,
            },
            InputCandidate {
                class: class(Transport::BluetoothLe, false),
                group: Some("{LE}"),
                form_factor: 5,
                onboard: false,
            },
        ];
        assert_eq!(rank_calibration_inputs(&inputs, &[bt_group]), vec![4, 0, 3]);
    }

    #[test]
    fn default_changes_are_deduplicated() {
        assert!(is_tracked_default_change(0, 0));
        assert!(!is_tracked_default_change(0, 1));
        assert!(!is_tracked_default_change(0, 2));
        assert!(!is_tracked_default_change(1, 0));
        let mut t = DefaultTracker::default();
        t.prime(Some("a"));
        assert_eq!(t.observe(Some("a")), None);
        assert_eq!(t.observe(Some("b")), Some(Some("b".into())));
        assert_eq!(t.observe(Some("b")), None);
        assert_eq!(t.observe(None), Some(None));
        assert_eq!(t.observe(None), None);
        let mut fresh = DefaultTracker::default();
        assert_eq!(fresh.observe(None), Some(None));
    }

    #[test]
    fn maps_errors() {
        assert_eq!(
            stream_error_from_hresult(hr::AUDCLNT_E_DEVICE_INVALIDATED),
            StreamError::DeviceGone
        );
        assert_eq!(
            stream_error_from_hresult(hr::AUDCLNT_E_DEVICE_IN_USE),
            StreamError::DeviceBusy
        );
        assert_eq!(stream_error_from_hresult(-5), StreamError::Os(-5));
        assert_eq!(stream_error_from_disconnect(0), StreamError::DeviceGone);
        assert_eq!(stream_error_from_disconnect(2), StreamError::FormatChanged);
        assert_eq!(stream_error_from_disconnect(5), StreamError::DeviceBusy);
        assert_eq!(stream_error_from_disconnect(1), StreamError::Os(1));
    }

    #[test]
    fn detects_remote_sessions() {
        assert!(is_remote_session_name(Some("RDP-Tcp#3")));
        assert!(is_remote_session_name(Some("rdp-tcp#0")));
        assert!(!is_remote_session_name(Some("Console")));
        assert!(!is_remote_session_name(Some("RD")));
        assert!(!is_remote_session_name(None));
    }

    #[test]
    fn process_names() {
        assert_eq!(
            process_display_name(r"C:\Program Files\Teams\ms-teams.exe").as_deref(),
            Some("ms-teams")
        );
        assert_eq!(process_display_name("zoom").as_deref(), Some("zoom"));
        assert_eq!(process_display_name(r"C:\x\").as_deref(), None);
    }
}
