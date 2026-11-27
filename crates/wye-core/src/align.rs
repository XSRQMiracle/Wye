//! Alignment planning: decides Reference vs Takeover mode and every device's target latency.
//! Pure function of the configuration — no I/O, no clocks — so it is fully unit-tested.
//!
//! All times are seconds. "λ" (lambda) = time from a frame's capture timestamp to the moment it
//! leaves a device acoustically.

use wye_platform::DeviceId;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum AlignPolicy {
    /// Reference mode when every checked device can be aligned, otherwise takeover (if supported).
    #[default]
    Auto,
    /// Never mute the system path; late mirrors play as early as possible.
    PreferReference,
    /// Always take over (mute the system path) when the platform supports it.
    PreferTakeover,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    /// The system plays the default device natively; Wye renders only the other checked devices.
    Reference,
    /// The captured audio is muted on the default device; Wye renders every checked device.
    Takeover,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AlignDevice {
    pub id: DeviceId,
    /// User wants this device to sound.
    pub checked: bool,
    /// Is the system default output.
    pub is_default: bool,
    /// Output timestamp → acoustic (calibrated value if available, else reported).
    pub after_timestamp: f64,
    /// buffer + safety offset: from the render callback to the output timestamp.
    pub lead: f64,
    /// Minimum extra buffering this device needs when rendered by Wye (§5.3: B_min).
    pub b_min: f64,
    /// User/calibration offset: positive = this device plays later.
    pub offset: f64,
}

impl AlignDevice {
    /// Smallest λ Wye can achieve when it renders this device.
    pub fn lambda_min(&self) -> f64 {
        self.lead + self.after_timestamp + self.b_min
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct AlignInput {
    pub devices: Vec<AlignDevice>,
    pub policy: AlignPolicy,
    /// The platform can mute the captured audio on the default device (macOS Process Tap).
    /// Windows: false. Without it, takeover is still used when the user unchecks the default
    /// device (e.g. made an unused endpoint the default), but that device keeps sounding.
    pub can_mute_capture: bool,
    /// Capture timestamp → default device's output timestamp (platform constant; 0 on macOS).
    pub capture_to_output_ts: f64,
    /// Allowed lateness before a mirror counts as "cannot align" (s). Default 0.002.
    pub tolerance: f64,
    /// Audio-later-than-video threshold for the lip-sync advice (s). Default 0.045.
    pub lipsync_threshold: f64,
    /// Largest λ Wye will target (ring capacity limit). Default 7.0 s.
    pub max_lambda: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Target {
    pub id: DeviceId,
    /// Target λ for Wye's renderer on this device.
    pub lambda: f64,
    /// How much later than intended the device will sound (0 when aligned).
    pub late_by: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Advice {
    /// A mirror cannot be delayed enough to match the reference; it plays `by` seconds late.
    MirrorLate { id: DeviceId, by: f64 },
    /// Audio reaches the listener `lag` seconds after the picture; making `suggest` the system
    /// default output fixes it.
    LipSync { lag: f64, suggest: DeviceId },
    /// The user unchecked the default device but the platform cannot mute it.
    CannotMuteDefault { id: DeviceId },
}

#[derive(Clone, Debug, PartialEq)]
pub struct AlignPlan {
    pub mode: Mode,
    /// Devices Wye renders (never contains the default device in Reference mode).
    pub targets: Vec<Target>,
    pub advice: Vec<Advice>,
}

/// λ of the system path on the default device (what players compensate for).
fn system_lambda(input: &AlignInput, def: &AlignDevice) -> f64 {
    input.capture_to_output_ts + def.after_timestamp
}

pub fn plan(input: &AlignInput) -> AlignPlan {
    let def = input.devices.iter().find(|d| d.is_default);
    let checked: Vec<&AlignDevice> = input.devices.iter().filter(|d| d.checked).collect();
    let mut advice = Vec::new();

    // Reference-mode targets (only meaningful with a default device that keeps playing).
    let cap = input.max_lambda;
    let reference = |advice: &mut Vec<Advice>| -> Vec<Target> {
        let Some(def) = def else { return Vec::new() };
        let d_ref = system_lambda(input, def);
        checked
            .iter()
            .filter(|d| !d.is_default)
            .map(|d| {
                let want = d_ref + d.offset - def.offset;
                let lam = want.max(d.lambda_min()).min(cap);
                let late = (want.max(d.lambda_min()) - want).max(0.0);
                if late > input.tolerance {
                    advice.push(Advice::MirrorLate {
                        id: d.id.clone(),
                        by: late,
                    });
                }
                Target {
                    id: d.id.clone(),
                    lambda: lam,
                    late_by: late.max(0.0),
                }
            })
            .collect()
    };
    let reference_feasible = def.is_some_and(|def| {
        def.checked && {
            let d_ref = system_lambda(input, def);
            checked
                .iter()
                .filter(|d| !d.is_default)
                .all(|d| d_ref + d.offset - def.offset + input.tolerance >= d.lambda_min())
        }
    });
    let default_unchecked = def.is_some_and(|d| !d.checked);

    let want_takeover = !checked.is_empty()
        && (default_unchecked
            || (input.can_mute_capture
                && match input.policy {
                    AlignPolicy::PreferReference => false,
                    AlignPolicy::PreferTakeover => true,
                    AlignPolicy::Auto => !reference_feasible,
                }));
    if default_unchecked && !input.can_mute_capture {
        if let Some(d) = def {
            advice.push(Advice::CannotMuteDefault { id: d.id.clone() });
        }
    }

    if !want_takeover {
        let targets = reference(&mut advice);
        // Lip-sync is exact on the reference; late mirrors were reported above. Suggest the
        // slowest device as default if one exists.
        if let Some(def) = def {
            if let Some(slowest) = checked
                .iter()
                .filter(|d| !d.is_default)
                .max_by(|a, b| (a.lambda_min() - a.offset).total_cmp(&(b.lambda_min() - b.offset)))
            {
                let lag = slowest.lambda_min()
                    - slowest.offset
                    - (system_lambda(input, def) - def.offset);
                if lag > input.tolerance {
                    advice.push(Advice::LipSync {
                        lag,
                        suggest: slowest.id.clone(),
                    });
                }
            }
        }
        return AlignPlan {
            mode: Mode::Reference,
            targets,
            advice,
        };
    }

    // Takeover: everyone at the common λ.
    let common = checked
        .iter()
        .map(|d| d.lambda_min() - d.offset)
        .fold(f64::NEG_INFINITY, f64::max);
    let targets = checked
        .iter()
        .map(|d| {
            let want = common + d.offset;
            let lam = want.min(cap);
            if want > cap {
                advice.push(Advice::MirrorLate {
                    id: d.id.clone(),
                    by: want - cap,
                });
            }
            Target {
                id: d.id.clone(),
                lambda: lam,
                late_by: 0.0,
            }
        })
        .collect();
    if let Some(def) = def {
        let lag = common + def.offset.min(0.0) - system_lambda(input, def);
        if lag > input.lipsync_threshold {
            let slowest = checked
                .iter()
                .max_by(|a, b| (a.lambda_min() - a.offset).total_cmp(&(b.lambda_min() - b.offset)))
                .unwrap();
            if !slowest.is_default {
                advice.push(Advice::LipSync {
                    lag,
                    suggest: slowest.id.clone(),
                });
            }
        }
    }
    AlignPlan {
        mode: Mode::Takeover,
        targets,
        advice,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dev(id: &str, default: bool, after: f64, offset: f64) -> AlignDevice {
        AlignDevice {
            id: DeviceId(id.into()),
            checked: true,
            is_default: default,
            after_timestamp: after,
            lead: 0.011,
            b_min: 0.026,
            offset,
        }
    }
    fn input(devices: Vec<AlignDevice>, policy: AlignPolicy, can_mute: bool) -> AlignInput {
        AlignInput {
            devices,
            policy,
            can_mute_capture: can_mute,
            capture_to_output_ts: 0.0,
            tolerance: 0.002,
            lipsync_threshold: 0.045,
            max_lambda: 7.0,
        }
    }

    #[test]
    fn slow_bt_reference_aligns_fast_speaker() {
        // BT headphones (200 ms) are default, built-in speaker (28 ms) is a mirror.
        let p = plan(&input(
            vec![dev("bt", true, 0.200, 0.0), dev("spk", false, 0.028, 0.0)],
            AlignPolicy::Auto,
            true,
        ));
        assert_eq!(p.mode, Mode::Reference);
        assert_eq!(p.targets.len(), 1);
        assert!((p.targets[0].lambda - 0.200).abs() < 1e-9);
        assert!(p.advice.is_empty(), "{:?}", p.advice);
    }

    #[test]
    fn fast_reference_forces_takeover_with_lipsync_advice() {
        // Speaker is default, BT is a mirror: reference mode impossible -> takeover.
        let p = plan(&input(
            vec![dev("spk", true, 0.028, 0.0), dev("bt", false, 0.200, 0.0)],
            AlignPolicy::Auto,
            true,
        ));
        assert_eq!(p.mode, Mode::Takeover);
        assert_eq!(p.targets.len(), 2);
        let l = p.targets[0].lambda;
        assert!(p.targets.iter().all(|t| (t.lambda - l).abs() < 1e-9));
        assert!(matches!(&p.advice[..], [Advice::LipSync { suggest, .. }] if suggest.0 == "bt"));
    }

    #[test]
    fn similar_devices_need_takeover() {
        // Two identical BT headsets: mirror path overhead (lead + B_min) makes reference infeasible.
        let p = plan(&input(
            vec![dev("a", true, 0.180, 0.0), dev("b", false, 0.180, 0.0)],
            AlignPolicy::Auto,
            true,
        ));
        assert_eq!(p.mode, Mode::Takeover);
        // video lag = lead + b_min = 37 ms < 45 ms -> no lip-sync advice
        assert!(p.advice.is_empty(), "{:?}", p.advice);
    }

    #[test]
    fn windows_without_takeover_reports_late_mirror() {
        let p = plan(&input(
            vec![dev("spk", true, 0.028, 0.0), dev("bt", false, 0.200, 0.0)],
            AlignPolicy::Auto,
            false,
        ));
        assert_eq!(p.mode, Mode::Reference);
        assert!(p
            .advice
            .iter()
            .any(|a| matches!(a, Advice::MirrorLate { id, by } if id.0 == "bt" && *by > 0.2)));
        assert!(p
            .advice
            .iter()
            .any(|a| matches!(a, Advice::LipSync { suggest, .. } if suggest.0 == "bt")));
    }

    #[test]
    fn unchecked_default_is_muted_by_takeover() {
        let mut d = dev("spk", true, 0.028, 0.0);
        d.checked = false;
        let p = plan(&input(
            vec![d, dev("a", false, 0.18, 0.0), dev("b", false, 0.19, 0.0)],
            AlignPolicy::Auto,
            true,
        ));
        assert_eq!(p.mode, Mode::Takeover);
        assert_eq!(p.targets.len(), 2);
    }

    #[test]
    fn windows_silent_sink_default_drives_all_real_devices() {
        // Windows: the user made an unused HDMI endpoint the default and unchecked it.
        let mut sink = dev("hdmi-unused", true, 0.010, 0.0);
        sink.checked = false;
        let p = plan(&input(
            vec![
                sink,
                dev("a", false, 0.18, 0.0),
                dev("spk", false, 0.03, 0.0),
            ],
            AlignPolicy::Auto,
            false,
        ));
        assert_eq!(p.mode, Mode::Takeover);
        assert_eq!(p.targets.len(), 2);
        assert!(p
            .advice
            .iter()
            .any(|a| matches!(a, Advice::CannotMuteDefault { .. })));
    }

    #[test]
    fn targets_are_capped_at_max_lambda() {
        // AirPlay at 6.99 s plus a +50 ms offset on the speaker exceeds the 7 s cap.
        let mut spk = dev("spk", true, 0.028, 0.050);
        spk.checked = true;
        let p = plan(&input(
            vec![spk, dev("airplay", false, 6.95, 0.0)],
            AlignPolicy::Auto,
            true,
        ));
        assert_eq!(p.mode, Mode::Takeover);
        assert!(p.targets.iter().all(|t| t.lambda <= 7.0 + 1e-9));
        assert!(p
            .advice
            .iter()
            .any(|a| matches!(a, Advice::MirrorLate { .. })));
    }

    #[test]
    fn prefer_reference_keeps_system_path_and_reports_late_mirror() {
        // Takeover would be chosen by Auto here; PreferReference never mutes the system path.
        let p = plan(&input(
            vec![dev("spk", true, 0.028, 0.0), dev("bt", false, 0.200, 0.0)],
            AlignPolicy::PreferReference,
            true,
        ));
        assert_eq!(p.mode, Mode::Reference);
        assert_eq!(p.targets.len(), 1);
        assert_eq!(p.targets[0].id.0, "bt");
        let lam_min = 0.011 + 0.200 + 0.026;
        assert!((p.targets[0].lambda - lam_min).abs() < 1e-9);
        assert!((p.targets[0].late_by - (lam_min - 0.028)).abs() < 1e-9);
        assert!(p
            .advice
            .iter()
            .any(|a| matches!(a, Advice::MirrorLate { id, .. } if id.0 == "bt")));
        assert!(p
            .advice
            .iter()
            .any(|a| matches!(a, Advice::LipSync { suggest, .. } if suggest.0 == "bt")));
    }

    #[test]
    fn prefer_takeover_takes_over_even_when_reference_is_feasible() {
        let devices = vec![dev("bt", true, 0.200, 0.0), dev("spk", false, 0.028, 0.0)];
        let p = plan(&input(devices.clone(), AlignPolicy::PreferTakeover, true));
        assert_eq!(p.mode, Mode::Takeover);
        assert_eq!(p.targets.len(), 2);
        let common = 0.011 + 0.200 + 0.026;
        assert!(p.targets.iter().all(|t| (t.lambda - common).abs() < 1e-9));
        // Slowest device is the default: no lip-sync suggestion.
        assert!(p.advice.is_empty(), "{:?}", p.advice);

        // Without the ability to mute the capture it falls back to reference mode.
        let p = plan(&input(devices, AlignPolicy::PreferTakeover, false));
        assert_eq!(p.mode, Mode::Reference);
        assert_eq!(p.targets.len(), 1);
        assert!((p.targets[0].lambda - 0.200).abs() < 1e-9);
    }

    #[test]
    fn empty_device_list_plans_nothing() {
        for policy in [
            AlignPolicy::Auto,
            AlignPolicy::PreferReference,
            AlignPolicy::PreferTakeover,
        ] {
            for can_mute in [true, false] {
                let p = plan(&input(Vec::new(), policy, can_mute));
                assert_eq!(p.mode, Mode::Reference);
                assert!(p.targets.is_empty());
                assert!(p.advice.is_empty());
            }
        }
    }

    #[test]
    fn only_default_device() {
        // The system plays it natively: nothing for Wye to render.
        let p = plan(&input(
            vec![dev("spk", true, 0.028, 0.0)],
            AlignPolicy::Auto,
            true,
        ));
        assert_eq!(p.mode, Mode::Reference);
        assert!(p.targets.is_empty());
        assert!(p.advice.is_empty(), "{:?}", p.advice);

        // Forced takeover renders the default device itself at its own minimum λ.
        let p = plan(&input(
            vec![dev("spk", true, 0.028, 0.0)],
            AlignPolicy::PreferTakeover,
            true,
        ));
        assert_eq!(p.mode, Mode::Takeover);
        assert_eq!(p.targets.len(), 1);
        assert!((p.targets[0].lambda - (0.011 + 0.028 + 0.026)).abs() < 1e-9);
        assert!(p.advice.is_empty(), "{:?}", p.advice);

        // Unchecked and the only device: nothing to play, nothing to advise but the mute limit.
        let mut d = dev("spk", true, 0.028, 0.0);
        d.checked = false;
        let p = plan(&input(vec![d.clone()], AlignPolicy::Auto, true));
        assert!(p.targets.is_empty());
        assert!(p.advice.is_empty(), "{:?}", p.advice);
        let p = plan(&input(vec![d], AlignPolicy::Auto, false));
        assert!(p.targets.is_empty());
        assert!(matches!(&p.advice[..], [Advice::CannotMuteDefault { id }] if id.0 == "spk"));
    }

    #[test]
    fn offsets_shift_targets() {
        let p = plan(&input(
            vec![dev("bt", true, 0.200, 0.0), dev("spk", false, 0.028, 0.010)],
            AlignPolicy::Auto,
            true,
        ));
        assert!((p.targets[0].lambda - 0.210).abs() < 1e-9);
    }
}
