//! Acoustic latency calibration. Runs on the control thread (not real-time).
//!
//! The engine plays an exponential sweep on a device, records a microphone, and locates the
//! sweep in the recording by cross-correlation. Only relative values are used: a reference
//! device and the target device are measured in the same session, so the microphone latency
//! and its reporting error cancel in the difference.

/// Exponential sine sweep with 10 ms raised fades. Same generator for playback and reference.
pub fn exp_sweep(rate: f64, f1: f64, f2: f64, secs: f64, amp: f32) -> Vec<f32> {
    let n = (rate * secs) as usize;
    let k = (f2 / f1).ln();
    let fade = ((rate * 0.01) as usize).max(1);
    (0..n)
        .map(|i| {
            let t = i as f64 / rate;
            let ph = 2.0 * std::f64::consts::PI * f1 * secs / k * ((t * k / secs).exp() - 1.0);
            let g = if i < fade {
                i as f64 / fade as f64
            } else if i + fade > n {
                (n - i) as f64 / fade as f64
            } else {
                1.0
            };
            (ph.sin() * g) as f32 * amp
        })
        .collect()
}

/// Sweep start frequency (Hz).
pub const SWEEP_F1: f64 = 200.0;
/// Sweep end frequency (Hz).
pub const SWEEP_F2: f64 = 8000.0;
/// Sweep duration (s).
pub const SWEEP_SECS: f64 = 0.6;
/// Playback amplitude of the sweep (the reference for `locate` may use any amplitude).
pub const SWEEP_AMP: f32 = 0.25;
/// Repetitions played per device.
pub const REPETITIONS: usize = 5;
/// Minimum number of repetitions with a confident detection.
pub const MIN_VALID: usize = 3;
/// Maximum allowed max − min of the accepted measurements of one device (s).
pub const MAX_SPREAD: f64 = 0.003;

#[derive(Clone, Debug, PartialEq)]
pub struct Detection {
    /// Index into the recording where the reference starts (integer correlation peak).
    pub lag: usize,
    /// Sub-sample position of the reference start in the recording (samples), refined by
    /// parabolic interpolation around `lag`; always within `lag ± 0.5`.
    pub lag_frac: f64,
    /// Peak / median of |cross-correlation|. Accept only ≥ `MIN_CONFIDENCE`.
    pub confidence: f32,
}

pub const MIN_CONFIDENCE: f32 = 20.0;

/// FFT cross-correlation (realfft) with parabolic peak interpolation.
///
/// Only lags where the reference lies fully inside the recording are considered. The peak of
/// |r| is accepted when `peak / median(|r|) ≥ MIN_CONFIDENCE`; its position is then refined to
/// sub-sample precision with a parabola through the peak and its two neighbours. Returns
/// `None` for an empty reference, a recording shorter than the reference, or no confident
/// peak (silence, noise only, a signal that is too quiet). Allocates: control thread only.
pub fn locate(recording: &[f32], reference: &[f32]) -> Option<Detection> {
    let r = cross_correlate(recording, reference)?;
    let (lag, peak) = r
        .iter()
        .map(|v| v.abs())
        .enumerate()
        .fold(
            (0usize, 0.0f32),
            |best, cur| if cur.1 > best.1 { cur } else { best },
        );
    if !(peak.is_finite() && peak > 0.0) {
        return None;
    }
    let mut mags: Vec<f32> = r.iter().map(|v| v.abs()).collect();
    let mid = mags.len() / 2;
    let median = *mags.select_nth_unstable_by(mid, f32::total_cmp).1;
    let confidence = if median > 0.0 {
        peak / median
    } else {
        f32::INFINITY
    };
    if confidence < MIN_CONFIDENCE {
        return None;
    }
    let lag_frac = lag as f64 + parabolic_offset(&r, lag);
    Some(Detection {
        lag,
        lag_frac,
        confidence,
    })
}

/// `r[k] = Σ_i recording[k + i] · reference[i]` for `k in 0..=recording.len() − reference.len()`.
fn cross_correlate(recording: &[f32], reference: &[f32]) -> Option<Vec<f32>> {
    if reference.is_empty() || recording.len() < reference.len() {
        return None;
    }
    // Zero padding to at least the sum of the lengths makes the circular correlation equal to
    // the linear one over every valid lag.
    let n = (recording.len() + reference.len()).next_power_of_two();
    let mut planner = realfft::RealFftPlanner::<f32>::new();
    let fwd = planner.plan_fft_forward(n);
    let inv = planner.plan_fft_inverse(n);

    let mut x = fwd.make_input_vec();
    x[..recording.len()].copy_from_slice(recording);
    let mut h = fwd.make_input_vec();
    h[..reference.len()].copy_from_slice(reference);
    let mut xs = fwd.make_output_vec();
    let mut hs = fwd.make_output_vec();
    fwd.process(&mut x, &mut xs).ok()?;
    fwd.process(&mut h, &mut hs).ok()?;
    for (a, b) in xs.iter_mut().zip(hs.iter()) {
        *a *= b.conj();
    }
    // The inverse real FFT requires purely real DC and Nyquist bins; rounding can leave tiny
    // imaginary parts there, which realfft would reject.
    xs[0].im = 0.0;
    if let Some(last) = xs.last_mut() {
        last.im = 0.0;
    }
    let mut out = inv.make_output_vec();
    inv.process(&mut xs, &mut out).ok()?;
    out.truncate(recording.len() - reference.len() + 1);
    let scale = 1.0 / n as f32;
    out.iter_mut().for_each(|v| *v *= scale);
    Some(out)
}

/// Vertex offset (−0.5..=0.5 samples) of the parabola through `r[k−1], r[k], r[k+1]`,
/// evaluated with the peak's polarity so an inverted signal is refined the same way.
fn parabolic_offset(r: &[f32], k: usize) -> f64 {
    if k == 0 || k + 1 >= r.len() {
        return 0.0;
    }
    let s = if r[k] < 0.0 { -1.0 } else { 1.0 };
    let (a, b, c) = (s * r[k - 1] as f64, s * r[k] as f64, s * r[k + 1] as f64);
    let den = a - 2.0 * b + c;
    // Not a maximum (or NaN): keep the integer peak.
    if den.is_nan() || den >= 0.0 {
        return 0.0;
    }
    (0.5 * (a - c) / den).clamp(-0.5, 0.5)
}

/// Time stamp of one recorded input block: the recording index of its first sample and the
/// host time (ns) at which that sample was captured.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BlockStamp {
    pub first_index: u64,
    pub host_ns: u64,
}

/// Host time (ns, fractional) of recording sample `index`: the stamp of the block containing
/// it (the last stamp with `first_index ≤ index`) plus the offset at `rate` Hz. `stamps` must
/// be sorted. `None` when no stamp precedes `index`.
pub fn host_of(stamps: &[BlockStamp], rate: f64, index: f64) -> Option<f64> {
    let k = stamps.partition_point(|b| (b.first_index as f64) <= index);
    let b = stamps.get(k.checked_sub(1)?)?;
    Some(b.host_ns as f64 + (index - b.first_index as f64) * 1e9 / rate)
}

/// Recording index (fractional) of the sample captured at host time `host_ns`, from the last
/// stamp at or before that time. `stamps` must be sorted. `None` when `host_ns` precedes every
/// stamp.
pub fn index_of(stamps: &[BlockStamp], rate: f64, host_ns: f64) -> Option<f64> {
    let k = stamps.partition_point(|b| (b.host_ns as f64) <= host_ns);
    let b = stamps.get(k.checked_sub(1)?)?;
    Some(b.first_index as f64 + (host_ns - b.host_ns as f64) * rate / 1e9)
}

/// Interval between the starts of two repetitions (s), so the previous sweep and its acoustic
/// tail have arrived before the next one starts. `reported_total` is the device's reported
/// output latency (s).
pub fn repetition_interval(reported_total: f64) -> f64 {
    (reported_total + 0.9).max(1.2)
}

/// Window of the recording to search, in seconds relative to the sweep start host time `t_s`:
/// from 50 ms before the start to the end of the sweep after the expected output and
/// microphone latency, plus 0.3 s of slack.
pub fn search_window(reported_total: f64, mic_latency: f64) -> (f64, f64) {
    (-0.05, reported_total + mic_latency + SWEEP_SECS + 0.3)
}

/// Why a device's calibration failed after all repetitions were measured.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalibrationFailure {
    /// Fewer than `MIN_VALID` repetitions produced a confident detection.
    TooQuiet,
    /// Enough detections, but their max − min exceeds `MAX_SPREAD`.
    Inconsistent,
}

impl CalibrationFailure {
    /// Stable snake_case reason used in notices and translation keys.
    pub fn reason(self) -> &'static str {
        match self {
            CalibrationFailure::TooQuiet => "too_quiet",
            CalibrationFailure::Inconsistent => "inconsistent",
        }
    }
}

/// Aggregate one device's per-repetition measurements (s; `None` = no confident detection).
/// Succeeds with ≥ `MIN_VALID` valid repetitions whose max − min is ≤ `MAX_SPREAD`; the
/// result holds their median (mean of the two middle values for an even count).
pub fn aggregate(measurements: &[Option<f64>]) -> Result<CalibrationResult, CalibrationFailure> {
    let samples: Vec<f64> = measurements
        .iter()
        .flatten()
        .copied()
        .filter(|v| v.is_finite())
        .collect();
    if samples.len() < MIN_VALID {
        return Err(CalibrationFailure::TooQuiet);
    }
    let mut sorted = samples.clone();
    sorted.sort_by(f64::total_cmp);
    let spread = sorted[sorted.len() - 1] - sorted[0];
    if spread > MAX_SPREAD {
        return Err(CalibrationFailure::Inconsistent);
    }
    let m = sorted.len() / 2;
    let median = if sorted.len() % 2 == 1 {
        sorted[m]
    } else {
        0.5 * (sorted[m - 1] + sorted[m])
    };
    Ok(CalibrationResult {
        samples,
        median,
        spread,
    })
}

/// Calibrated output-timestamp → acoustic latency of a device (s): the latency currently used
/// for the reference device plus the difference of the two measured apparent latencies.
pub fn calibrated_after(after_ref_used: f64, app_dev: f64, app_ref: f64) -> f64 {
    after_ref_used + (app_dev - app_ref)
}

#[derive(Clone, Debug, PartialEq)]
pub struct CalibrationResult {
    /// Measured output-timestamp → acoustic latency per repetition (s).
    pub samples: Vec<f64>,
    pub median: f64,
    /// max − min of accepted samples (s).
    pub spread: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 48_000.0;

    /// Deterministic xorshift64* generator with Box–Muller Gaussian output.
    struct Rng(u64);

    impl Rng {
        fn uniform(&mut self) -> f64 {
            self.0 ^= self.0 >> 12;
            self.0 ^= self.0 << 25;
            self.0 ^= self.0 >> 27;
            let v = self.0.wrapping_mul(0x2545_F491_4F6C_DD1D);
            ((v >> 11) as f64 + 0.5) / (1u64 << 53) as f64
        }

        fn gauss(&mut self) -> f64 {
            let (u1, u2) = (self.uniform(), self.uniform());
            (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
        }
    }

    /// The same sweep as `exp_sweep`, evaluated in continuous time and delayed by `delay`
    /// samples (fractional delays allowed), placed in a buffer of `len` samples.
    fn delayed_sweep(len: usize, delay: f64, amp: f32) -> Vec<f32> {
        let n = RATE * SWEEP_SECS;
        let k = (SWEEP_F2 / SWEEP_F1).ln();
        let fade = RATE * 0.01;
        (0..len)
            .map(|i| {
                let x = i as f64 - delay;
                if x < 0.0 || x >= n {
                    return 0.0;
                }
                let t = x / RATE;
                let ph = 2.0 * std::f64::consts::PI * SWEEP_F1 * SWEEP_SECS / k
                    * ((t * k / SWEEP_SECS).exp() - 1.0);
                let g = (x / fade).min((n - x) / fade).min(1.0);
                (ph.sin() * g) as f32 * amp
            })
            .collect()
    }

    fn reference() -> Vec<f32> {
        exp_sweep(RATE, SWEEP_F1, SWEEP_F2, SWEEP_SECS, 1.0)
    }

    fn mean_square(x: &[f32]) -> f64 {
        x.iter().map(|&v| (v as f64).powi(2)).sum::<f64>() / x.len() as f64
    }

    /// Recording of 1.5 s: sweep at `delay` plus white noise with the sweep's power (SNR 0 dB
    /// measured over the sweep duration).
    fn noisy_recording(delay: f64, seed: u64) -> Vec<f32> {
        let amp = SWEEP_AMP;
        let mut rec = delayed_sweep((RATE * 1.5) as usize, delay, amp);
        let sigma = mean_square(&exp_sweep(RATE, SWEEP_F1, SWEEP_F2, SWEEP_SECS, amp)).sqrt();
        let mut rng = Rng(seed);
        for v in rec.iter_mut() {
            *v += (rng.gauss() * sigma) as f32;
        }
        rec
    }

    #[test]
    fn locate_integer_and_fractional_delay_at_0db_snr() {
        let r = reference();
        for (i, &delay) in [2400.0, 12_345.0, 7000.37, 20_000.5, 31_111.83, 0.0]
            .iter()
            .enumerate()
        {
            let rec = noisy_recording(delay, 0x9E37_79B9_7F4A_7C15 ^ i as u64);
            let d = locate(&rec, &r).expect("sweep must be found at 0 dB SNR");
            let err = (d.lag_frac - delay).abs();
            assert!(
                err <= 1.0,
                "delay {delay}: located {} (err {err})",
                d.lag_frac
            );
            assert!((d.lag_frac - d.lag as f64).abs() <= 0.5);
            assert!(d.confidence >= MIN_CONFIDENCE);
        }
    }

    #[test]
    fn locate_noise_free_fractional_delay_is_sub_sample() {
        let r = reference();
        for &delay in &[1000.25, 5000.5, 9000.75, 3333.1] {
            let rec = delayed_sweep((RATE * 1.2) as usize, delay, SWEEP_AMP);
            let d = locate(&rec, &r).unwrap();
            let err = (d.lag_frac - delay).abs();
            assert!(
                err <= 0.25,
                "delay {delay}: located {} (err {err})",
                d.lag_frac
            );
        }
    }

    #[test]
    fn locate_inverted_polarity() {
        let r = reference();
        let rec: Vec<f32> = noisy_recording(8888.4, 7).iter().map(|v| -v).collect();
        let d = locate(&rec, &r).unwrap();
        assert!((d.lag_frac - 8888.4).abs() <= 1.0, "{}", d.lag_frac);
    }

    #[test]
    fn locate_rejects_noise_only() {
        let r = reference();
        for seed in 1..=5u64 {
            let mut rng = Rng(seed.wrapping_mul(0xD1B5_4A32_D192_ED03));
            let rec: Vec<f32> = (0..(RATE * 1.5) as usize)
                .map(|_| (rng.gauss() * 0.2) as f32)
                .collect();
            assert_eq!(locate(&rec, &r), None, "seed {seed}");
        }
    }

    #[test]
    fn locate_rejects_silence_and_bad_input() {
        let r = reference();
        assert_eq!(locate(&vec![0.0; (RATE * 1.5) as usize], &r), None);
        assert_eq!(locate(&r[..r.len() - 1], &r), None);
        assert_eq!(locate(&[], &r), None);
        assert_eq!(locate(&r, &[]), None);
    }

    #[test]
    fn locate_near_window_edges() {
        let r = reference();
        // A recording exactly as long as the reference has a single lag, so there is no
        // background to judge confidence against.
        assert_eq!(locate(&r, &r), None);
        // Sweep at the very first and very last valid lag.
        let len = r.len() + 4000;
        for &delay in &[0.0, 4000.0] {
            let d = locate(&delayed_sweep(len, delay, SWEEP_AMP), &r).unwrap();
            assert!((d.lag_frac - delay).abs() <= 1.0, "{delay}: {}", d.lag_frac);
        }
    }

    #[test]
    fn host_and_index_mapping() {
        let stamps = [
            BlockStamp {
                first_index: 0,
                host_ns: 1_000_000_000,
            },
            BlockStamp {
                first_index: 512,
                host_ns: 1_010_700_000,
            },
        ];
        assert_eq!(host_of(&stamps, RATE, 0.0), Some(1e9));
        let h = host_of(&stamps, RATE, 48.0).unwrap();
        assert!((h - (1e9 + 1e6)).abs() < 1e-3);
        // Second block uses its own stamp.
        let h = host_of(&stamps, RATE, 560.0).unwrap();
        assert!((h - (1_010_700_000.0 + 1e6)).abs() < 1e-3);
        let i = index_of(&stamps, RATE, 1_010_700_000.0 + 1e6).unwrap();
        assert!((i - 560.0).abs() < 1e-9);
        assert_eq!(index_of(&stamps, RATE, 0.0), None);
        assert_eq!(host_of(&[], RATE, 0.0), None);
    }

    #[test]
    fn timing_helpers() {
        assert_eq!(repetition_interval(0.01), 1.2);
        assert!((repetition_interval(0.5) - 1.4).abs() < 1e-12);
        let (a, b) = search_window(0.2, 0.05);
        assert_eq!(a, -0.05);
        assert!((b - 1.15).abs() < 1e-12);
    }

    #[test]
    fn aggregate_median_and_spread() {
        let r = aggregate(&[Some(0.101), None, Some(0.100), Some(0.102), Some(0.1015)]).unwrap();
        assert_eq!(r.samples, vec![0.101, 0.100, 0.102, 0.1015]);
        assert!((r.median - 0.10125).abs() < 1e-12);
        assert!((r.spread - 0.002).abs() < 1e-12);
        let r = aggregate(&[Some(0.2), Some(0.201), Some(0.199)]).unwrap();
        assert_eq!(r.median, 0.2);
    }

    #[test]
    fn aggregate_failures() {
        let e = aggregate(&[Some(0.1), None, Some(0.1), None, None]).unwrap_err();
        assert_eq!(e, CalibrationFailure::TooQuiet);
        assert_eq!(e.reason(), "too_quiet");
        assert_eq!(aggregate(&[]), Err(CalibrationFailure::TooQuiet));
        assert_eq!(
            aggregate(&[Some(f64::NAN), Some(0.1), Some(0.1)]),
            Err(CalibrationFailure::TooQuiet)
        );
        let e = aggregate(&[Some(0.100), Some(0.1031), Some(0.101)]).unwrap_err();
        assert_eq!(e, CalibrationFailure::Inconsistent);
        assert_eq!(e.reason(), "inconsistent");
        // Exactly at the limit is accepted.
        assert!(aggregate(&[Some(0.0), Some(MAX_SPREAD), Some(0.001)]).is_ok());
    }

    #[test]
    fn calibrated_after_uses_difference() {
        let after = calibrated_after(0.028, 0.250, 0.060);
        assert!((after - 0.218).abs() < 1e-12);
    }
}
