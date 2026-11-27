//! Acoustic latency calibration. Runs on the control thread (not real-time).

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

#[derive(Clone, Debug, PartialEq)]
pub struct Detection {
    /// Index into the recording where the reference starts.
    pub lag: usize,
    /// Peak / median of |cross-correlation|. Accept only ≥ `MIN_CONFIDENCE`.
    pub confidence: f32,
}

pub const MIN_CONFIDENCE: f32 = 20.0;

/// FFT cross-correlation (realfft) with parabolic peak interpolation.
pub fn locate(_recording: &[f32], _reference: &[f32]) -> Option<Detection> {
    todo!("cross-correlation locate")
}

#[derive(Clone, Debug, PartialEq)]
pub struct CalibrationResult {
    /// Measured output-timestamp → acoustic latency per repetition (s).
    pub samples: Vec<f64>,
    pub median: f64,
    /// max − min of accepted samples (s).
    pub spread: f64,
}
