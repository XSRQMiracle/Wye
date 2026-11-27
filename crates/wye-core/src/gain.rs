//! Per-device volume: slider position → linear gain, with a one-pole smoother to avoid zipper
//! noise.

/// Slider position (0..=1) to linear gain: cubic taper (≈ −60 dB at 0.1, −18 dB at 0.5).
pub fn slider_to_gain(pos: f32) -> f32 {
    let p = pos.clamp(0.0, 1.0);
    p * p * p
}

/// Smooths gain changes over ~20 ms. Real-time safe.
pub struct GainSmoother {
    current: f32,
    coeff: f32,
}

impl GainSmoother {
    pub fn new(rate: f64, initial: f32) -> Self {
        let tau = 0.020;
        GainSmoother {
            current: initial,
            coeff: (-1.0 / (tau * rate)).exp() as f32,
        }
    }
    /// Apply to interleaved stereo in place, ramping towards `target`.
    pub fn apply(&mut self, target: f32, stereo: &mut [f32]) {
        for fr in stereo.chunks_exact_mut(2) {
            self.current = target + (self.current - target) * self.coeff;
            fr[0] *= self.current;
            fr[1] *= self.current;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn taper() {
        assert_eq!(slider_to_gain(1.0), 1.0);
        assert_eq!(slider_to_gain(0.0), 0.0);
        assert!((20.0 * slider_to_gain(0.5).log10() + 18.06).abs() < 0.1);
    }

    /// Feed `frames` frames of constant 1.0 through the smoother and return the last gain.
    fn ramp(s: &mut GainSmoother, target: f32, frames: usize) -> f32 {
        let mut buf = vec![1.0f32; frames * 2];
        s.apply(target, &mut buf);
        assert_eq!(buf[buf.len() - 2], buf[buf.len() - 1]);
        buf[buf.len() - 1]
    }

    #[test]
    fn smoothing_settles_within_one_percent() {
        for rate in [44_100.0, 48_000.0, 96_000.0] {
            for (from, to) in [(0.0f32, 1.0f32), (1.0, 0.0), (1.0, 0.125)] {
                let mut s = GainSmoother::new(rate, from);
                // Five time constants (100 ms) bring the residual to e^-5 < 1 %.
                let g = ramp(&mut s, to, (0.100 * rate) as usize);
                assert!(
                    (g - to).abs() < 0.01 * (to - from).abs(),
                    "rate {rate}: {from} -> {to} reached {g}"
                );
                // The change is a ramp, not a jump: the first sample moves only slightly.
                let mut s = GainSmoother::new(rate, from);
                let g1 = ramp(&mut s, to, 1);
                assert!((g1 - from).abs() < 0.01 * (to - from).abs());
            }
        }
    }

    #[test]
    fn smoothing_time_constant_is_20ms() {
        // After 20 ms the step has advanced by 1 − 1/e of its size (one time constant).
        let rate = 48_000.0;
        let mut s = GainSmoother::new(rate, 0.0);
        let g = ramp(&mut s, 1.0, (0.020 * rate) as usize);
        let expect = 1.0 - (-1.0f32).exp();
        assert!((g - expect).abs() < 0.01, "{g} vs {expect}");
    }
}
