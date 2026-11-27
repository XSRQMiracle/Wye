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
}
