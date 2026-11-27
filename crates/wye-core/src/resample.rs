//! Variable-ratio resampler used inside each render callback.
//! Implementation: rubato 5.1.0 `Async` sinc128 (cubic, BlackmanHarris2), `FixedAsync::Output`.

/// Real-time safe resampler: all buffers are allocated in `new`; no method allocates.
pub trait VarResampler: Send {
    /// Set the ratio correction δ (output/input = f_out/f_in × (1 + δ)). |δ| ≤ 1e-3.
    /// Applied with ramping (rubato `set_resample_ratio_relative(1+δ, true)`).
    fn set_correction(&mut self, delta: f64);
    /// Exact number of input frames the next `process` call producing `out_frames` frames will
    /// consume (rubato `set_chunk_size(out_frames)` + `input_frames_next()`).
    /// `out_frames` ≤ `max_out_frames()`; callers split larger requests.
    fn input_needed(&mut self, out_frames: usize) -> usize;
    /// Produce exactly `out.len() / 2` interleaved stereo frames. The input is the interleaved
    /// stereo data `a` followed by `b` (the two slices of an rtrb read chunk; zero-copy).
    /// `valid_frames` = frames actually present in a+b; if it is less than
    /// `input_needed(out_frames)` the missing tail is treated as zeros (rubato `partial_len`).
    /// Returns the number of input frames consumed from a+b (= min(valid_frames, needed)).
    fn process(&mut self, a: &[f32], b: &[f32], valid_frames: usize, out: &mut [f32]) -> usize;
    /// Group delay in input frames (constant, = sinc_len / 2 = 64 for sinc128).
    fn delay_input_frames(&self) -> f64;
    /// Largest output chunk per `process` call (constructor `max_chunk`, 4096).
    fn max_out_frames(&self) -> usize;
    /// Drop internal history (after a resync). Real-time safe.
    fn reset(&mut self);
}
