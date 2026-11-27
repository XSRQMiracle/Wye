//! Variable-ratio resampler used inside each render callback.
//! Implementation: rubato 5.1.0 `Async` sinc128 (cubic, BlackmanHarris2), `FixedAsync::Output`.

use audioadapter::Adapter;
use rubato::audioadapter_buffers::direct::InterleavedSlice;
use rubato::{
    Adjustable, Async, FixedAsync, Indexing, Resampler, Resizable, SincInterpolationParameters,
    SincInterpolationType, WindowFunction,
};

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

/// Largest render callback (frames) a single `process` call handles.
pub const MAX_CHUNK_FRAMES: usize = 4096;
const CHANNELS: usize = 2;
/// Headroom of the ratio range over the ±1000 ppm controller limit. It only sizes rubato's
/// internal buffers.
const MAX_RELATIVE_RATIO: f64 = 1.01;
/// Defensive clamp for `set_correction`; keeps the ratio well inside `MAX_RELATIVE_RATIO`.
const DELTA_LIMIT: f64 = 5.0e-3;

#[derive(Debug, thiserror::Error)]
pub enum ResamplerError {
    #[error("invalid sample rates: {in_rate} Hz -> {out_rate} Hz")]
    InvalidRate { in_rate: f64, out_rate: f64 },
    #[error(transparent)]
    Construction(#[from] rubato::ResamplerConstructionError),
}

/// rubato `Async` windowed-sinc resampler for interleaved stereo f32.
///
/// Every render callback sets the chunk size to the frames actually requested (O(1)), asks
/// for the exact input frame count and reads straight out of the ring buffer through a
/// zero-copy two-slice adapter, writing directly into the device buffer. There is no extra
/// FIFO, so the only latency is the filter's own half length.
pub struct RubatoResampler {
    rs: Async<f32>,
    /// Chunk size currently configured in `rs` (avoids redundant recomputation).
    chunk: usize,
    /// Last correction δ, re-applied after `reset` (which restores the nominal ratio).
    delta: f64,
    delay: f64,
}

impl RubatoResampler {
    /// Not real-time safe (allocates). `high_quality` selects sinc256 instead of sinc128.
    pub fn new(in_rate: f64, out_rate: f64, high_quality: bool) -> Result<Self, ResamplerError> {
        if !(in_rate.is_finite() && out_rate.is_finite() && in_rate > 0.0 && out_rate > 0.0) {
            return Err(ResamplerError::InvalidRate { in_rate, out_rate });
        }
        let sinc_len = if high_quality { 256 } else { 128 };
        let params = SincInterpolationParameters::new(sinc_len, WindowFunction::BlackmanHarris2)
            .oversampling_factor(128)
            .interpolation(SincInterpolationType::Cubic);
        let rs = Async::<f32>::new_sinc(
            out_rate / in_rate,
            MAX_RELATIVE_RATIO,
            &params,
            MAX_CHUNK_FRAMES,
            CHANNELS,
            FixedAsync::Output,
        )?;
        Ok(RubatoResampler {
            rs,
            chunk: MAX_CHUNK_FRAMES,
            delta: 0.0,
            delay: (sinc_len / 2) as f64,
        })
    }

    #[inline]
    fn set_chunk(&mut self, frames: usize) {
        if frames != self.chunk {
            // Cannot fail: callers guarantee 1 ..= MAX_CHUNK_FRAMES.
            if self.rs.set_chunk_size(frames).is_ok() {
                self.chunk = frames;
            }
        }
    }
}

impl VarResampler for RubatoResampler {
    fn set_correction(&mut self, delta: f64) {
        let d = if delta.is_finite() {
            delta.clamp(-DELTA_LIMIT, DELTA_LIMIT)
        } else {
            0.0
        };
        self.delta = d;
        // Ramped linearly across the next chunk; cannot fail inside MAX_RELATIVE_RATIO.
        let _ = self.rs.set_resample_ratio_relative(1.0 + d, true);
    }

    fn input_needed(&mut self, out_frames: usize) -> usize {
        debug_assert!(out_frames <= MAX_CHUNK_FRAMES);
        if out_frames == 0 || out_frames > MAX_CHUNK_FRAMES {
            return 0;
        }
        self.set_chunk(out_frames);
        self.rs.input_frames_next()
    }

    fn process(&mut self, a: &[f32], b: &[f32], valid_frames: usize, out: &mut [f32]) -> usize {
        let frames = out.len() / CHANNELS;
        debug_assert!(frames <= MAX_CHUNK_FRAMES);
        debug_assert!(a.len().is_multiple_of(CHANNELS) && b.len().is_multiple_of(CHANNELS));
        if frames == 0 {
            return 0;
        }
        if frames > MAX_CHUNK_FRAMES {
            out.fill(0.0);
            return 0;
        }
        self.set_chunk(frames);
        let need = self.rs.input_frames_next();
        let take = valid_frames.min(need).min((a.len() + b.len()) / CHANNELS);
        let input = TwoSlices::new(a, b);
        let out = &mut out[..frames * CHANNELS];
        let Ok(mut output) = InterleavedSlice::new_mut(&mut *out, CHANNELS, frames) else {
            return 0; // unreachable: the slice holds exactly `frames` frames
        };
        // Underrun: rubato zero-pads the missing tail; the stream still advances by `need`.
        // An `Indexing` without a channel mask does not allocate.
        let partial;
        let indexing = if take < need {
            partial = Indexing::new().partial_len(take);
            Some(&partial)
        } else {
            None
        };
        if self
            .rs
            .process_into_buffer(&input, &mut output, indexing)
            .is_err()
        {
            // Only possible on a programming error (buffer sizes); output silence.
            out.fill(0.0);
        }
        take
    }

    fn delay_input_frames(&self) -> f64 {
        self.delay
    }

    fn max_out_frames(&self) -> usize {
        MAX_CHUNK_FRAMES
    }

    fn reset(&mut self) {
        // Clears the history in place (no allocation); also restores the nominal ratio and
        // the maximum chunk size, so both are re-applied.
        self.rs.reset();
        self.chunk = MAX_CHUNK_FRAMES;
        let _ = self.rs.set_resample_ratio_relative(1.0 + self.delta, false);
    }
}

/// Read-only interleaved stereo view of a (possibly wrapped) ring-buffer region, as returned
/// by `rtrb::chunks::ReadChunk::as_slices()`. Both slices must hold whole frames.
struct TwoSlices<'a> {
    a: &'a [f32],
    b: &'a [f32],
    frames: usize,
    frames_a: usize,
}

impl<'a> TwoSlices<'a> {
    fn new(a: &'a [f32], b: &'a [f32]) -> Self {
        TwoSlices {
            a,
            b,
            frames: (a.len() + b.len()) / CHANNELS,
            frames_a: a.len() / CHANNELS,
        }
    }
}

// SAFETY: `read_sample_unchecked` is only called by rubato with `channel < channels()` and
// `frame < frames()`, so the computed index is inside `a` or inside `b`.
unsafe impl Adapter<f32> for TwoSlices<'_> {
    unsafe fn read_sample_unchecked(&self, channel: usize, frame: usize) -> f32 {
        let i = frame * CHANNELS + channel;
        // SAFETY: see the impl comment; `i < a.len() + b.len()`.
        unsafe {
            if i < self.a.len() {
                *self.a.get_unchecked(i)
            } else {
                *self.b.get_unchecked(i - self.a.len())
            }
        }
    }
    fn channels(&self) -> usize {
        CHANNELS
    }
    fn frames(&self) -> usize {
        self.frames
    }
    // rubato pulls input one channel at a time through this method; a strided loop per half
    // is much cheaper than the default per-sample path.
    fn copy_from_channel_to_slice(&self, channel: usize, skip: usize, slice: &mut [f32]) -> usize {
        if channel >= CHANNELS || skip >= self.frames {
            return 0;
        }
        let n = slice.len().min(self.frames - skip);
        let mut k = 0;
        let mut f = skip;
        while k < n && f < self.frames_a {
            slice[k] = self.a[f * CHANNELS + channel];
            k += 1;
            f += 1;
        }
        while k < n {
            slice[k] = self.b[(f - self.frames_a) * CHANNELS + channel];
            k += 1;
            f += 1;
        }
        n
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_alloc::count_allocs;
    use std::f64::consts::PI;

    const AMP: f64 = 0.5; // −6 dBFS
    const TONE_HZ: f64 = 1000.0;

    /// Deterministic xorshift generator for block sizes and corrections.
    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }
        fn below(&mut self, n: usize) -> usize {
            (self.next() % n as u64) as usize
        }
        fn unit(&mut self) -> f64 {
            (self.next() >> 11) as f64 / (1u64 << 53) as f64
        }
    }

    fn tone(frames: usize, f_in: f64) -> Vec<f32> {
        let w = 2.0 * PI * TONE_HZ / f_in;
        let mut v = Vec::with_capacity(frames * 2);
        for n in 0..frames {
            let p = w * n as f64;
            v.push((AMP * p.sin()) as f32);
            v.push((AMP * p.cos()) as f32);
        }
        v
    }

    /// Least-squares fit y ≈ a·sin θ + b·cos θ + c; returns SINAD in dB.
    #[allow(clippy::needless_range_loop)] // 3x3 normal equations read clearer with indices
    fn sinad_db(y: &[f64], theta: &[f64]) -> f64 {
        let mut m = [[0.0f64; 3]; 3];
        let mut v = [0.0f64; 3];
        for (&yk, &t) in y.iter().zip(theta) {
            let basis = [t.sin(), t.cos(), 1.0];
            for i in 0..3 {
                v[i] += basis[i] * yk;
                for j in 0..3 {
                    m[i][j] += basis[i] * basis[j];
                }
            }
        }
        // Gaussian elimination with partial pivoting.
        for col in 0..3 {
            let piv = (col..3)
                .max_by(|&p, &q| m[p][col].abs().total_cmp(&m[q][col].abs()))
                .unwrap();
            m.swap(col, piv);
            v.swap(col, piv);
            for r in col + 1..3 {
                let f = m[r][col] / m[col][col];
                for c in col..3 {
                    m[r][c] -= f * m[col][c];
                }
                v[r] -= f * v[col];
            }
        }
        let mut x = [0.0; 3];
        for r in (0..3).rev() {
            let s: f64 = v[r] - (r + 1..3).map(|c| m[r][c] * x[c]).sum::<f64>();
            x[r] = s / m[r][r];
        }
        let res: f64 = y
            .iter()
            .zip(theta)
            .map(|(&yk, &t)| {
                let e = yk - (x[0] * t.sin() + x[1] * t.cos() + x[2]);
                e * e
            })
            .sum::<f64>()
            / y.len() as f64;
        let sig = 0.5 * (x[0] * x[0] + x[1] * x[1]);
        10.0 * (sig / res.max(1e-300)).log10()
    }

    /// Resample a 1 kHz stereo tone with a constant correction `delta`, callback sizes drawn
    /// from `rng` (1 ..= max_block) and the input split into two slices at a random point.
    /// Returns the worse channel's SINAD.
    fn tone_sinad(f_in: f64, f_out: f64, hq: bool, delta: f64, max_block: usize) -> f64 {
        const SKIP: usize = 8192;
        const ANALYSE: usize = 32768;
        let total_out = SKIP + ANALYSE;
        let input = tone(
            (total_out as f64 * f_in / f_out * 1.01) as usize + 8192,
            f_in,
        );
        let mut r = RubatoResampler::new(f_in, f_out, hq).unwrap();
        let mut rng = Rng(0x2545_F491_4F6C_DD1D);
        let mut out = vec![0.0f32; MAX_CHUNK_FRAMES * 2];
        let mut left = Vec::with_capacity(total_out);
        let mut right = Vec::with_capacity(total_out);
        let mut pos = 0usize; // input frames consumed
        while left.len() < total_out {
            let m = 1 + rng.below(max_block);
            r.set_correction(delta);
            let need = r.input_needed(m);
            let region = &input[pos * 2..(pos + need) * 2];
            let split = rng.below(need + 1) * 2;
            let (a, b) = region.split_at(split);
            let used = r.process(a, b, need, &mut out[..m * 2]);
            assert_eq!(used, need);
            pos += used;
            for fr in out[..m * 2].chunks_exact(2) {
                left.push(fr[0] as f64);
                right.push(fr[1] as f64);
            }
        }
        // Output frame k plays input position x0 + k·(f_in/f_out)/(1+δ); the fit absorbs x0.
        let w = 2.0 * PI * TONE_HZ / f_in;
        let step = f_in / (f_out * (1.0 + delta));
        let theta: Vec<f64> = (SKIP..total_out).map(|k| w * k as f64 * step).collect();
        let l = sinad_db(&left[SKIP..total_out], &theta);
        let rr = sinad_db(&right[SKIP..total_out], &theta);
        eprintln!("{f_in} -> {f_out} hq={hq} delta={delta:e}: SINAD L {l:.1} dB, R {rr:.1} dB");
        l.min(rr)
    }

    #[test]
    fn sinad_48k_to_48k() {
        let s = tone_sinad(48_000.0, 48_000.0, false, 0.0, 1024);
        assert!(s >= 120.0, "SINAD {s:.1} dB");
        let s = tone_sinad(48_000.0, 48_000.0, false, 300e-6, 512);
        assert!(s >= 120.0, "SINAD with +300 ppm {s:.1} dB");
    }

    #[test]
    fn sinad_44k1_to_48k() {
        let s = tone_sinad(44_100.0, 48_000.0, false, -250e-6, 1024);
        assert!(s >= 120.0, "SINAD {s:.1} dB");
    }

    #[test]
    fn sinad_48k_to_16k() {
        let s = tone_sinad(48_000.0, 16_000.0, false, 1e-3, 1024);
        assert!(s >= 120.0, "SINAD {s:.1} dB");
    }

    #[test]
    fn sinad_high_quality() {
        let s = tone_sinad(44_100.0, 48_000.0, true, 100e-6, 1024);
        assert!(s >= 120.0, "SINAD {s:.1} dB");
    }

    #[test]
    fn construction() {
        assert!(RubatoResampler::new(0.0, 48_000.0, false).is_err());
        assert!(RubatoResampler::new(48_000.0, f64::NAN, false).is_err());
        let r = RubatoResampler::new(48_000.0, 44_100.0, false).unwrap();
        assert_eq!(r.delay_input_frames(), 64.0);
        assert_eq!(r.max_out_frames(), 4096);
        let r = RubatoResampler::new(48_000.0, 44_100.0, true).unwrap();
        assert_eq!(r.delay_input_frames(), 128.0);
    }

    /// Feeds an impulse and checks where it comes out: the output frame produced when `R`
    /// input frames have been consumed before the call plays input position
    /// `R − delay_input_frames()` (plus the elapsed output time).
    #[test]
    fn group_delay_matches_delay_input_frames() {
        for (f_in, f_out, hq) in [
            (48_000.0, 48_000.0, false),
            (48_000.0, 48_000.0, true),
            (44_100.0, 48_000.0, false),
            (48_000.0, 16_000.0, false),
        ] {
            let mut r = RubatoResampler::new(f_in, f_out, hq).unwrap();
            let impulse_at = 3000usize;
            let mut input = vec![0.0f32; 20_000 * 2];
            input[impulse_at * 2] = 1.0;
            input[impulse_at * 2 + 1] = 1.0;
            let step = f_in / f_out;
            let mut out = vec![0.0f32; 256 * 2];
            let mut consumed = 0usize;
            let mut best = (0.0f32, 0.0f64); // (|peak|, input position it represents)
            let mut produced = 0usize;
            while consumed + 4096 < 20_000 {
                let need = r.input_needed(256);
                let region = &input[consumed * 2..(consumed + need) * 2];
                let r_before = consumed as f64;
                consumed += r.process(region, &[], need, &mut out);
                for (k, fr) in out.chunks_exact(2).enumerate() {
                    if fr[0].abs() > best.0 {
                        let x = r_before - r.delay_input_frames() + k as f64 * step;
                        best = (fr[0].abs(), x);
                    }
                }
                produced += 256;
            }
            assert!(produced > 0);
            assert!(
                (best.1 - impulse_at as f64).abs() <= step.max(1.0),
                "{f_in}->{f_out} hq={hq}: peak at input position {:.2}, impulse at {impulse_at}",
                best.1
            );
        }
    }

    #[test]
    fn accounting_is_consistent() {
        let f_in = 44_100.0;
        let f_out = 48_000.0;
        let mut r = RubatoResampler::new(f_in, f_out, false).unwrap();
        let input = tone(1_500_000, f_in);
        let mut out = vec![0.0f32; MAX_CHUNK_FRAMES * 2];
        let mut rng = Rng(7);
        let mut pos = 0usize; // stream position (frames rubato has advanced over)
        let mut out_total = 0usize;
        let mut weighted = 0.0f64; // Σ m / ratio: expected long-run input consumption
        for i in 0..2000 {
            let m = 1 + rng.below(if i % 50 == 0 { MAX_CHUNK_FRAMES } else { 512 });
            let delta = (rng.unit() - 0.5) * 2e-3;
            r.set_correction(delta);
            let need = r.input_needed(m);
            // Asking again without a state change returns the same value.
            assert_eq!(r.input_needed(m), need);
            assert!(need > 0);
            let region = &input[pos * 2..];
            let used = match i % 10 {
                // underrun: only part of the data is there
                3 => {
                    let valid = rng.below(need);
                    let used = r.process(&region[..valid * 2], &[], valid, &mut out[..m * 2]);
                    assert_eq!(used, valid);
                    // The missing tail was zero-filled; the stream advanced by `need`.
                    need
                }
                // more data available than needed: only `need` is consumed
                5 => {
                    let avail = need + rng.below(1000);
                    let split = rng.below(avail + 1) * 2;
                    let (a, b) = region[..avail * 2].split_at(split);
                    let used = r.process(a, b, avail, &mut out[..m * 2]);
                    assert_eq!(used, need);
                    used
                }
                _ => {
                    let split = rng.below(need + 1) * 2;
                    let (a, b) = region[..need * 2].split_at(split);
                    let used = r.process(a, b, need, &mut out[..m * 2]);
                    assert_eq!(used, need);
                    used
                }
            };
            pos += used;
            out_total += m;
            weighted += m as f64 * f_in / (f_out * (1.0 + delta));
        }
        assert!(out_total > 0);
        // Long-run consumption follows the ratio; the slack is the filter history and ramps.
        let slack = (pos as f64 - weighted).abs();
        eprintln!("accounting: consumed {pos}, ratio model {weighted:.1}, slack {slack:.1}");
        assert!(slack < 64.0, "consumed {pos}, expected ≈ {weighted:.0}");

        // After a reset the stream restarts with the same (exact) accounting.
        r.reset();
        let need = r.input_needed(480);
        assert_eq!(
            r.process(&input[..need * 2], &[], need, &mut out[..960]),
            need
        );
        // Empty request.
        assert_eq!(r.input_needed(0), 0);
        assert_eq!(r.process(&[], &[], 0, &mut []), 0);
    }

    #[test]
    fn ten_thousand_callbacks_without_allocation() {
        let f_in = 48_000.0;
        let f_out = 44_100.0;
        let mut r = RubatoResampler::new(f_in, f_out, false).unwrap();
        let mut boxed: Box<dyn VarResampler> =
            Box::new(RubatoResampler::new(48e3, 48e3, true).unwrap());
        let (mut prod, mut cons) = rtrb::RingBuffer::<f32>::new(16_384 * 2);
        let mut out = vec![0.0f32; MAX_CHUNK_FRAMES * 2];
        let mut rng = Rng(42);
        let w = 2.0 * PI * TONE_HZ / f_in;
        let mut n_in = 0u64;

        let ((underruns, frames), allocs) = count_allocs(|| {
            let mut underruns = 0u32;
            let mut frames = 0usize;
            for i in 0..10_000 {
                let m = if i % 997 == 0 {
                    MAX_CHUNK_FRAMES
                } else {
                    1 + rng.below(1024)
                };
                // Capture side: keep the ring stocked, occasionally starve it.
                let target = if i % 101 == 0 { 0 } else { 6000 };
                let have = cons.slots() / 2;
                if have < target {
                    let free = prod.slots() / 2;
                    let add = (target - have).min(free);
                    if let Ok(mut chunk) = prod.write_chunk(add * 2) {
                        let (a, b) = chunk.as_mut_slices();
                        for s in a.chunks_exact_mut(2).chain(b.chunks_exact_mut(2)) {
                            let v = (AMP * (w * n_in as f64).sin()) as f32;
                            s[0] = v;
                            s[1] = -v;
                            n_in += 1;
                        }
                        chunk.commit_all();
                    }
                }
                // Render side, exactly as the renderer uses the trait.
                let res: &mut dyn VarResampler = if i % 3 == 0 { &mut *boxed } else { &mut r };
                res.set_correction((rng.unit() - 0.5) * 2e-3);
                let need = res.input_needed(m);
                let take = need.min(cons.slots() / 2);
                let Ok(chunk) = cons.read_chunk(take * 2) else {
                    continue;
                };
                let (a, b) = chunk.as_slices();
                let used = res.process(a, b, take, &mut out[..m * 2]);
                chunk.commit_all();
                if used < need {
                    underruns += 1;
                }
                if i % 500 == 0 {
                    res.reset();
                }
                frames += m;
            }
            (underruns, frames)
        });
        assert_eq!(allocs, 0, "real-time path allocated");
        assert!(underruns > 0, "the starvation path was not exercised");
        assert!(frames > 1_000_000);
    }
}
