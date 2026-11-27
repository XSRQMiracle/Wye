//! Platform-independent core of Wye.
//!
//! Module map:
//! - [`control`]  — per-device latency controller
//! - [`anchor`]   — seqlock capture anchor (frame index ↔ host time)
//! - [`align`]    — reference/takeover mode selection and target latencies (pure, tested)
//! - [`resample`] — variable-ratio resampler trait
//! - [`pipeline`] — real-time capture fan-out and per-device renderer
//! - [`gain`]     — volume taper and smoothing
//! - [`calib`]    — acoustic calibration algorithms
//! - [`stats`]    — lock-free statistics for the debug panel
//! - [`settings`] — persisted settings model
//! - [`engine`]   — control thread, state machine, public API for GUI/CLI

pub mod align;
pub mod anchor;
pub mod calib;
pub mod control;
pub mod engine;
pub mod gain;
pub mod pipeline;
pub mod resample;
pub mod settings;
pub mod stats;

/// Unit tests run under an allocator that flags any (de)allocation made inside
/// `assert_no_alloc`, so real-time code paths can be proven allocation-free.
#[cfg(test)]
#[global_allocator]
static TEST_ALLOCATOR: assert_no_alloc::AllocDisabler = assert_no_alloc::AllocDisabler;

#[cfg(test)]
pub(crate) mod test_alloc {
    /// Runs `f` with allocation forbidden on the current thread and returns its result with
    /// the number of allocator calls it attempted (0 for real-time safe code). Violations
    /// are counted instead of aborting so the test can report them.
    pub fn count_allocs<T>(f: impl FnOnce() -> T) -> (T, u32) {
        let before = assert_no_alloc::violation_count();
        let r = assert_no_alloc::assert_no_alloc(f);
        (r, assert_no_alloc::violation_count() - before)
    }

    #[test]
    fn detects_allocation() {
        let (_, n) = count_allocs(|| std::hint::black_box(vec![1u8; 16]).len());
        assert!(n > 0);
        let (_, n) = count_allocs(|| std::hint::black_box(3) + 4);
        assert_eq!(n, 0);
    }
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    Platform(#[from] wye_platform::PlatformError),
    #[error("settings: {0}")]
    Settings(String),
}
