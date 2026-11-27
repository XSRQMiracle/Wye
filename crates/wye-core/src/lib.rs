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

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    Platform(#[from] wye_platform::PlatformError),
    #[error("settings: {0}")]
    Settings(String),
}
