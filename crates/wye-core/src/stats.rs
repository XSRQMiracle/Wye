//! Lock-free statistics shared from real-time callbacks to the control/UI thread.

use std::sync::atomic::{AtomicU64, Ordering};

/// f64 stored as bits in an AtomicU64 (Relaxed is enough: values are independent gauges).
#[derive(Default)]
pub struct AtomicF64(AtomicU64);
impl AtomicF64 {
    pub fn store(&self, v: f64) {
        self.0.store(v.to_bits(), Ordering::Relaxed)
    }
    pub fn load(&self) -> f64 {
        f64::from_bits(self.0.load(Ordering::Relaxed))
    }
}

/// Written by one render callback, read by the control thread (debug panel, health checks).
#[derive(Default)]
pub struct DeviceStats {
    pub callbacks: AtomicU64,
    pub last_callback_host_ns: AtomicU64,
    /// Filtered latency error e_f (s).
    pub error_filtered: AtomicF64,
    /// Raw latency error e of the last callback (s).
    pub error_raw: AtomicF64,
    /// Current ratio correction δ.
    pub delta: AtomicF64,
    /// Measured λ (s).
    pub lambda: AtomicF64,
    /// Source frames buffered for this device (ring + resampler), seconds.
    pub buffered: AtomicF64,
    pub underruns: AtomicU64,
    pub resyncs: AtomicU64,
    pub discontinuities: AtomicU64,
    pub saturated_callbacks: AtomicU64,
}

#[derive(Default)]
pub struct CaptureStats {
    pub callbacks: AtomicU64,
    pub frames: AtomicU64,
    pub last_callback_host_ns: AtomicU64,
    /// Frames dropped because a device ring was full (per callback, any device).
    pub overflows: AtomicU64,
    pub discontinuities: AtomicU64,
    /// Peak |sample| of the last second (for "is anything playing" and permission inference).
    pub peak: AtomicF64,
    /// Count of callbacks whose samples were all exactly 0.0.
    pub all_zero_callbacks: AtomicU64,
}
