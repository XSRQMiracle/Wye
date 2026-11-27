//! Real-time objects: capture fan-out and per-device renderer.
//! Everything here runs on OS real-time threads: no allocation, no locks, no I/O, no logging.
//! Structural changes arrive through SPSC command rings; objects that must be freed are sent
//! back to the control thread through "graveyard" rings.

use crate::anchor::Anchor;
use crate::control::Controller;
use crate::gain::GainSmoother;
use crate::resample::VarResampler;
use crate::stats::{CaptureStats, DeviceStats};
use std::sync::atomic::{AtomicBool, AtomicI64, AtomicU64};
use std::sync::Arc;
use wye_platform::{CaptureHandler, CaptureInfo, RenderHandler, RenderInfo};

/// Maximum simultaneously rendered devices.
pub const MAX_OUTPUTS: usize = 8;

/// State shared between the capture slot feeding one device ring and that device's renderer.
pub struct SlotShared {
    /// Global frame index of the first frame written into the ring (`u64::MAX` = not yet).
    pub start_frame: AtomicU64,
    /// Set by the capture callback when the ring was full and a block was dropped.
    pub overflowed: AtomicBool,
    /// Set by the renderer when its target was unreachable: λ_eff − λ_target in ns (0 = ok).
    pub late_ns: AtomicI64,
    /// Host time (ns) of the first sample of the last test signal played (calibration).
    pub test_start_host_ns: AtomicU64,
}

impl SlotShared {
    pub fn new() -> Self {
        SlotShared {
            start_frame: AtomicU64::new(u64::MAX),
            overflowed: AtomicBool::new(false),
            late_ns: AtomicI64::new(0),
            test_start_host_ns: AtomicU64::new(0),
        }
    }
}

impl Default for SlotShared {
    fn default() -> Self {
        Self::new()
    }
}

/// Commands from the control thread to the capture callback.
pub enum CaptureCmd {
    /// Start feeding `producer` (interleaved stereo f32 ring) in `slot`.
    Attach {
        slot: usize,
        producer: rtrb::Producer<f32>,
        shared: Arc<SlotShared>,
    },
    /// Stop feeding `slot`; producer and shared handle go back through the graveyard.
    Detach { slot: usize },
    /// Add a click train to every slot (alignment helper): clicks start at
    /// global frame `start_frame`, every `period_frames`, `count` times.
    InjectClicks {
        start_frame: u64,
        period_frames: u64,
        count: u32,
    },
}

/// Things the capture callback hands back for deallocation on the control thread.
pub enum CaptureGarbage {
    Slot(rtrb::Producer<f32>, Arc<SlotShared>),
}

/// Capture callback: copies every block into each attached device ring and publishes the anchor.
pub struct CaptureFanout {
    pub slots: [Option<(rtrb::Producer<f32>, Arc<SlotShared>)>; MAX_OUTPUTS],
    pub cmds: rtrb::Consumer<CaptureCmd>,
    pub graveyard: rtrb::Producer<CaptureGarbage>,
    pub anchor: Arc<Anchor>,
    pub rate: f64,
    /// Global index of the next frame (counts inserted silence too). Starts at 0; a new
    /// CaptureFanout and Anchor are created on every capture rebuild.
    pub frames: u64,
    pub stats: Arc<CaptureStats>,
}

impl CaptureHandler for CaptureFanout {
    fn on_capture(&mut self, _stereo: &[f32], _info: &CaptureInfo) {
        todo!("capture fan-out")
    }
}

/// Commands from the control thread to a render callback. Boxed payloads are moved in and the
/// replaced objects go back through the renderer's graveyard.
pub enum RenderCmd {
    /// New target λ in ns. `jump = true` performs an immediate crossfaded re-position (R4);
    /// otherwise the PI loop slews to it. `margin_frames` = (T_cap_block + J_i)·f_in, the
    /// headroom R4 keeps so that λ_min of §5.3 is exactly reachable. Clears `late_ns`.
    SetTarget {
        lambda_ns: i64,
        jump: bool,
        margin_frames: u32,
    },
    /// Slider position 0..=1 already multiplied with the system-volume factor (§5.5).
    SetGain(f32),
    /// Calibrated output-timestamp → acoustic latency (ns); `None` = use `RenderInfo::after_timestamp`.
    SetCalibratedAfter(Option<i64>),
    Mute(bool),
    /// Replace the resampler (source or device rate changed). Implies a jump.
    SwapResampler(Box<dyn VarResampler>),
    /// Play a pre-generated mono test signal directly into the output after gain (calibration).
    PlayTestSignal(Box<[f32]>),
}

pub enum RenderGarbage {
    Resampler(Box<dyn VarResampler>),
    TestSignal(Box<[f32]>),
}

/// Render callback for one output device.
pub struct MirrorRenderer {
    pub ring: rtrb::Consumer<f32>,
    pub shared: Arc<SlotShared>,
    pub cmds: rtrb::Consumer<RenderCmd>,
    pub graveyard: rtrb::Producer<RenderGarbage>,
    pub anchor: Arc<Anchor>,
    pub resampler: Box<dyn VarResampler>,
    pub controller: Controller,
    pub gain: GainSmoother,
    pub target_gain: f32,
    pub muted: bool,
    pub src_rate: f64,
    pub out_rate: f64,
    pub target_lambda_ns: i64,
    /// Effective target actually used by the controller (= target, or the reachable λ when the
    /// target is unreachable; then `shared.late_ns = lambda_eff_ns − target_lambda_ns`).
    pub lambda_eff_ns: i64,
    pub margin_frames: u32,
    pub calibrated_after_ns: Option<i64>,
    /// Global index of the next input frame to read from the ring (= ring read pointer,
    /// `start_frame + frames popped`). Advanced only by frames actually read or skipped.
    pub p_next_input: u64,
    /// Output frames of silence still to emit before resuming (inserted delay).
    pub silence_out: u64,
    /// Remaining frames of fade-in after a (re)position.
    pub fade_in_left: u32,
    pub positioned: bool,
    /// Test signal being played and its read position.
    pub test_signal: Option<(Box<[f32]>, usize)>,
    pub stats: Arc<DeviceStats>,
}

impl RenderHandler for MirrorRenderer {
    fn on_render(&mut self, _stereo: &mut [f32], _info: &RenderInfo) {
        todo!("renderer")
    }
}
