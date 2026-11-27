//! Platform-independent timing logic for the WASAPI streams: QPC conversions, the render
//! presentation-time formula, render discontinuity detection, and the capture timeline that keeps
//! loopback capture continuous across silent periods (no packets) by synthesizing silence and
//! re-anchoring on the next real packet.
//!
//! All functions are allocation-free and panic-free (they run on the audio threads).

/// Nanoseconds per second.
const NS: f64 = 1e9;

/// Converts a 100 ns QPC-derived value (as returned by WASAPI) to nanoseconds.
#[inline]
pub fn hns_to_ns(hns: u64) -> u64 {
    hns.saturating_mul(100)
}

/// Converts raw `QueryPerformanceCounter` ticks to nanoseconds (128-bit intermediate).
#[inline]
pub fn qpc_ticks_to_ns(ticks: u64, freq: u64) -> u64 {
    if freq == 0 {
        return 0;
    }
    let ns = (ticks as u128).saturating_mul(1_000_000_000) / freq as u128;
    u64::try_from(ns).unwrap_or(u64::MAX)
}

/// Frames → nanoseconds at `rate`.
#[inline]
pub fn frames_to_ns(frames: u64, rate: u32) -> u64 {
    if rate == 0 {
        return 0;
    }
    let ns = (frames as u128).saturating_mul(1_000_000_000) / rate as u128;
    u64::try_from(ns).unwrap_or(u64::MAX)
}

/// Host time (ns) at which the first frame of the next render write reaches the speaker:
///
/// `t = qpc·100 + (W / rate − pos / freq) · 1e9`
///
/// where `pos/freq` (from `IAudioClock::GetPosition`) is the stream position currently playing at
/// QPC time `qpc_hns`, and `written` (`W`) is the number of frames written before this write.
pub fn render_first_frame_time(qpc_hns: u64, pos: u64, freq: u64, written: u64, rate: u32) -> u64 {
    let base = hns_to_ns(qpc_hns) as i128;
    let ahead = frames_to_ns(written, rate) as i128;
    let played = if freq == 0 {
        0
    } else {
        ((pos as u128).saturating_mul(1_000_000_000) / freq as u128) as i128
    };
    let t = base + ahead - played;
    t.clamp(0, u64::MAX as i128) as u64
}

/// Detects breaks in the render timeline: the host time of each write should continue the
/// previous one (previous host time + previous frames / rate) within a tolerance.
#[derive(Clone, Copy, Debug)]
pub struct RenderTracker {
    expected_next: Option<u64>,
    tolerance_ns: u64,
}

impl RenderTracker {
    pub fn new(tolerance_ns: u64) -> Self {
        RenderTracker {
            expected_next: None,
            tolerance_ns,
        }
    }

    /// Returns `true` if this write does not continue the previous one, or the device buffer ran
    /// dry (`underrun`) since the previous write. The first write is never a discontinuity.
    pub fn observe(&mut self, host_time: u64, frames: u32, rate: u32, underrun: bool) -> bool {
        let had_reference = self.expected_next.is_some();
        let jumped = match self.expected_next {
            Some(expected) => host_time.abs_diff(expected) > self.tolerance_ns,
            None => false,
        };
        self.expected_next = Some(host_time.saturating_add(frames_to_ns(frames as u64, rate)));
        jumped || (underrun && had_reference)
    }
}

/// How to deliver one capture packet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PacketPlan {
    /// Silent frames to deliver before the packet (fills a timeline gap).
    pub pad_frames: u64,
    /// Host time of the first padding frame.
    pub pad_time: u64,
    /// Leading packet frames to drop (they overlap already-delivered synthesized silence).
    pub skip_frames: u32,
    /// Host time of the first delivered packet frame (after `skip_frames`).
    pub host_time: u64,
    /// The timeline broke and the gap could not be counted.
    pub discontinuity: bool,
}

/// Capture timeline: tracks the host time of the next expected frame.
///
/// - Normal packets are delivered with their own QPC timestamp.
/// - A packet flagged as discontinuous, one that follows synthesized silence, or one whose
///   timestamp deviates from the timeline by at least one period is re-anchored: a gap is filled
///   with silence, an overlap is trimmed from the packet front. Gaps over [`MAX_REANCHOR_NS`]
///   are reported as a discontinuity instead.
/// - When no packet arrives for more than two periods (the engine produces no loopback data while
///   nothing plays), [`CaptureTimeline::on_idle`] synthesizes silence up to one period before now.
#[derive(Clone, Copy, Debug)]
pub struct CaptureTimeline {
    rate: u32,
    period_ns: u64,
    next_ns: Option<f64>,
    synthesized: bool,
    seen_packet: bool,
}

/// Largest gap that is bridged with silence instead of being reported as a discontinuity.
pub const MAX_REANCHOR_NS: u64 = 1_000_000_000;

impl CaptureTimeline {
    pub fn new(rate: u32, period_frames: u32) -> Self {
        CaptureTimeline {
            rate,
            period_ns: frames_to_ns(period_frames.max(1) as u64, rate).max(1),
            next_ns: None,
            synthesized: false,
            seen_packet: false,
        }
    }

    /// Anchors the timeline at stream start so silence can be synthesized even before the first
    /// packet.
    pub fn start(&mut self, now_ns: u64) {
        self.next_ns = Some(now_ns as f64);
    }

    fn frames_ns(&self, frames: u64) -> f64 {
        frames as f64 * NS / self.rate.max(1) as f64
    }

    /// Plans delivery of a packet of `frames` frames.
    ///
    /// `qpc_ns`: the packet's timestamp (`None` if the driver flagged it as unreliable or did not
    /// provide one). `fallback_ns`: used when there is neither a timestamp nor a timeline yet.
    pub fn on_packet(
        &mut self,
        qpc_ns: Option<u64>,
        frames: u32,
        flagged_discontinuity: bool,
        fallback_ns: u64,
    ) -> PacketPlan {
        let first = !self.seen_packet;
        self.seen_packet = true;
        let ts = match (qpc_ns, self.next_ns) {
            (Some(t), _) => t as f64,
            (None, Some(next)) => next,
            (None, None) => fallback_ns as f64,
        };
        let mut plan = PacketPlan {
            pad_frames: 0,
            pad_time: 0,
            skip_frames: 0,
            host_time: ts.max(0.0) as u64,
            discontinuity: false,
        };
        let packet_end = ts + self.frames_ns(frames as u64);

        if let Some(next) = self.next_ns {
            // The first packet's discontinuity flag is undefined: ignore it.
            let flagged = flagged_discontinuity && !first;
            let gap = ts - next;
            let reanchor = self.synthesized || flagged || gap.abs() >= self.period_ns as f64;
            if flagged && qpc_ns.is_none() {
                // Cannot count the lost frames without a timestamp.
                plan.discontinuity = true;
            } else if gap.abs() > MAX_REANCHOR_NS as f64 {
                plan.discontinuity = !first || self.synthesized;
            } else if reanchor && (!first || self.synthesized) {
                if gap > 0.0 {
                    plan.pad_frames = (gap * self.rate as f64 / NS).round() as u64;
                    plan.pad_time = next.max(0.0) as u64;
                } else if gap < 0.0 {
                    let skip = ((-gap) * self.rate as f64 / NS).round() as u64;
                    plan.skip_frames = skip.min(frames as u64) as u32;
                    plan.host_time = (ts + self.frames_ns(plan.skip_frames as u64)).max(0.0) as u64;
                }
            }
        }
        self.synthesized = false;
        self.next_ns = Some(match self.next_ns {
            // Never move the timeline backwards (trimmed overlap keeps `next` where it was).
            Some(next) if next > packet_end && !plan.discontinuity => next,
            _ => packet_end,
        });
        plan
    }

    /// Called when the capture thread wakes without data. Returns `(host_time, frames)` of
    /// silence to synthesize, if more than two periods have passed since the last frame.
    /// At most one second is synthesized per call.
    pub fn on_idle(&mut self, now_ns: u64) -> Option<(u64, u64)> {
        let next = self.next_ns?;
        let elapsed = now_ns as f64 - next;
        if elapsed <= 2.0 * self.period_ns as f64 {
            return None;
        }
        let span = elapsed - self.period_ns as f64;
        let frames = ((span * self.rate as f64 / NS).floor() as u64).min(self.rate as u64);
        if frames == 0 {
            return None;
        }
        self.next_ns = Some(next + self.frames_ns(frames));
        self.synthesized = true;
        Some((next.max(0.0) as u64, frames))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: u32 = 48_000;
    const PERIOD: u32 = 480; // 10 ms

    #[test]
    fn qpc_conversions() {
        assert_eq!(hns_to_ns(12_345), 1_234_500);
        assert_eq!(qpc_ticks_to_ns(10_000_000, 10_000_000), 1_000_000_000);
        assert_eq!(qpc_ticks_to_ns(3, 3_000_000), 1_000);
        assert_eq!(qpc_ticks_to_ns(u64::MAX, 1), u64::MAX);
        assert_eq!(qpc_ticks_to_ns(5, 0), 0);
        assert_eq!(frames_to_ns(480, 48_000), 10_000_000);
    }

    #[test]
    fn render_time_formula() {
        // 1 s on the QPC clock, 0.5 s played, 0.6 s written → first new frame plays 0.1 s later.
        let freq = 48_000 * 8; // position units need not be frames
        let t = render_first_frame_time(10_000_000, 24_000 * 8, freq, 28_800, RATE);
        assert_eq!(t, 1_100_000_000);
        // Never negative.
        assert_eq!(render_first_frame_time(0, 48_000, 48_000, 0, RATE), 0);
    }

    #[test]
    fn render_tracker_flags_jumps_and_underruns() {
        let mut tr = RenderTracker::new(2_000_000);
        assert!(!tr.observe(1_000_000_000, PERIOD, RATE, false));
        assert!(!tr.observe(1_010_000_500, PERIOD, RATE, false));
        assert!(tr.observe(1_030_000_000, PERIOD, RATE, false)); // 10 ms jump
        assert!(!tr.observe(1_040_000_000, PERIOD, RATE, false));
        assert!(tr.observe(1_050_000_000, PERIOD, RATE, true)); // underrun
    }

    #[test]
    fn continuous_packets_pass_through() {
        let mut tl = CaptureTimeline::new(RATE, PERIOD);
        tl.start(0);
        let p = tl.on_packet(Some(5_000_000), PERIOD, true, 0);
        assert_eq!(p.pad_frames, 0);
        assert_eq!(p.skip_frames, 0);
        assert!(
            !p.discontinuity,
            "first packet's discontinuity flag is ignored"
        );
        assert_eq!(p.host_time, 5_000_000);
        // Small jitter (0.3 ms) is delivered as-is.
        let p = tl.on_packet(Some(15_300_000), PERIOD, false, 0);
        assert_eq!(
            (p.pad_frames, p.skip_frames, p.host_time),
            (0, 0, 15_300_000)
        );
    }

    #[test]
    fn flagged_gap_is_filled_with_silence() {
        let mut tl = CaptureTimeline::new(RATE, PERIOD);
        tl.start(0);
        tl.on_packet(Some(0), PERIOD, false, 0);
        // Next expected at 10 ms; packet arrives at 15 ms flagged → 240 frames of padding.
        let p = tl.on_packet(Some(15_000_000), PERIOD, true, 0);
        assert_eq!(p.pad_frames, 240);
        assert_eq!(p.pad_time, 10_000_000);
        assert!(!p.discontinuity);
    }

    #[test]
    fn idle_synthesizes_then_reanchors() {
        let mut tl = CaptureTimeline::new(RATE, PERIOD);
        tl.start(0);
        tl.on_packet(Some(0), PERIOD, false, 0); // next = 10 ms
        assert_eq!(tl.on_idle(25_000_000), None, "not yet two periods");
        // 40 ms: elapsed 30 ms > 20 ms → synthesize 20 ms (keeps one period of slack).
        assert_eq!(tl.on_idle(40_000_000), Some((10_000_000, 960)));
        // Real packet at 35 ms: timeline at 30 ms → pad 5 ms.
        let p = tl.on_packet(Some(35_000_000), PERIOD, false, 0);
        assert_eq!((p.pad_frames, p.pad_time), (240, 30_000_000));
        // Synthesize again, then a packet that overlaps the synthesized range.
        assert_eq!(tl.on_idle(80_000_000), Some((45_000_000, 1200)));
        let p = tl.on_packet(Some(65_000_000), PERIOD, false, 0); // timeline at 70 ms
        assert_eq!(p.skip_frames, 240);
        assert_eq!(p.host_time, 70_000_000);
        assert_eq!(p.pad_frames, 0);
    }

    #[test]
    fn idle_before_first_packet_uses_start_anchor() {
        let mut tl = CaptureTimeline::new(RATE, PERIOD);
        tl.start(1_000_000_000);
        assert_eq!(tl.on_idle(1_030_000_000), Some((1_000_000_000, 960)));
        // First packet after synthesized silence is re-anchored too.
        let p = tl.on_packet(Some(1_025_000_000), PERIOD, true, 0);
        assert_eq!(p.pad_frames, 240);
        assert!(!p.discontinuity);
    }

    #[test]
    fn huge_gap_is_a_discontinuity() {
        let mut tl = CaptureTimeline::new(RATE, PERIOD);
        tl.start(0);
        tl.on_packet(Some(0), PERIOD, false, 0);
        let p = tl.on_packet(Some(5_000_000_000), PERIOD, true, 0);
        assert!(p.discontinuity);
        assert_eq!(p.pad_frames, 0);
        assert_eq!(p.host_time, 5_000_000_000);
        // Timeline follows the new packet.
        let p = tl.on_packet(Some(5_010_000_000), PERIOD, false, 0);
        assert!(!p.discontinuity);
        assert_eq!(p.pad_frames, 0);
    }

    #[test]
    fn missing_timestamp_is_extrapolated() {
        let mut tl = CaptureTimeline::new(RATE, PERIOD);
        tl.start(0);
        tl.on_packet(Some(100_000_000), PERIOD, false, 0);
        let p = tl.on_packet(None, PERIOD, false, 0);
        assert_eq!(p.host_time, 110_000_000);
        assert!(!p.discontinuity);
        let p = tl.on_packet(None, PERIOD, true, 0);
        assert!(p.discontinuity, "uncountable loss");
    }

    #[test]
    fn idle_is_capped_to_one_second() {
        let mut tl = CaptureTimeline::new(RATE, PERIOD);
        tl.start(0);
        assert_eq!(tl.on_idle(10_000_000_000), Some((0, 48_000)));
    }
}
