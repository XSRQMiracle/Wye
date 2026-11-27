//! Proves that the platform-independent code running on the audio threads (capture timeline,
//! render discontinuity tracking, sample format conversion) never touches the heap.

use assert_no_alloc::{assert_no_alloc, AllocDisabler};

use crate::format::{
    decode_channel, decode_to_stereo, encode_from_stereo, RawWaveFormat, StereoPick, WaveFormat,
    SUBTYPE_IEEE_FLOAT, SUBTYPE_PCM, TAG_EXTENSIBLE,
};
use crate::timing::{frames_to_ns, render_first_frame_time, CaptureTimeline, RenderTracker};

#[global_allocator]
static ALLOCATOR: AllocDisabler = AllocDisabler;

const MAX_FRAMES: usize = 4096;

fn format(channels: u16, bits: u16, sub: u128, mask: u32) -> WaveFormat {
    WaveFormat::parse(&RawWaveFormat {
        tag: TAG_EXTENSIBLE,
        channels,
        rate: 48_000,
        block_align: bits / 8 * channels,
        bits,
        extensible: Some((bits, mask, sub)),
    })
    .expect("valid format")
}

#[test]
fn capture_timeline_does_not_allocate() {
    let rate = 48_000;
    let period = 480u32;
    let mut timeline = CaptureTimeline::new(rate, period);
    timeline.start(1_000_000);
    let mut t = 1_000_000u64 + frames_to_ns(period as u64, rate);
    let mut delivered = 0u64;
    assert_no_alloc(|| {
        for i in 0..10_000u64 {
            // Every 100th packet goes missing for a while (silence synthesis), every 333rd is
            // flagged or has no timestamp.
            if i % 100 == 50 {
                t += 5 * frames_to_ns(period as u64, rate);
                if let Some((_, frames)) = timeline.on_idle(t) {
                    delivered += frames;
                }
            }
            let qpc = (i % 333 != 7).then_some(t);
            let plan = timeline.on_packet(qpc, period, i % 333 == 100, t);
            delivered += plan.pad_frames + (period - plan.skip_frames) as u64;
            t += frames_to_ns(period as u64, rate);
        }
    });
    assert!(delivered > 0);
}

#[test]
fn render_timing_does_not_allocate() {
    let mut tracker = RenderTracker::new(2_000_000);
    let mut breaks = 0u32;
    assert_no_alloc(|| {
        let mut written = 960u64;
        for i in 0..10_000u64 {
            let pos = i * 480;
            let t = render_first_frame_time(1_000_000 + i * 100_000, pos, 48_000, written, 48_000);
            if tracker.observe(t, 480, 48_000, false) {
                breaks += 1;
            }
            written += 480;
        }
    });
    assert_eq!(breaks, 0);
}

#[test]
fn sample_conversion_does_not_allocate() {
    let formats = [
        format(2, 32, SUBTYPE_IEEE_FLOAT, 0x3),
        format(8, 32, SUBTYPE_IEEE_FLOAT, 0x63F),
        format(2, 16, SUBTYPE_PCM, 0x3),
        format(1, 24, SUBTYPE_PCM, 0x4),
    ];
    let device = vec![0u8; MAX_FRAMES * 8 * 4];
    let mut device_out = vec![0u8; MAX_FRAMES * 8 * 4];
    let mut stereo = vec![0.25f32; MAX_FRAMES * 2];
    let mut mono = vec![0.0f32; MAX_FRAMES];
    assert_no_alloc(|| {
        for fmt in &formats {
            let pick = StereoPick::from_mask(fmt.channels, fmt.channel_mask);
            for _ in 0..100 {
                decode_to_stereo(&device, MAX_FRAMES, fmt, pick, &mut stereo);
                decode_channel(&device, MAX_FRAMES, fmt, 0, &mut mono);
                encode_from_stereo(&stereo, MAX_FRAMES, fmt, pick, &mut device_out);
            }
        }
    });
}
