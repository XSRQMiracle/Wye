//! Per-device output: event-driven shared-mode render stream in the device's mix format (Wye
//! resamples itself, so no AUTOCONVERTPCM), buffer of two default engine periods.
//!
//! Presentation time of the first frame of each write comes from `IAudioClock::GetPosition`:
//! `t = qpc·100 + (W/rate − pos/freq)·1e9`, where `W` counts frames written before this write.

use std::time::Duration;

use windows::Win32::Media::Audio::{
    IAudioClient, IAudioClock, IAudioRenderClient, AUDCLNT_BUFFERFLAGS_SILENT,
    AUDCLNT_SHAREMODE_SHARED, AUDCLNT_STREAMFLAGS_EVENTCALLBACK,
};
use windows::Win32::System::Com::CLSCTX_ALL;
use wye_platform::{
    DeviceId, OutputConfig, PlatformError, RenderHandler, RenderInfo, StreamError, StreamFormat,
    TimestampQuality,
};

use crate::capture::{SessionDisconnect, MAX_BLOCK_FRAMES};
use crate::classify::hr;
use crate::com::{
    default_period_hns, enumerator, hns_to_frames, os_err, set_media_category, stream_err,
    wait_any, Event, MixFormat, Qpc, StreamLoop, Wake,
};
use crate::devices::get_device;
use crate::format::{encode_from_stereo, StereoPick, WaveFormat};
use crate::timing::{frames_to_ns, render_first_frame_time, RenderTracker};

/// Tolerance of the render timeline continuity check.
const DISCONTINUITY_TOLERANCE_NS: u64 = 2_000_000;

/// Consecutive wait timeouts (each two periods long) after which the device is considered stalled.
const MAX_CONSECUTIVE_TIMEOUTS: u32 = 50;

/// Maps an `Initialize` failure on a specific device to a platform error.
pub(crate) fn init_error(
    call: &'static str,
    id: &DeviceId,
    e: &windows_core::Error,
) -> PlatformError {
    match e.code().0 {
        hr::AUDCLNT_E_DEVICE_IN_USE | hr::AUDCLNT_E_EXCLUSIVE_MODE_NOT_ALLOWED => {
            PlatformError::DeviceBusy(id.clone())
        }
        hr::AUDCLNT_E_DEVICE_INVALIDATED | hr::E_NOTFOUND => {
            PlatformError::DeviceNotFound(id.clone())
        }
        _ => os_err(call, e),
    }
}

pub(crate) struct RenderStream {
    client: IAudioClient,
    render: IAudioRenderClient,
    clock: IAudioClock,
    clock_freq: u64,
    disconnect: SessionDisconnect,
    event: Event,
    format: WaveFormat,
    pick: StereoPick,
    buffer_frames: u32,
    period_ms: u32,
    /// Frames written since the stream started (including the silent prefill).
    written: u64,
    tracker: RenderTracker,
    scratch: Vec<f32>,
    handler: Box<dyn RenderHandler>,
    qpc: Qpc,
    stream_format: StreamFormat,
    _mix: MixFormat,
}

impl RenderStream {
    pub(crate) fn open(
        id: &DeviceId,
        _cfg: OutputConfig,
        handler: Box<dyn RenderHandler>,
    ) -> Result<RenderStream, PlatformError> {
        let en = enumerator()?;
        let device = get_device(&en, id)?;
        // SAFETY: plain COM calls on interfaces created on this thread.
        unsafe {
            let client: IAudioClient = device
                .Activate(CLSCTX_ALL, None)
                .map_err(|e| init_error("Activate(IAudioClient)", id, &e))?;
            set_media_category(&client);
            let mix = MixFormat::of(&client)?;
            let format = mix.format;
            let period_hns = default_period_hns(&client);
            client
                .Initialize(
                    AUDCLNT_SHAREMODE_SHARED,
                    AUDCLNT_STREAMFLAGS_EVENTCALLBACK,
                    2 * period_hns,
                    0,
                    mix.as_ptr(),
                    None,
                )
                .map_err(|e| init_error("Initialize(render)", id, &e))?;
            let event = Event::new(false)?;
            client
                .SetEventHandle(event.handle())
                .map_err(|e| os_err("SetEventHandle(render)", &e))?;
            let render: IAudioRenderClient = client
                .GetService()
                .map_err(|e| os_err("GetService(IAudioRenderClient)", &e))?;
            let clock: IAudioClock = client
                .GetService()
                .map_err(|e| os_err("GetService(IAudioClock)", &e))?;
            let clock_freq = clock
                .GetFrequency()
                .map_err(|e| os_err("IAudioClock::GetFrequency", &e))?;
            let buffer_frames = client
                .GetBufferSize()
                .map_err(|e| os_err("GetBufferSize(render)", &e))?;
            let disconnect = SessionDisconnect::register(&client)?;
            // Prefill the whole buffer with silence, then start.
            render
                .GetBuffer(buffer_frames)
                .map_err(|e| os_err("GetBuffer(prefill)", &e))?;
            render
                .ReleaseBuffer(buffer_frames, AUDCLNT_BUFFERFLAGS_SILENT.0 as u32)
                .map_err(|e| os_err("ReleaseBuffer(prefill)", &e))?;
            client
                .Start()
                .map_err(|e| init_error("Start(render)", id, &e))?;
            let period_frames = hns_to_frames(period_hns, format.rate).max(1);
            let max_frames = MAX_BLOCK_FRAMES.max(buffer_frames as usize);
            Ok(RenderStream {
                client,
                render,
                clock,
                clock_freq,
                disconnect,
                event,
                pick: StereoPick::from_mask(format.channels, format.channel_mask),
                format,
                buffer_frames,
                period_ms: ((period_hns / 10_000) as u32).max(1),
                written: buffer_frames as u64,
                tracker: RenderTracker::new(DISCONTINUITY_TOLERANCE_NS),
                scratch: vec![0.0; max_frames * 2],
                handler,
                qpc: Qpc::new(),
                stream_format: StreamFormat {
                    rate: format.rate as f64,
                    block_frames: period_frames,
                },
                _mix: mix,
            })
        }
    }

    /// Fills all writable frames. Real-time code.
    fn process(&mut self) -> windows_core::Result<()> {
        // SAFETY: COM calls on this thread; GetBuffer/ReleaseBuffer strictly paired.
        unsafe {
            let padding = self.client.GetCurrentPadding()?;
            let avail = self.buffer_frames.saturating_sub(padding);
            if avail == 0 {
                return Ok(());
            }
            let rate = self.format.rate;
            let mut pos = 0u64;
            let mut qpc_hns = 0u64;
            self.clock.GetPosition(&mut pos, Some(&mut qpc_hns))?;
            let (host_time, quality) = if pos == 0 || qpc_hns == 0 {
                // The position stays 0 for the first milliseconds of a stream: estimate from the
                // queued frames instead.
                (
                    self.qpc
                        .now_ns()
                        .saturating_add(frames_to_ns(padding as u64, rate)),
                    TimestampQuality::CallbackClock,
                )
            } else {
                (
                    render_first_frame_time(qpc_hns, pos, self.clock_freq, self.written, rate),
                    TimestampQuality::Device,
                )
            };
            let discontinuity = self.tracker.observe(host_time, avail, rate, padding == 0);

            let data = self.render.GetBuffer(avail)?;
            let block = self.format.block_align as usize;
            let total = avail as usize;
            let dst: &mut [u8] = if data.is_null() {
                &mut []
            } else {
                std::slice::from_raw_parts_mut(data, total * block)
            };
            let max = self.scratch.len() / 2;
            let mut done = 0usize;
            while done < total {
                let n = (total - done).min(max);
                let stereo = &mut self.scratch[..n * 2];
                stereo.fill(0.0);
                let info = RenderInfo {
                    host_time: host_time.saturating_add(frames_to_ns(done as u64, rate)),
                    quality,
                    discontinuity: discontinuity && done == 0,
                    after_timestamp: Duration::ZERO,
                };
                self.handler.on_render(stereo, &info);
                if let Some(out) = dst.get_mut(done * block..) {
                    encode_from_stereo(stereo, n, &self.format, self.pick, out);
                }
                done += n;
            }
            let flags = if dst.is_empty() {
                AUDCLNT_BUFFERFLAGS_SILENT.0 as u32
            } else {
                0
            };
            self.render.ReleaseBuffer(avail, flags)?;
            self.written = self.written.saturating_add(avail as u64);
        }
        Ok(())
    }
}

impl StreamLoop for RenderStream {
    fn format(&self) -> StreamFormat {
        self.stream_format
    }

    fn run(&mut self, stop: &Event) -> Result<(), StreamError> {
        let handles = [
            stop.handle(),
            self.disconnect.event().handle(),
            self.event.handle(),
        ];
        let timeout = self.period_ms.saturating_mul(2);
        let mut timeouts = 0u32;
        let result = loop {
            match wait_any(&handles, timeout) {
                Wake::Signalled(0) => break Ok(()),
                Wake::Signalled(1) => break Err(self.disconnect.error()),
                Wake::Signalled(_) => {
                    timeouts = 0;
                    if let Err(e) = self.process() {
                        break Err(stream_err(&e));
                    }
                }
                Wake::Timeout => {
                    timeouts += 1;
                    if timeouts >= MAX_CONSECUTIVE_TIMEOUTS {
                        // The engine stopped requesting data (stalled or removed device).
                        break Err(StreamError::DeviceGone);
                    }
                    if let Err(e) = self.process() {
                        break Err(stream_err(&e));
                    }
                }
                Wake::Failed => break Err(StreamError::Os(-1)),
            }
        };
        // SAFETY: stops the stream started in `open`.
        let _ = unsafe { self.client.Stop() };
        result
    }
}
