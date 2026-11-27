//! System-mix capture: event-driven endpoint loopback on the default render endpoint.
//!
//! The audio engine produces no loopback packets while nothing plays, so the stream keeps the
//! endpoint busy with a silent shared-mode render stream (keep-alive) and, as a fallback,
//! synthesizes silence from the QPC clock when no packet arrived for more than two periods. The
//! next real packet re-anchors the timeline (see [`CaptureTimeline`]).

use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::Arc;

use windows::Win32::Media::Audio::{
    eConsole, eRender, AudioSessionDisconnectReason, AudioSessionState, IAudioCaptureClient,
    IAudioClient, IAudioRenderClient, IAudioSessionControl, IAudioSessionEvents,
    IAudioSessionEvents_Impl, IMMDevice, AUDCLNT_BUFFERFLAGS_DATA_DISCONTINUITY,
    AUDCLNT_BUFFERFLAGS_SILENT, AUDCLNT_BUFFERFLAGS_TIMESTAMP_ERROR, AUDCLNT_SHAREMODE_SHARED,
    AUDCLNT_STREAMFLAGS_EVENTCALLBACK, AUDCLNT_STREAMFLAGS_LOOPBACK,
};
use windows::Win32::System::Com::CLSCTX_ALL;
use windows_core::{implement, BOOL, GUID, PCWSTR};
use wye_platform::{
    CaptureConfig, CaptureHandler, CaptureInfo, PlatformError, StreamError, StreamFormat,
    TimestampQuality,
};

use crate::classify::stream_error_from_disconnect;
use crate::com::{
    default_period_hns, enumerator, hns_to_frames, os_err, set_media_category, stream_err,
    wait_any, Event, MixFormat, Qpc, StreamLoop, Wake,
};
use crate::format::{decode_to_stereo, StereoPick, WaveFormat};
use crate::timing::{frames_to_ns, hns_to_ns, CaptureTimeline};

/// Largest block passed to a handler (frames); larger packets are split.
pub(crate) const MAX_BLOCK_FRAMES: usize = 4096;

/// Wait timeout of the capture loop (ms): wakes up to synthesize silence while no packets come.
const CAPTURE_TIMEOUT_MS: u32 = 20;

/// Watches the stream's audio session for `OnSessionDisconnected` and wakes the stream thread.
#[implement(IAudioSessionEvents)]
pub(crate) struct DisconnectWatch {
    reason: Arc<AtomicI32>,
    event: Arc<Event>,
}

impl IAudioSessionEvents_Impl for DisconnectWatch_Impl {
    fn OnDisplayNameChanged(&self, _: &PCWSTR, _: *const GUID) -> windows_core::Result<()> {
        Ok(())
    }
    fn OnIconPathChanged(&self, _: &PCWSTR, _: *const GUID) -> windows_core::Result<()> {
        Ok(())
    }
    fn OnSimpleVolumeChanged(&self, _: f32, _: BOOL, _: *const GUID) -> windows_core::Result<()> {
        Ok(())
    }
    fn OnChannelVolumeChanged(
        &self,
        _: u32,
        _: *const f32,
        _: u32,
        _: *const GUID,
    ) -> windows_core::Result<()> {
        Ok(())
    }
    fn OnGroupingParamChanged(&self, _: *const GUID, _: *const GUID) -> windows_core::Result<()> {
        Ok(())
    }
    fn OnStateChanged(&self, _: AudioSessionState) -> windows_core::Result<()> {
        Ok(())
    }
    fn OnSessionDisconnected(
        &self,
        reason: AudioSessionDisconnectReason,
    ) -> windows_core::Result<()> {
        self.reason.store(reason.0, Ordering::Release);
        self.event.set();
        Ok(())
    }
}

/// Registration of a [`DisconnectWatch`] on a client's session; unregistered on drop.
pub(crate) struct SessionDisconnect {
    control: Option<(IAudioSessionControl, IAudioSessionEvents)>,
    reason: Arc<AtomicI32>,
    event: Arc<Event>,
}

impl SessionDisconnect {
    pub(crate) fn register(client: &IAudioClient) -> Result<SessionDisconnect, PlatformError> {
        let reason = Arc::new(AtomicI32::new(-1));
        let event = Arc::new(Event::new(false)?);
        let watch: IAudioSessionEvents = DisconnectWatch {
            reason: reason.clone(),
            event: event.clone(),
        }
        .into();
        // SAFETY: plain COM calls; a missing session control only disables disconnect events
        // (failures still surface as HRESULTs).
        let control = unsafe { client.GetService::<IAudioSessionControl>() }
            .ok()
            .and_then(|c| {
                unsafe { c.RegisterAudioSessionNotification(&watch) }
                    .ok()
                    .map(|()| (c, watch))
            });
        Ok(SessionDisconnect {
            control,
            reason,
            event,
        })
    }

    pub(crate) fn event(&self) -> &Event {
        &self.event
    }

    pub(crate) fn error(&self) -> StreamError {
        stream_error_from_disconnect(self.reason.load(Ordering::Acquire))
    }
}

impl Drop for SessionDisconnect {
    fn drop(&mut self) {
        if let Some((control, watch)) = self.control.take() {
            // SAFETY: unregisters the callback registered in `register`.
            let _ = unsafe { control.UnregisterAudioSessionNotification(&watch) };
        }
    }
}

/// Silent render stream on the captured endpoint so the engine keeps producing loopback packets.
struct KeepAlive {
    client: IAudioClient,
    render: IAudioRenderClient,
    event: Event,
    buffer_frames: u32,
}

impl KeepAlive {
    fn open(device: &IMMDevice) -> Result<KeepAlive, PlatformError> {
        // SAFETY: plain COM calls on interfaces created on this thread.
        unsafe {
            let client: IAudioClient = device
                .Activate(CLSCTX_ALL, None)
                .map_err(|e| os_err("Activate(keep-alive)", &e))?;
            set_media_category(&client);
            let mix = MixFormat::of(&client)?;
            client
                .Initialize(
                    AUDCLNT_SHAREMODE_SHARED,
                    AUDCLNT_STREAMFLAGS_EVENTCALLBACK,
                    0,
                    0,
                    mix.as_ptr(),
                    None,
                )
                .map_err(|e| os_err("Initialize(keep-alive)", &e))?;
            let event = Event::new(false)?;
            client
                .SetEventHandle(event.handle())
                .map_err(|e| os_err("SetEventHandle(keep-alive)", &e))?;
            let render: IAudioRenderClient = client
                .GetService()
                .map_err(|e| os_err("GetService(IAudioRenderClient)", &e))?;
            let buffer_frames = client
                .GetBufferSize()
                .map_err(|e| os_err("GetBufferSize(keep-alive)", &e))?;
            let ka = KeepAlive {
                client,
                render,
                event,
                buffer_frames,
            };
            ka.feed().map_err(|e| os_err("prefill(keep-alive)", &e))?;
            ka.client
                .Start()
                .map_err(|e| os_err("Start(keep-alive)", &e))?;
            Ok(ka)
        }
    }

    /// Tops the buffer up with silence. Real-time safe.
    fn feed(&self) -> windows_core::Result<()> {
        // SAFETY: GetBuffer/ReleaseBuffer pair on this thread; no data is written (SILENT flag).
        unsafe {
            let padding = self.client.GetCurrentPadding()?;
            let avail = self.buffer_frames.saturating_sub(padding);
            if avail > 0 {
                self.render.GetBuffer(avail)?;
                self.render
                    .ReleaseBuffer(avail, AUDCLNT_BUFFERFLAGS_SILENT.0 as u32)?;
            }
        }
        Ok(())
    }
}

impl Drop for KeepAlive {
    fn drop(&mut self) {
        // SAFETY: stops the stream started in `open`.
        let _ = unsafe { self.client.Stop() };
    }
}

pub(crate) struct CaptureStream {
    client: IAudioClient,
    capture: IAudioCaptureClient,
    keep_alive: Option<KeepAlive>,
    disconnect: SessionDisconnect,
    event: Event,
    format: WaveFormat,
    pick: StereoPick,
    timeline: CaptureTimeline,
    scratch: Vec<f32>,
    handler: Box<dyn CaptureHandler>,
    qpc: Qpc,
    stream_format: StreamFormat,
    // Format the client was initialized with (kept for diagnostics and symmetry with `format`).
    _mix: MixFormat,
}

impl CaptureStream {
    /// Opens loopback capture on the current default render endpoint.
    pub(crate) fn open(
        _cfg: CaptureConfig,
        handler: Box<dyn CaptureHandler>,
    ) -> Result<CaptureStream, PlatformError> {
        let en = enumerator()?;
        // SAFETY: plain COM calls on interfaces created on this thread.
        unsafe {
            let device = en
                .GetDefaultAudioEndpoint(eRender, eConsole)
                .map_err(|e| os_err("GetDefaultAudioEndpoint", &e))?;
            let client: IAudioClient = device
                .Activate(CLSCTX_ALL, None)
                .map_err(|e| os_err("Activate(IAudioClient)", &e))?;
            let mix = MixFormat::of(&client)?;
            let format = mix.format;
            let period_frames = hns_to_frames(default_period_hns(&client), format.rate).max(1);
            client
                .Initialize(
                    AUDCLNT_SHAREMODE_SHARED,
                    AUDCLNT_STREAMFLAGS_LOOPBACK | AUDCLNT_STREAMFLAGS_EVENTCALLBACK,
                    0,
                    0,
                    mix.as_ptr(),
                    None,
                )
                .map_err(|e| os_err("Initialize(loopback)", &e))?;
            let event = Event::new(false)?;
            client
                .SetEventHandle(event.handle())
                .map_err(|e| os_err("SetEventHandle(loopback)", &e))?;
            let capture: IAudioCaptureClient = client
                .GetService()
                .map_err(|e| os_err("GetService(IAudioCaptureClient)", &e))?;
            let buffer_frames = client
                .GetBufferSize()
                .map_err(|e| os_err("GetBufferSize(loopback)", &e))?;
            let disconnect = SessionDisconnect::register(&client)?;
            // Without keep-alive the synthesized silence still keeps the timeline going.
            let keep_alive = KeepAlive::open(&device).ok();
            let qpc = Qpc::new();
            let mut timeline = CaptureTimeline::new(format.rate, period_frames);
            client.Start().map_err(|e| os_err("Start(loopback)", &e))?;
            timeline.start(qpc.now_ns());
            let max_frames = MAX_BLOCK_FRAMES.max(buffer_frames as usize);
            Ok(CaptureStream {
                client,
                capture,
                keep_alive,
                disconnect,
                event,
                pick: StereoPick::from_mask(format.channels, format.channel_mask),
                format,
                timeline,
                scratch: vec![0.0; max_frames * 2],
                handler,
                qpc,
                stream_format: StreamFormat {
                    rate: format.rate as f64,
                    block_frames: period_frames,
                },
                _mix: mix,
            })
        }
    }

    fn max_frames(&self) -> usize {
        self.scratch.len() / 2
    }

    /// Delivers `frames` frames of silence starting at `host_time`, split into blocks.
    fn deliver_silence(&mut self, host_time: u64, frames: u64, discontinuity: bool) {
        let rate = self.format.rate;
        let max = self.max_frames() as u64;
        let mut done = 0u64;
        let mut first = true;
        while done < frames {
            let n = (frames - done).min(max) as usize;
            let block = &mut self.scratch[..n * 2];
            block.fill(0.0);
            let info = CaptureInfo {
                host_time: host_time.saturating_add(frames_to_ns(done, rate)),
                quality: TimestampQuality::Device,
                discontinuity: discontinuity && first,
            };
            self.handler.on_capture(block, &info);
            done += n as u64;
            first = false;
        }
    }

    /// Delivers packet frames `skip..frames` (silent if flagged) starting at `host_time`.
    fn deliver_packet(
        &mut self,
        data: *const u8,
        frames: u32,
        skip: u32,
        silent: bool,
        host_time: u64,
        discontinuity: bool,
    ) {
        let rate = self.format.rate;
        let block = self.format.block_align as usize;
        let max = self.max_frames();
        let total = frames as usize;
        let mut pos = skip as usize;
        let mut first = true;
        let bytes: &[u8] = if silent || data.is_null() {
            &[]
        } else {
            // SAFETY: WASAPI guarantees `frames * block_align` readable bytes until ReleaseBuffer.
            unsafe { std::slice::from_raw_parts(data, total * block) }
        };
        while pos < total {
            let n = (total - pos).min(max);
            let out = &mut self.scratch[..n * 2];
            if bytes.is_empty() {
                out.fill(0.0);
            } else {
                let src = bytes.get(pos * block..).unwrap_or(&[]);
                let got = decode_to_stereo(src, n, &self.format, self.pick, out);
                if got < n {
                    out[got * 2..].fill(0.0);
                }
            }
            let info = CaptureInfo {
                host_time: host_time
                    .saturating_add(frames_to_ns((pos - skip as usize) as u64, rate)),
                quality: TimestampQuality::Device,
                discontinuity: discontinuity && first,
            };
            self.handler.on_capture(out, &info);
            pos += n;
            first = false;
        }
    }

    /// Reads every pending packet. Real-time code.
    fn drain(&mut self) -> windows_core::Result<()> {
        loop {
            // SAFETY: GetNextPacketSize/GetBuffer/ReleaseBuffer on this thread, strictly paired.
            let pending = unsafe { self.capture.GetNextPacketSize()? };
            if pending == 0 {
                return Ok(());
            }
            let mut data: *mut u8 = std::ptr::null_mut();
            let mut frames = 0u32;
            let mut flags = 0u32;
            let mut devpos = 0u64;
            let mut qpc = 0u64;
            // SAFETY: valid out pointers.
            unsafe {
                self.capture.GetBuffer(
                    &mut data,
                    &mut frames,
                    &mut flags,
                    Some(&mut devpos),
                    Some(&mut qpc),
                )?
            };
            if frames == 0 {
                // AUDCLNT_S_BUFFER_EMPTY: nothing to release beyond a zero-length release.
                // SAFETY: paired with the GetBuffer above.
                unsafe { self.capture.ReleaseBuffer(0)? };
                return Ok(());
            }
            let silent = flags & AUDCLNT_BUFFERFLAGS_SILENT.0 as u32 != 0;
            let flagged = flags & AUDCLNT_BUFFERFLAGS_DATA_DISCONTINUITY.0 as u32 != 0;
            let ts_error = flags & AUDCLNT_BUFFERFLAGS_TIMESTAMP_ERROR.0 as u32 != 0;
            let qpc_ns = (!ts_error && qpc != 0).then(|| hns_to_ns(qpc));
            let now = self.qpc.now_ns();
            let fallback = now.saturating_sub(frames_to_ns(frames as u64, self.format.rate));
            let plan = self.timeline.on_packet(qpc_ns, frames, flagged, fallback);
            if plan.pad_frames > 0 {
                self.deliver_silence(plan.pad_time, plan.pad_frames, false);
            }
            self.deliver_packet(
                data,
                frames,
                plan.skip_frames,
                silent,
                plan.host_time,
                plan.discontinuity,
            );
            // SAFETY: paired with the GetBuffer above.
            unsafe { self.capture.ReleaseBuffer(frames)? };
        }
    }

    /// Synthesizes silence when no packets arrived for more than two periods.
    fn idle(&mut self) {
        let now = self.qpc.now_ns();
        if let Some((t, frames)) = self.timeline.on_idle(now) {
            self.deliver_silence(t, frames, false);
        }
    }
}

impl StreamLoop for CaptureStream {
    fn format(&self) -> StreamFormat {
        self.stream_format
    }

    fn run(&mut self, stop: &Event) -> Result<(), StreamError> {
        let ka_handle = self.keep_alive.as_ref().map(|k| k.event.handle());
        let mut handles = [
            stop.handle(),
            self.disconnect.event().handle(),
            self.event.handle(),
            self.event.handle(),
        ];
        let count = match ka_handle {
            Some(h) => {
                handles[3] = h;
                4
            }
            None => 3,
        };
        let result = loop {
            match wait_any(&handles[..count], CAPTURE_TIMEOUT_MS) {
                Wake::Signalled(0) => break Ok(()),
                Wake::Signalled(1) => break Err(self.disconnect.error()),
                Wake::Signalled(2) | Wake::Timeout => {
                    if let Err(e) = self.drain() {
                        break Err(stream_err(&e));
                    }
                }
                Wake::Signalled(_) => {
                    // A keep-alive failure is not fatal for capture: the loopback stream reports
                    // device loss itself, and silence synthesis covers the timeline.
                    if let Some(ka) = &self.keep_alive {
                        let _ = ka.feed();
                    }
                    if let Err(e) = self.drain() {
                        break Err(stream_err(&e));
                    }
                }
                Wake::Failed => break Err(StreamError::Os(-1)),
            }
            self.idle();
        };
        // SAFETY: stops the stream started in `open`.
        let _ = unsafe { self.client.Stop() };
        result
    }
}
