//! Calibration microphone: selection (never a Bluetooth headset microphone, which would switch the
//! headset to the hands-free profile) and an event-driven shared-mode capture stream delivering
//! the first channel with the packet's QPC timestamp.

use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;

use windows::Win32::Media::Audio::{
    IAudioCaptureClient, IAudioClient, IMMDeviceEnumerator, AUDCLNT_BUFFERFLAGS_DATA_DISCONTINUITY,
    AUDCLNT_BUFFERFLAGS_SILENT, AUDCLNT_BUFFERFLAGS_TIMESTAMP_ERROR, AUDCLNT_SHAREMODE_SHARED,
    AUDCLNT_STREAMFLAGS_EVENTCALLBACK,
};
use windows::Win32::System::Com::CLSCTX_ALL;
use wye_platform::{
    CaptureInfo, DeviceId, DeviceInfo, InputHandler, PlatformError, StreamError, StreamFormat,
    TimestampQuality,
};

use crate::capture::{SessionDisconnect, MAX_BLOCK_FRAMES};
use crate::classify::{hr, rank_calibration_inputs, InputCandidate};
use crate::com::{
    default_period_hns, enumerator, hns_to_frames, os_err, set_media_category, stream_err,
    wait_any, Event, MixFormat, Qpc, StreamLoop, Wake,
};
use crate::devices::{bluetooth_output_groups, capture_endpoints, device_info, get_device};
use crate::format::{decode_channel, WaveFormat};
use crate::render::init_error;
use crate::timing::{frames_to_ns, hns_to_ns};

/// Microphone permission as last observed: 0 unknown, 1 granted, 2 denied.
pub(crate) type MicPermission = Arc<AtomicU8>;
pub(crate) const MIC_UNKNOWN: u8 = 0;
pub(crate) const MIC_GRANTED: u8 = 1;
pub(crate) const MIC_DENIED: u8 = 2;

/// Capture endpoints usable for calibration, best first.
pub(crate) fn calibration_inputs(
    en: &IMMDeviceEnumerator,
) -> Result<Vec<DeviceInfo>, PlatformError> {
    let inputs = capture_endpoints(en)?;
    let bt_groups = bluetooth_output_groups(en);
    let bt_refs: Vec<&str> = bt_groups.iter().map(String::as_str).collect();
    let candidates: Vec<InputCandidate<'_>> = inputs
        .iter()
        .map(|e| InputCandidate {
            class: e.class,
            group: e.group.as_deref(),
            form_factor: e.form_factor,
            onboard: e.onboard(),
        })
        .collect();
    let order = rank_calibration_inputs(&candidates, &bt_refs);
    Ok(order
        .into_iter()
        .filter_map(|i| inputs.get(i))
        .map(|e| {
            let mut info = device_info(e);
            // An input has no system-default-output role.
            info.can_be_default = false;
            info
        })
        .collect())
}

pub(crate) struct InputStream {
    client: IAudioClient,
    capture: IAudioCaptureClient,
    disconnect: SessionDisconnect,
    event: Event,
    format: WaveFormat,
    scratch: Vec<f32>,
    handler: Box<dyn InputHandler>,
    qpc: Qpc,
    next_ns: Option<u64>,
    seen_packet: bool,
    period_ms: u32,
    stream_format: StreamFormat,
    _mix: MixFormat,
}

impl InputStream {
    pub(crate) fn open(
        id: &DeviceId,
        handler: Box<dyn InputHandler>,
        permission: &MicPermission,
    ) -> Result<InputStream, PlatformError> {
        let en = enumerator()?;
        let device = get_device(&en, id)?;
        // SAFETY: plain COM calls on interfaces created on this thread.
        unsafe {
            // The microphone privacy setting surfaces as E_ACCESSDENIED from Activate or
            // Initialize.
            let denied = |call: &'static str, e: &windows_core::Error| {
                if e.code().0 == hr::E_ACCESSDENIED {
                    permission.store(MIC_DENIED, Ordering::Release);
                    PlatformError::PermissionDenied
                } else {
                    init_error(call, id, e)
                }
            };
            let client: IAudioClient = device
                .Activate(CLSCTX_ALL, None)
                .map_err(|e| denied("Activate(IAudioClient)", &e))?;
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
                .map_err(|e| denied("Initialize(input)", &e))?;
            permission.store(MIC_GRANTED, Ordering::Release);
            let event = Event::new(false)?;
            client
                .SetEventHandle(event.handle())
                .map_err(|e| os_err("SetEventHandle(input)", &e))?;
            let capture: IAudioCaptureClient = client
                .GetService()
                .map_err(|e| os_err("GetService(IAudioCaptureClient)", &e))?;
            let buffer_frames = client
                .GetBufferSize()
                .map_err(|e| os_err("GetBufferSize(input)", &e))?;
            let disconnect = SessionDisconnect::register(&client)?;
            client
                .Start()
                .map_err(|e| init_error("Start(input)", id, &e))?;
            let period_frames = hns_to_frames(period_hns, format.rate).max(1);
            Ok(InputStream {
                client,
                capture,
                disconnect,
                event,
                format,
                scratch: vec![0.0; MAX_BLOCK_FRAMES.max(buffer_frames as usize) * 2],
                handler,
                qpc: Qpc::new(),
                next_ns: None,
                seen_packet: false,
                period_ms: ((period_hns / 10_000) as u32).max(1),
                stream_format: StreamFormat {
                    rate: format.rate as f64,
                    block_frames: period_frames,
                },
                _mix: mix,
            })
        }
    }

    /// Reads every pending packet. Real-time code.
    fn drain(&mut self) -> windows_core::Result<()> {
        let rate = self.format.rate;
        let block = self.format.block_align as usize;
        loop {
            // SAFETY: GetNextPacketSize/GetBuffer/ReleaseBuffer on this thread, strictly paired.
            let pending = unsafe { self.capture.GetNextPacketSize()? };
            if pending == 0 {
                return Ok(());
            }
            let mut data: *mut u8 = std::ptr::null_mut();
            let mut frames = 0u32;
            let mut flags = 0u32;
            let mut qpc = 0u64;
            // SAFETY: valid out pointers.
            unsafe {
                self.capture
                    .GetBuffer(&mut data, &mut frames, &mut flags, None, Some(&mut qpc))?
            };
            if frames == 0 {
                // SAFETY: paired with the GetBuffer above.
                unsafe { self.capture.ReleaseBuffer(0)? };
                return Ok(());
            }
            let silent = flags & AUDCLNT_BUFFERFLAGS_SILENT.0 as u32 != 0 || data.is_null();
            let flagged = flags & AUDCLNT_BUFFERFLAGS_DATA_DISCONTINUITY.0 as u32 != 0;
            let ts_error = flags & AUDCLNT_BUFFERFLAGS_TIMESTAMP_ERROR.0 as u32 != 0;
            let host_time = match (ts_error || qpc == 0, self.next_ns) {
                (false, _) => hns_to_ns(qpc),
                (true, Some(next)) => next,
                (true, None) => self
                    .qpc
                    .now_ns()
                    .saturating_sub(frames_to_ns(frames as u64, rate)),
            };
            // The flag is undefined on the first packet after Start.
            let discontinuity = flagged && self.seen_packet;
            self.seen_packet = true;
            let total = frames as usize;
            let bytes: &[u8] = if silent {
                &[]
            } else {
                // SAFETY: `frames * block_align` bytes are readable until ReleaseBuffer.
                unsafe { std::slice::from_raw_parts(data, total * block) }
            };
            let max = self.scratch.len();
            let mut pos = 0usize;
            while pos < total {
                let n = (total - pos).min(max);
                let out = &mut self.scratch[..n];
                if bytes.is_empty() {
                    out.fill(0.0);
                } else {
                    let src = bytes.get(pos * block..).unwrap_or(&[]);
                    let got = decode_channel(src, n, &self.format, 0, out);
                    if got < n {
                        out[got..].fill(0.0);
                    }
                }
                let info = CaptureInfo {
                    host_time: host_time.saturating_add(frames_to_ns(pos as u64, rate)),
                    quality: TimestampQuality::Device,
                    discontinuity: discontinuity && pos == 0,
                };
                self.handler.on_input(out, &info);
                pos += n;
            }
            self.next_ns = Some(host_time.saturating_add(frames_to_ns(frames as u64, rate)));
            // SAFETY: paired with the GetBuffer above.
            unsafe { self.capture.ReleaseBuffer(frames)? };
        }
    }
}

impl StreamLoop for InputStream {
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
        let result = loop {
            match wait_any(&handles, timeout) {
                Wake::Signalled(0) => break Ok(()),
                Wake::Signalled(1) => break Err(self.disconnect.error()),
                Wake::Signalled(_) | Wake::Timeout => {
                    if let Err(e) = self.drain() {
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
