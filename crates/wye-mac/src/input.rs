//! Calibration input: first channel of the first input buffer of a (non-Bluetooth) microphone.

use std::ffi::c_void;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::Duration;

use objc2_core_audio_types::{AudioBufferList, AudioTimeStamp};
use wye_platform::{
    CaptureInfo, DeviceId, InputHandler, PlatformError, Stream, StreamFormat, StreamKind,
    TimestampQuality,
};

use crate::hal::*;
use crate::output::{check_float, os, start_io, stream_listeners, DeviceIo};
use crate::rt::{rate_scalar, Continuity, Gap, MAX_FRAMES};
use crate::Shared;

/// Real-time state of the input IOProc.
pub struct InputRt {
    handler: Box<dyn InputHandler>,
    scratch: Box<[f32]>,
    tb: Timebase,
    rate: f64,
    cont: Continuity,
}

impl InputRt {
    pub fn new(handler: Box<dyn InputHandler>, rate: f64, tb: Timebase) -> InputRt {
        InputRt {
            handler,
            scratch: vec![0.0; MAX_FRAMES].into_boxed_slice(),
            tb,
            rate,
            cont: Continuity::default(),
        }
    }

    /// # Safety
    /// `input` / `in_time` must be NULL or valid for the duration of the call (HAL contract).
    pub unsafe fn process(
        &mut self,
        input: *const AudioBufferList,
        in_time: *const AudioTimeStamp,
    ) {
        if in_time.is_null() {
            return;
        }
        // SAFETY: HAL contract.
        let ts = unsafe { &*in_time };
        // SAFETY: HAL contract.
        let bufs = unsafe { buffers(input) };
        let Some(b0) = bufs.first() else { return };
        if b0.mData.is_null() {
            return;
        }
        let ch = b0.mNumberChannels.max(1) as usize;
        let frames = b0.mDataByteSize as usize / 4 / ch;
        // SAFETY: `frames * ch` f32 samples at `mData`.
        let d = unsafe { std::slice::from_raw_parts(b0.mData as *const f32, frames * ch) };
        let mut disc = matches!(
            self.cont.check(ts, frames, 0.0),
            Gap::Missing(_) | Gap::Break
        );
        let rs = rate_scalar(ts);
        let host = self.tb.ticks_to_ns(ts.mHostTime);
        let mut off = 0usize;
        while off < frames {
            let n = (frames - off).min(MAX_FRAMES);
            let out = &mut self.scratch[..n];
            for (f, o) in out.iter_mut().enumerate() {
                *o = d[(off + f) * ch];
            }
            let info = CaptureInfo {
                host_time: host.saturating_add(frames_to_ns(off as f64, self.rate, rs)),
                quality: TimestampQuality::Device,
                discontinuity: disc,
            };
            self.handler.on_input(out, &info);
            disc = false;
            off += n;
        }
    }
}

unsafe extern "C" fn input_ioproc(
    _dev: AudioObjectID,
    _now: *const AudioTimeStamp,
    input: *const AudioBufferList,
    in_time: *const AudioTimeStamp,
    _out: *mut AudioBufferList,
    _out_time: *const AudioTimeStamp,
    client: *mut c_void,
) -> OSStatus {
    if client.is_null() {
        return 0;
    }
    // SAFETY: `client` is the `InputRt` owned by the stream, alive while the IOProc exists.
    unsafe { (*(client as *mut InputRt)).process(input, in_time) };
    0
}

unsafe fn free_input(p: *mut c_void) {
    // SAFETY: `p` came from Box::into_raw::<InputRt>.
    drop(unsafe { Box::from_raw(p as *mut InputRt) });
}

/// `(Latency(input) + StreamLatency(first input stream)) / rate`.
pub fn input_latency(dev: AudioObjectID) -> Duration {
    Duration::from_secs_f64(Latency::read(dev, SCOPE_INPUT).after_timestamp_s().max(0.0))
}

/// Opens `dev` for input. The caller must have checked that it is a calibration input.
pub fn open_input(
    shared: &Arc<Shared>,
    id: &DeviceId,
    dev: AudioObjectID,
    handler: Box<dyn InputHandler>,
) -> Result<Box<dyn Stream>, PlatformError> {
    let rate: f64 = get(dev, kAudioDevicePropertyNominalSampleRate, SCOPE_GLOBAL).unwrap_or(0.0);
    if rate <= 0.0 {
        return Err(os("nominal rate", 0));
    }
    check_float(dev, SCOPE_INPUT)?;
    let block: u32 = get(dev, kAudioDevicePropertyBufferFrameSize, SCOPE_GLOBAL).unwrap_or(0);
    let rt = Box::into_raw(Box::new(InputRt::new(handler, rate, shared.tb)));
    let active = Arc::new(AtomicBool::new(true));
    let mut io = DeviceIo {
        active: active.clone(),
        listeners: Vec::new(),
        dev,
        proc_id: std::ptr::null_mut(),
        started: false,
        rt: rt as *mut c_void,
        free_rt: free_input,
        format: StreamFormat {
            rate,
            block_frames: block,
        },
    };
    start_io(&mut io, input_ioproc)?;
    io.listeners = stream_listeners(
        shared,
        &active,
        dev,
        StreamKind::Input(id.clone()),
        SCOPE_INPUT,
        None,
    );
    Ok(Box::new(io))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rt::test_util::{ts, Abl4};
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Mutex;

    type Calls = Arc<Mutex<Vec<(Vec<f32>, u64, bool)>>>;
    struct Rec(Calls);
    impl InputHandler for Rec {
        fn on_input(&mut self, mono: &[f32], info: &CaptureInfo) {
            self.0
                .lock()
                .unwrap()
                .push((mono.to_vec(), info.host_time, info.discontinuity));
        }
    }

    struct Sum(Arc<AtomicU64>);
    impl InputHandler for Sum {
        fn on_input(&mut self, mono: &[f32], _info: &CaptureInfo) {
            self.0.fetch_add(mono.len() as u64, Ordering::Relaxed);
        }
    }

    #[test]
    fn takes_first_channel_of_first_buffer() {
        let log = Arc::new(Mutex::new(Vec::new()));
        let mut rt = InputRt::new(Box::new(Rec(log.clone())), 48_000.0, Timebase::identity());
        let mut d: Vec<f32> = (0..12).map(|i| i as f32).collect();
        let mut other = vec![7.0f32; 4];
        let mut abl = Abl4::new(&mut [(3, &mut d), (1, &mut other)]);
        unsafe { rt.process(abl.ptr(), &ts(0.0, 42)) };
        unsafe { rt.process(abl.ptr(), &ts(10.0, 43)) };
        let g = log.lock().unwrap();
        assert_eq!(g[0], (vec![0.0, 3.0, 6.0, 9.0], 42, false));
        assert!(g[1].2);
    }

    #[test]
    fn input_ioproc_does_not_allocate() {
        let n = Arc::new(AtomicU64::new(0));
        let rt = Box::into_raw(Box::new(InputRt::new(
            Box::new(Sum(n.clone())),
            48_000.0,
            Timebase::new(),
        )));
        let mut d = vec![0.0f32; 5000 * 2];
        let mut abl = Abl4::new(&mut [(2, &mut d)]);
        let t = ts(0.0, 1);
        crate::assert_no_alloc(|| unsafe {
            input_ioproc(
                0,
                &t,
                abl.ptr(),
                &t,
                std::ptr::null_mut(),
                std::ptr::null(),
                rt.cast(),
            );
        });
        unsafe { drop(Box::from_raw(rt)) };
        assert_eq!(n.load(Ordering::Relaxed), 5000);
    }
}
