//! System-mix capture: a global stereo Process Tap (excluding this process) inside a private,
//! tap-only aggregate device, read by a C IOProc.

use std::ffi::c_void;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use objc2::rc::Retained;
use objc2::runtime::AnyObject;
use objc2::{AnyThread, Message};
use objc2_core_audio::{AudioHardwareCreateProcessTap, CATapDescription, CATapMuteBehavior};
use objc2_core_audio_types::{AudioBufferList, AudioStreamBasicDescription, AudioTimeStamp};
use objc2_foundation::{NSArray, NSDictionary, NSNumber, NSString};
use wye_platform::{
    CaptureConfig, CaptureHandler, CaptureInfo, PlatformError, PlatformEvent, Stream, StreamError,
    StreamFormat, StreamKind, TimestampQuality,
};

use crate::hal::*;
use crate::rt::{rate_scalar, Continuity, Gap, MAX_FRAMES};
use crate::Shared;

/// How long `open_capture` waits after IO started before the second permission probe.
pub const PROBE_WAIT: Duration = Duration::from_secs(2);

/// Outcome of the permission probe of the last `open_capture` call (diagnostics).
#[derive(Clone, Debug, PartialEq)]
pub struct CaptureProbe {
    /// Process object of this process that was excluded from the tap (0 = none excluded).
    pub own_process_object: u32,
    /// Status of re-writing the tap description right after creating the tap.
    pub st1: i32,
    /// Same, after IO has been running for two seconds.
    pub st2: i32,
    /// Both probes reported a refusal → `PermissionDenied`.
    pub denied: bool,
    pub tap_uid: String,
    /// Nominal rate of the aggregate device (= callback rate).
    pub rate: f64,
    /// Buffer frame size of the aggregate device after setting the preferred size.
    pub block_frames: u32,
    /// Time spent in `open_capture` (including the probe wait).
    pub open_time: Duration,
}

pub fn is_refusal(st: OSStatus) -> bool {
    st == STATUS_PERMISSION || st == STATUS_WHAT
}

/// Real-time state of the capture IOProc.
pub struct CaptureRt {
    handler: Box<dyn CaptureHandler>,
    scratch: Box<[f32]>,
    tb: Timebase,
    rate: f64,
    cont: Continuity,
}

impl CaptureRt {
    pub fn new(handler: Box<dyn CaptureHandler>, rate: f64, tb: Timebase) -> CaptureRt {
        CaptureRt {
            handler,
            scratch: vec![0.0; MAX_FRAMES * 2].into_boxed_slice(),
            tb,
            rate,
            cont: Continuity::default(),
        }
    }

    /// Converts the tap buffers to interleaved stereo and calls the handler (in blocks of at most
    /// `MAX_FRAMES`). Small sample-time gaps are filled with silence first; larger jumps are
    /// reported as a discontinuity.
    ///
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
        // SAFETY: non-null, provided by the HAL for this call.
        let ts = unsafe { &*in_time };
        // SAFETY: HAL contract.
        let bufs = unsafe { buffers(input) };
        let Some(b0) = bufs.first() else { return };
        if b0.mData.is_null() {
            return;
        }
        let ch0 = b0.mNumberChannels.max(1) as usize;
        // Non-interleaved stereo: two mono buffers.
        let planar = ch0 == 1 && bufs.len() >= 2 && !bufs[1].mData.is_null();
        let frames = if planar {
            (b0.mDataByteSize.min(bufs[1].mDataByteSize) / 4) as usize
        } else {
            b0.mDataByteSize as usize / 4 / ch0
        };
        // SAFETY: the buffers hold at least `frames` frames of f32 samples (sizes from the HAL).
        let left = unsafe {
            std::slice::from_raw_parts(
                b0.mData as *const f32,
                if planar { frames } else { frames * ch0 },
            )
        };
        let right: &[f32] = if planar {
            // SAFETY: as above.
            unsafe { std::slice::from_raw_parts(bufs[1].mData as *const f32, frames) }
        } else {
            &[]
        };

        let rs = rate_scalar(ts);
        let host = self.tb.ticks_to_ns(ts.mHostTime);
        let mut discontinuity = false;
        match self.cont.check(ts, frames, self.rate * 0.5) {
            Gap::None => {}
            Gap::Break => discontinuity = true,
            Gap::Missing(n) => self.silence(n as usize, host, rs),
        }

        let mut off = 0usize;
        while off < frames {
            let n = (frames - off).min(MAX_FRAMES);
            let out = &mut self.scratch[..n * 2];
            for f in 0..n {
                let i = off + f;
                let (l, r) = if planar {
                    (left[i], right[i])
                } else if ch0 >= 2 {
                    (left[i * ch0], left[i * ch0 + 1])
                } else {
                    (left[i], left[i])
                };
                out[f * 2] = l;
                out[f * 2 + 1] = r;
            }
            let info = CaptureInfo {
                host_time: host.saturating_add(frames_to_ns(off as f64, self.rate, rs)),
                quality: TimestampQuality::Device,
                discontinuity,
            };
            self.handler.on_capture(out, &info);
            discontinuity = false;
            off += n;
        }
    }

    /// Delivers `gap` frames of silence that end exactly at `host_end`.
    fn silence(&mut self, gap: usize, host_end: u64, rs: f64) {
        let start = host_end.saturating_sub(frames_to_ns(gap as f64, self.rate, rs));
        let mut off = 0usize;
        while off < gap {
            let n = (gap - off).min(MAX_FRAMES);
            let out = &mut self.scratch[..n * 2];
            out.fill(0.0);
            let info = CaptureInfo {
                host_time: start.saturating_add(frames_to_ns(off as f64, self.rate, rs)),
                quality: TimestampQuality::Device,
                discontinuity: false,
            };
            self.handler.on_capture(out, &info);
            off += n;
        }
    }
}

unsafe extern "C" fn capture_ioproc(
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
    // SAFETY: `client` is the `CaptureRt` owned by the stream; it outlives the IOProc and is only
    // touched from this IO thread while the IOProc is registered.
    unsafe { (*(client as *mut CaptureRt)).process(input, in_time) };
    0
}

fn obj<T: Message>(r: &Retained<T>) -> &AnyObject {
    // SAFETY: every Objective-C object is an AnyObject.
    unsafe { &*(Retained::as_ptr(r) as *const AnyObject) }
}

fn sub_tap_dict(tap_uid: &str) -> Retained<NSDictionary<NSString, AnyObject>> {
    let k_uid = NSString::from_str("uid");
    let k_drift = NSString::from_str("drift");
    let v_uid = NSString::from_str(tap_uid);
    let v_drift = NSNumber::new_u32(0);
    NSDictionary::from_slices(&[&*k_uid, &*k_drift], &[obj(&v_uid), obj(&v_drift)])
}

fn aggregate_dict(tap_uid: &str) -> Retained<NSDictionary<NSString, AnyObject>> {
    let keys =
        ["name", "uid", "private", "stacked", "tapautostart", "taps"].map(NSString::from_str);
    let name = NSString::from_str("Wye Capture");
    let uid = NSString::from_str(&format!("io.github.xsrqmiracle.wye.capture.{tap_uid}"));
    let one = NSNumber::new_u32(1);
    let zero = NSNumber::new_u32(0);
    let sub = sub_tap_dict(tap_uid);
    let taps: Retained<NSArray<AnyObject>> = NSArray::from_slice(&[obj(&sub)]);
    let key_refs: Vec<&NSString> = keys.iter().map(|k| &**k).collect();
    NSDictionary::from_slices(
        &key_refs,
        &[
            obj(&name),
            obj(&uid),
            obj(&one),
            obj(&zero),
            obj(&zero),
            obj(&taps),
        ],
    )
}

fn probe(tap: AudioObjectID, desc: &Retained<CATapDescription>) -> OSStatus {
    let p: *const CATapDescription = Retained::as_ptr(desc);
    match set(tap, kAudioTapPropertyDescription, SCOPE_GLOBAL, &p) {
        Ok(()) => 0,
        Err(st) => st,
    }
}

/// Running capture. Drop order: listeners → Stop → DestroyIOProcID → DestroyAggregateDevice →
/// DestroyProcessTap.
pub struct CaptureStream {
    active: Arc<AtomicBool>,
    listeners: Vec<Listener>,
    tap: AudioObjectID,
    agg: AudioObjectID,
    proc_id: AudioDeviceIOProcID,
    started: bool,
    rt: *mut CaptureRt,
    _desc: Retained<CATapDescription>,
    format: StreamFormat,
}

// SAFETY: the raw pointers are owned exclusively by this object and only dereferenced by the
// HAL IO thread between IOProc creation and destruction; the tap description is immutable here.
unsafe impl Send for CaptureStream {}

impl Stream for CaptureStream {
    fn format(&self) -> StreamFormat {
        self.format
    }
}

impl Drop for CaptureStream {
    fn drop(&mut self) {
        self.active.store(false, Ordering::SeqCst);
        self.listeners.clear();
        // SAFETY: ids created by this object; each is destroyed exactly once, in order.
        unsafe {
            if !self.proc_id.is_null() {
                if self.started {
                    AudioDeviceStop(self.agg, self.proc_id);
                }
                AudioDeviceDestroyIOProcID(self.agg, self.proc_id);
                self.proc_id = std::ptr::null_mut();
            }
            if !self.rt.is_null() {
                drop(Box::from_raw(self.rt));
                self.rt = std::ptr::null_mut();
            }
            if self.agg != 0 {
                AudioHardwareDestroyAggregateDevice(self.agg);
            }
            if self.tap != 0 {
                AudioHardwareDestroyProcessTap(self.tap);
            }
        }
    }
}

fn os(call: &'static str, st: OSStatus) -> PlatformError {
    PlatformError::Os {
        call,
        code: st as i64,
    }
}

fn input_format(agg: AudioObjectID) -> Option<AudioStreamBasicDescription> {
    let streams: Vec<AudioObjectID> =
        get_vec(agg, kAudioDevicePropertyStreams, SCOPE_INPUT).unwrap_or_default();
    streams
        .first()
        .and_then(|&s| get(s, kAudioStreamPropertyVirtualFormat, SCOPE_GLOBAL).ok())
}

pub fn is_f32(f: &AudioStreamBasicDescription) -> bool {
    f.mFormatID == kAudioFormatLinearPCM
        && f.mFormatFlags & kAudioFormatFlagIsFloat != 0
        && f.mBitsPerChannel == 32
}

pub fn open_capture(
    shared: &Arc<Shared>,
    cfg: CaptureConfig,
    handler: Box<dyn CaptureHandler>,
) -> Result<(Box<dyn Stream>, CaptureProbe), PlatformError> {
    let t0 = Instant::now();
    // 1. Own process object; an unknown (0) id must not be put in the exclusion list.
    let me = own_process_object();
    let excl: Retained<NSArray<NSNumber>> = if me != kAudioObjectUnknown {
        NSArray::from_retained_slice(&[NSNumber::new_u32(me)])
    } else {
        NSArray::new()
    };
    // 2. Global stereo tap excluding this process, private, mute behaviour per mode.
    // SAFETY: CATapDescription exists (checked in MacPlatform::new); arguments are valid objects.
    let desc = unsafe {
        let d = CATapDescription::initStereoGlobalTapButExcludeProcesses(
            CATapDescription::alloc(),
            &excl,
        );
        d.setName(&NSString::from_str("Wye"));
        d.setPrivate(true);
        d.setMuteBehavior(if cfg.mute_original {
            CATapMuteBehavior::MutedWhenTapped
        } else {
            CATapMuteBehavior::Unmuted
        });
        d
    };
    // SAFETY: valid description object.
    let tap_uid = unsafe { desc.UUID().UUIDString() }.to_string();
    // 3. Create the tap.
    let mut tap: AudioObjectID = 0;
    // SAFETY: valid description and out pointer.
    let st = unsafe { AudioHardwareCreateProcessTap(Some(&desc), &mut tap) };
    if st != 0 || tap == 0 {
        return Err(os("AudioHardwareCreateProcessTap", st));
    }
    let mut stream = CaptureStream {
        active: Arc::new(AtomicBool::new(true)),
        listeners: Vec::new(),
        tap,
        agg: 0,
        proc_id: std::ptr::null_mut(),
        started: false,
        rt: std::ptr::null_mut(),
        _desc: desc.clone(),
        format: StreamFormat {
            rate: 0.0,
            block_frames: 0,
        },
    };
    // 4. First permission probe (recorded only; an undetermined state must still reach the
    // point where the system prompt appears).
    let st1 = probe(tap, &desc);
    // 5. Tap-only private aggregate device (no main device, no sub-devices).
    let dict = aggregate_dict(&tap_uid);
    let mut agg: AudioObjectID = 0;
    // SAFETY: NSDictionary is toll-free bridged to CFDictionary.
    let st = unsafe {
        AudioHardwareCreateAggregateDevice(Retained::as_ptr(&dict) as *const c_void, &mut agg)
    };
    if st != 0 || agg == 0 {
        return Err(os("AudioHardwareCreateAggregateDevice", st));
    }
    stream.agg = agg;
    // 6. Buffer size, rate and format.
    let _ = set(
        agg,
        kAudioDevicePropertyBufferFrameSize,
        SCOPE_GLOBAL,
        &cfg.preferred_block_frames,
    );
    let rate: f64 = get(agg, kAudioDevicePropertyNominalSampleRate, SCOPE_GLOBAL).unwrap_or(0.0);
    if rate <= 0.0 {
        return Err(PlatformError::Os {
            call: "aggregate nominal rate",
            code: 0,
        });
    }
    if let Some(f) = input_format(agg) {
        if !is_f32(&f) {
            return Err(PlatformError::Unsupported("non-float32 capture format"));
        }
    }
    let block: u32 = get(agg, kAudioDevicePropertyBufferFrameSize, SCOPE_GLOBAL)
        .unwrap_or(cfg.preferred_block_frames);
    stream.format = StreamFormat {
        rate,
        block_frames: block,
    };
    // 7. IOProc + start.
    let rt = Box::into_raw(Box::new(CaptureRt::new(handler, rate, shared.tb)));
    stream.rt = rt;
    let mut pid: AudioDeviceIOProcID = std::ptr::null_mut();
    // SAFETY: valid device, IOProc and client pointer (owned by `stream`).
    let st = unsafe { AudioDeviceCreateIOProcID(agg, capture_ioproc, rt as *mut c_void, &mut pid) };
    if st != 0 || pid.is_null() {
        return Err(os("AudioDeviceCreateIOProcID", st));
    }
    stream.proc_id = pid;
    // SAFETY: valid device and IOProc id.
    let st = unsafe { AudioDeviceStart(agg, pid) };
    if st != 0 {
        return Err(os("AudioDeviceStart", st));
    }
    stream.started = true;
    // 8. Listeners: aggregate alive / rate, tap format, service restart.
    stream.listeners = capture_listeners(shared, &stream.active, agg, tap);
    // 9. Second probe after IO has run for a while (prompt shown or answered by then).
    std::thread::sleep(PROBE_WAIT);
    let st2 = probe(tap, &desc);
    let denied = is_refusal(st1) && is_refusal(st2);
    let probe = CaptureProbe {
        own_process_object: me,
        st1,
        st2,
        denied,
        tap_uid,
        rate,
        block_frames: block,
        open_time: t0.elapsed(),
    };
    if denied {
        drop(stream);
        return Err(PlatformError::PermissionDenied);
    }
    Ok((Box::new(stream), probe))
}

fn capture_listeners(
    shared: &Arc<Shared>,
    active: &Arc<AtomicBool>,
    agg: AudioObjectID,
    tap: AudioObjectID,
) -> Vec<Listener> {
    let mut v = Vec::new();
    let targets = [
        (agg, kAudioDevicePropertyDeviceIsAlive),
        (agg, kAudioDevicePropertyNominalSampleRate),
        (tap, kAudioTapPropertyFormat),
        (
            kAudioObjectSystemObject,
            kAudioHardwarePropertyServiceRestarted,
        ),
    ];
    for (o, sel) in targets {
        let sh = shared.clone();
        let act = active.clone();
        let cb = move |s: u32| {
            if !act.load(Ordering::SeqCst) {
                return;
            }
            let error = match s {
                kAudioDevicePropertyDeviceIsAlive => {
                    let alive: u32 =
                        get(agg, kAudioDevicePropertyDeviceIsAlive, SCOPE_GLOBAL).unwrap_or(0);
                    if alive != 0 {
                        return;
                    }
                    StreamError::DeviceGone
                }
                kAudioHardwarePropertyServiceRestarted => StreamError::Os(s as i64),
                _ => StreamError::FormatChanged,
            };
            sh.emit(PlatformEvent::StreamFailed {
                stream: StreamKind::Capture,
                error,
            });
        };
        if let Ok(l) = Listener::add(&shared.queue, o, addr(sel, SCOPE_GLOBAL), cb) {
            v.push(l);
        }
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rt::test_util::{ts, Abl4};
    use std::sync::Mutex;

    #[derive(Default)]
    struct Log {
        calls: Vec<(usize, u64, bool, f32, f32)>,
    }

    struct Rec(Arc<Mutex<Log>>);
    impl CaptureHandler for Rec {
        fn on_capture(&mut self, stereo: &[f32], info: &CaptureInfo) {
            let mut g = self.0.lock().unwrap();
            let (l, r) = (
                stereo.first().copied().unwrap_or(9.0),
                stereo.get(1).copied().unwrap_or(9.0),
            );
            g.calls
                .push((stereo.len() / 2, info.host_time, info.discontinuity, l, r));
        }
    }

    /// Counts calls without allocating (for assert_no_alloc).
    struct Count(Arc<std::sync::atomic::AtomicU64>);
    impl CaptureHandler for Count {
        fn on_capture(&mut self, stereo: &[f32], _info: &CaptureInfo) {
            self.0.fetch_add(stereo.len() as u64, Ordering::Relaxed);
        }
    }

    fn rt_with_log(rate: f64) -> (CaptureRt, Arc<Mutex<Log>>) {
        let log = Arc::new(Mutex::new(Log::default()));
        (
            CaptureRt::new(Box::new(Rec(log.clone())), rate, Timebase::identity()),
            log,
        )
    }

    #[test]
    fn interleaved_stereo_passes_through() {
        let (mut rt, log) = rt_with_log(48_000.0);
        let mut d: Vec<f32> = (0..512).map(|i| i as f32).collect();
        let mut abl = Abl4::new(&mut [(2, &mut d)]);
        let t = ts(0.0, 1000);
        unsafe { rt.process(abl.ptr(), &t) };
        let g = log.lock().unwrap();
        assert_eq!(g.calls.len(), 1);
        assert_eq!(g.calls[0], (256, 1000, false, 0.0, 1.0));
    }

    #[test]
    fn mono_is_duplicated_and_planar_is_interleaved() {
        let (mut rt, log) = rt_with_log(48_000.0);
        let mut m = vec![0.5f32; 64];
        let mut abl = Abl4::new(&mut [(1, &mut m)]);
        unsafe { rt.process(abl.ptr(), &ts(0.0, 0)) };
        let mut l = vec![0.25f32; 64];
        let mut r = vec![-0.25f32; 64];
        let mut abl = Abl4::new(&mut [(1, &mut l), (1, &mut r)]);
        unsafe { rt.process(abl.ptr(), &ts(64.0, 0)) };
        let mut four: Vec<f32> = (0..256).map(|i| (i % 4) as f32).collect();
        let mut abl = Abl4::new(&mut [(4, &mut four)]);
        unsafe { rt.process(abl.ptr(), &ts(128.0, 0)) };
        let g = log.lock().unwrap();
        assert_eq!(g.calls[0].0, 64);
        assert_eq!((g.calls[0].3, g.calls[0].4), (0.5, 0.5));
        assert_eq!((g.calls[1].3, g.calls[1].4), (0.25, -0.25));
        assert_eq!((g.calls[2].0, g.calls[2].3, g.calls[2].4), (64, 0.0, 1.0));
    }

    #[test]
    fn small_gap_is_filled_with_silence_large_gap_is_discontinuity() {
        let rate = 48_000.0;
        let (mut rt, log) = rt_with_log(rate);
        let mut d = vec![1.0f32; 512];
        let mut abl = Abl4::new(&mut [(2, &mut d)]);
        let ns = |f: f64| (f / rate * 1e9) as u64;
        unsafe { rt.process(abl.ptr(), &ts(0.0, 0)) };
        unsafe { rt.process(abl.ptr(), &ts(256.0, ns(256.0))) };
        // 100-frame gap.
        unsafe { rt.process(abl.ptr(), &ts(612.0, ns(612.0))) };
        // Jump of 1 s (> 0.5 s) → discontinuity.
        unsafe { rt.process(abl.ptr(), &ts(868.0 + rate, ns(868.0 + rate))) };
        // Backwards → discontinuity.
        unsafe { rt.process(abl.ptr(), &ts(10.0, 5)) };
        let g = log.lock().unwrap();
        assert_eq!(g.calls.len(), 6);
        assert!(!g.calls[1].2);
        // silence call: 100 frames, zeros, ends at the next block's host time
        assert_eq!(g.calls[2].0, 100);
        assert_eq!((g.calls[2].3, g.calls[2].4), (0.0, 0.0));
        assert_eq!(g.calls[2].1, ns(612.0) - ns(100.0));
        assert!(!g.calls[2].2);
        assert!(!g.calls[3].2);
        assert_eq!(g.calls[3].0, 256);
        assert!(g.calls[4].2);
        assert!(g.calls[5].2);
    }

    #[test]
    fn long_buffers_are_split() {
        let rate = 48_000.0;
        let (mut rt, log) = rt_with_log(rate);
        let mut d = vec![0.0f32; 10_000 * 2];
        let mut abl = Abl4::new(&mut [(2, &mut d)]);
        unsafe { rt.process(abl.ptr(), &ts(0.0, 1_000_000)) };
        let g = log.lock().unwrap();
        let sizes: Vec<usize> = g.calls.iter().map(|c| c.0).collect();
        assert_eq!(sizes, vec![4096, 4096, 1808]);
        assert_eq!(g.calls[1].1, 1_000_000 + (4096.0 / rate * 1e9) as u64);
    }

    #[test]
    fn null_inputs_are_ignored() {
        let (mut rt, log) = rt_with_log(48_000.0);
        unsafe { rt.process(std::ptr::null(), &ts(0.0, 0)) };
        let mut d = vec![0.0f32; 8];
        let mut abl = Abl4::new(&mut [(2, &mut d)]);
        unsafe { rt.process(abl.ptr(), std::ptr::null()) };
        abl.bufs[0].mData = std::ptr::null_mut();
        unsafe { rt.process(abl.ptr(), &ts(0.0, 0)) };
        assert!(log.lock().unwrap().calls.is_empty());
    }

    #[test]
    fn capture_ioproc_does_not_allocate() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let rt = Box::into_raw(Box::new(CaptureRt::new(
            Box::new(Count(count.clone())),
            48_000.0,
            Timebase::new(),
        )));
        let mut d = vec![0.1f32; 9000 * 2];
        let mut abl = Abl4::new(&mut [(2, &mut d)]);
        let t1 = ts(0.0, 10);
        let t2 = ts(9100.0, 20); // 100-frame gap → silence path
        let t3 = ts(1e9, 30); // break
        crate::assert_no_alloc(|| unsafe {
            capture_ioproc(
                0,
                &t1,
                abl.ptr(),
                &t1,
                std::ptr::null_mut(),
                std::ptr::null(),
                rt.cast(),
            );
            capture_ioproc(
                0,
                &t2,
                abl.ptr(),
                &t2,
                std::ptr::null_mut(),
                std::ptr::null(),
                rt.cast(),
            );
            capture_ioproc(
                0,
                &t3,
                abl.ptr(),
                &t3,
                std::ptr::null_mut(),
                std::ptr::null(),
                rt.cast(),
            );
        });
        unsafe { drop(Box::from_raw(rt)) };
        assert_eq!(count.load(Ordering::Relaxed), (9000 * 3 + 100) * 2);
    }

    #[test]
    fn refusal_codes() {
        assert!(is_refusal(fcc(b"!hog") as i32));
        assert!(is_refusal(0x7761743F));
        assert!(!is_refusal(0));
        assert!(!is_refusal(fcc(b"!obj") as i32));
    }
}
