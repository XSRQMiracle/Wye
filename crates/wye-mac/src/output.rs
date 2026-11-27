//! Mirror output: a raw HAL IOProc per device. Device properties are only read, never written.

use std::ffi::c_void;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use objc2_core_audio_types::{AudioBufferList, AudioStreamBasicDescription, AudioTimeStamp};
use wye_platform::{
    DeviceId, OutputConfig, PlatformError, PlatformEvent, RenderHandler, RenderInfo, Stream,
    StreamError, StreamFormat, StreamKind, TimestampQuality,
};

use crate::devices::{hogged_by_other, preferred_stereo};
use crate::hal::*;
use crate::rt::{rate_scalar, Continuity, Gap, MAX_FRAMES};
use crate::tap::is_f32;
use crate::Shared;

/// Where one logical channel is written: buffer index and channel offset inside that buffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Slot {
    pub buffer: usize,
    pub offset: usize,
}

/// Mapping of the interleaved L/R render buffer onto the device's buffers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChannelMap {
    Stereo {
        left: Slot,
        right: Slot,
    },
    /// Single-channel device: (L + R) / 2.
    Mono(Slot),
}

/// Builds the map from the device layout.
///
/// `buffers` = channel count of each IOProc buffer (output stream configuration), `starting` =
/// starting channel (1-based) of each output stream when there is one stream per buffer, and
/// `stereo` = preferred stereo channels (1-based).
pub fn channel_map(
    buffers: &[u32],
    starting: Option<&[u32]>,
    stereo: (u32, u32),
) -> Option<ChannelMap> {
    let total: u32 = buffers.iter().sum();
    if total == 0 {
        return None;
    }
    // First device channel of each buffer: from StartingChannel when it is consistent, else the
    // running sum of the buffer channel counts.
    let mut first: Vec<u32> = Vec::with_capacity(buffers.len());
    let mut acc = 1u32;
    for &n in buffers {
        first.push(acc);
        acc += n;
    }
    if let Some(s) = starting {
        if s.len() == buffers.len() && s.iter().all(|&c| c >= 1) {
            first = s.to_vec();
        }
    }
    let locate = |ch: u32| -> Option<Slot> {
        buffers
            .iter()
            .zip(&first)
            .enumerate()
            .find(|(_, (&n, &f))| ch >= f && ch < f + n)
            .map(|(i, (_, &f))| Slot {
                buffer: i,
                offset: (ch - f) as usize,
            })
    };
    if total == 1 {
        return locate(first[0])
            .or(Some(Slot {
                buffer: 0,
                offset: 0,
            }))
            .map(ChannelMap::Mono);
    }
    let (l, r) = match (locate(stereo.0), locate(stereo.1)) {
        (Some(l), Some(r)) => (l, r),
        _ => {
            // Preferred channels not present: use the first two device channels.
            let mut it = buffers.iter().enumerate().flat_map(|(i, &n)| {
                (0..n as usize).map(move |o| Slot {
                    buffer: i,
                    offset: o,
                })
            });
            match (it.next(), it.next()) {
                (Some(l), Some(r)) => (l, r),
                (Some(l), None) => return Some(ChannelMap::Mono(l)),
                _ => return None,
            }
        }
    };
    Some(ChannelMap::Stereo { left: l, right: r })
}

/// Real-time state of the render IOProc.
pub struct RenderRt {
    handler: Box<dyn RenderHandler>,
    scratch: Box<[f32]>,
    map: ChannelMap,
    tb: Timebase,
    rate: f64,
    after_ns: Arc<AtomicU64>,
    cont: Continuity,
}

impl RenderRt {
    pub fn new(
        handler: Box<dyn RenderHandler>,
        map: ChannelMap,
        rate: f64,
        after_ns: Arc<AtomicU64>,
        tb: Timebase,
    ) -> RenderRt {
        RenderRt {
            handler,
            scratch: vec![0.0; MAX_FRAMES * 2].into_boxed_slice(),
            map,
            tb,
            rate,
            after_ns,
            cont: Continuity::default(),
        }
    }

    /// # Safety
    /// `out` / `out_time` must be NULL or valid for the duration of the call (HAL contract).
    pub unsafe fn process(&mut self, out: *mut AudioBufferList, out_time: *const AudioTimeStamp) {
        if out_time.is_null() || out.is_null() {
            return;
        }
        // SAFETY: HAL contract.
        let ts = unsafe { &*out_time };
        // SAFETY: HAL contract; buffers are writable for this call.
        let bufs = unsafe { buffers(out) };
        // Pre-zero every device buffer (channels Wye does not use stay silent).
        let mut frames = usize::MAX;
        for b in bufs {
            if b.mData.is_null() {
                continue;
            }
            // SAFETY: `mDataByteSize` writable bytes at `mData`.
            unsafe { std::ptr::write_bytes(b.mData as *mut u8, 0, b.mDataByteSize as usize) };
            let ch = b.mNumberChannels.max(1) as usize;
            frames = frames.min(b.mDataByteSize as usize / 4 / ch);
        }
        if frames == usize::MAX || frames == 0 {
            return;
        }
        let discontinuity = matches!(
            self.cont.check(ts, frames, 0.0),
            Gap::Missing(_) | Gap::Break
        );
        let rs = rate_scalar(ts);
        let host = self.tb.ticks_to_ns(ts.mHostTime);
        let after = Duration::from_nanos(self.after_ns.load(Ordering::Relaxed));

        let target = |s: Slot| -> Option<(*mut f32, usize)> {
            let b = bufs.get(s.buffer)?;
            let ch = b.mNumberChannels.max(1) as usize;
            if b.mData.is_null() || s.offset >= ch {
                return None;
            }
            Some((b.mData as *mut f32, ch))
        };
        let (l_t, r_t, mono) = match self.map {
            ChannelMap::Stereo { left, right } => (
                target(left).map(|t| (t, left.offset)),
                target(right).map(|t| (t, right.offset)),
                false,
            ),
            ChannelMap::Mono(s) => (target(s).map(|t| (t, s.offset)), None, true),
        };

        let mut off = 0usize;
        let mut disc = discontinuity;
        while off < frames {
            let n = (frames - off).min(MAX_FRAMES);
            let s = &mut self.scratch[..n * 2];
            s.fill(0.0);
            let info = RenderInfo {
                host_time: host.saturating_add(frames_to_ns(off as f64, self.rate, rs)),
                quality: TimestampQuality::Device,
                discontinuity: disc,
                after_timestamp: after,
            };
            self.handler.on_render(s, &info);
            disc = false;
            for f in 0..n {
                let (l, r) = (s[f * 2], s[f * 2 + 1]);
                let i = off + f;
                if mono {
                    if let Some(((p, ch), o)) = l_t {
                        // SAFETY: i < frames ≤ buffer frames and o < ch.
                        unsafe { *p.add(i * ch + o) = (l + r) * 0.5 };
                    }
                } else {
                    if let Some(((p, ch), o)) = l_t {
                        // SAFETY: as above.
                        unsafe { *p.add(i * ch + o) = l };
                    }
                    if let Some(((p, ch), o)) = r_t {
                        // SAFETY: as above.
                        unsafe { *p.add(i * ch + o) = r };
                    }
                }
            }
            off += n;
        }
    }
}

unsafe extern "C" fn render_ioproc(
    _dev: AudioObjectID,
    _now: *const AudioTimeStamp,
    _input: *const AudioBufferList,
    _in_time: *const AudioTimeStamp,
    out: *mut AudioBufferList,
    out_time: *const AudioTimeStamp,
    client: *mut c_void,
) -> OSStatus {
    if client.is_null() {
        return 0;
    }
    // SAFETY: `client` is the `RenderRt` owned by the stream, alive while the IOProc exists.
    unsafe { (*(client as *mut RenderRt)).process(out, out_time) };
    0
}

/// Reads `after_timestamp` of the output direction in nanoseconds.
pub fn after_timestamp_ns(dev: AudioObjectID) -> u64 {
    (Latency::read(dev, SCOPE_OUTPUT).after_timestamp_s() * 1e9).max(0.0) as u64
}

/// Shared IOProc teardown for output and input streams.
pub struct DeviceIo {
    pub active: Arc<AtomicBool>,
    pub listeners: Vec<Listener>,
    pub dev: AudioObjectID,
    pub proc_id: AudioDeviceIOProcID,
    pub started: bool,
    /// Owned real-time state and its destructor.
    pub rt: *mut c_void,
    pub free_rt: unsafe fn(*mut c_void),
    pub format: StreamFormat,
}

// SAFETY: raw pointers are owned exclusively; the HAL IO thread uses `rt` only while the IOProc
// is registered, and it is freed after DestroyIOProcID.
unsafe impl Send for DeviceIo {}

impl Stream for DeviceIo {
    fn format(&self) -> StreamFormat {
        self.format
    }
}

impl Drop for DeviceIo {
    fn drop(&mut self) {
        self.active.store(false, Ordering::SeqCst);
        self.listeners.clear();
        // SAFETY: ids owned by this object, destroyed once.
        unsafe {
            if !self.proc_id.is_null() {
                if self.started {
                    AudioDeviceStop(self.dev, self.proc_id);
                }
                AudioDeviceDestroyIOProcID(self.dev, self.proc_id);
            }
            if !self.rt.is_null() {
                (self.free_rt)(self.rt);
            }
        }
    }
}

unsafe fn free_render(p: *mut c_void) {
    // SAFETY: `p` came from Box::into_raw::<RenderRt>.
    drop(unsafe { Box::from_raw(p as *mut RenderRt) });
}

pub fn os(call: &'static str, st: OSStatus) -> PlatformError {
    PlatformError::Os {
        call,
        code: st as i64,
    }
}

/// Starts `ioproc` on `dev` with the owned real-time state `rt`.
pub fn start_io(io: &mut DeviceIo, ioproc: AudioDeviceIOProc) -> Result<(), PlatformError> {
    let mut pid: AudioDeviceIOProcID = std::ptr::null_mut();
    // SAFETY: valid device, function and client pointer (owned by `io`).
    let st = unsafe { AudioDeviceCreateIOProcID(io.dev, ioproc, io.rt, &mut pid) };
    if st != 0 || pid.is_null() {
        return Err(os("AudioDeviceCreateIOProcID", st));
    }
    io.proc_id = pid;
    // SAFETY: valid device and IOProc id.
    let st = unsafe { AudioDeviceStart(io.dev, pid) };
    if st != 0 {
        return Err(os("AudioDeviceStart", st));
    }
    io.started = true;
    Ok(())
}

fn stream_formats(
    dev: AudioObjectID,
    scope: u32,
) -> Vec<(AudioObjectID, Option<AudioStreamBasicDescription>)> {
    get_vec::<AudioObjectID>(dev, kAudioDevicePropertyStreams, scope)
        .unwrap_or_default()
        .into_iter()
        .map(|s| {
            (
                s,
                get(s, kAudioStreamPropertyVirtualFormat, SCOPE_GLOBAL).ok(),
            )
        })
        .collect()
}

pub fn check_float(dev: AudioObjectID, scope: u32) -> Result<(), PlatformError> {
    for (_, f) in stream_formats(dev, scope) {
        if let Some(f) = f {
            if !is_f32(&f) {
                return Err(PlatformError::Unsupported("non-float32 device format"));
            }
        }
    }
    Ok(())
}

fn buffer_channels(dev: AudioObjectID, scope: u32) -> Vec<u32> {
    match get_bytes(dev, kAudioDevicePropertyStreamConfiguration, scope) {
        // SAFETY: the property is an AudioBufferList.
        Ok(raw) => unsafe { buffers(raw.as_ptr() as *const AudioBufferList) }
            .iter()
            .map(|b| b.mNumberChannels)
            .collect(),
        Err(_) => Vec::new(),
    }
}

/// Listeners every device stream installs: death, abnormal stop, hog by another process and
/// service restart → `StreamFailed`; `on_latency` is called when a latency property changes.
pub fn stream_listeners(
    shared: &Arc<Shared>,
    active: &Arc<AtomicBool>,
    dev: AudioObjectID,
    kind: StreamKind,
    scope: u32,
    on_latency: Option<Arc<dyn Fn() + Send + Sync>>,
) -> Vec<Listener> {
    let mut targets = vec![
        (dev, addr(kAudioDevicePropertyDeviceIsAlive, SCOPE_GLOBAL)),
        (
            dev,
            addr(kAudioDevicePropertyIOStoppedAbnormally, SCOPE_GLOBAL),
        ),
        (dev, addr(kAudioDevicePropertyHogMode, SCOPE_GLOBAL)),
        (
            kAudioObjectSystemObject,
            addr(kAudioHardwarePropertyServiceRestarted, SCOPE_GLOBAL),
        ),
    ];
    if on_latency.is_some() {
        targets.push((dev, addr(kAudioDevicePropertyLatency, scope)));
        for s in
            get_vec::<AudioObjectID>(dev, kAudioDevicePropertyStreams, scope).unwrap_or_default()
        {
            targets.push((s, addr(kAudioStreamPropertyLatency, SCOPE_GLOBAL)));
        }
    }
    let mut v = Vec::new();
    for (o, a) in targets {
        let sh = shared.clone();
        let act = active.clone();
        let kind = kind.clone();
        let lat = on_latency.clone();
        let cb = move |sel: u32| {
            if !act.load(Ordering::SeqCst) {
                return;
            }
            let error = match sel {
                kAudioDevicePropertyDeviceIsAlive => {
                    if get::<u32>(dev, kAudioDevicePropertyDeviceIsAlive, SCOPE_GLOBAL).unwrap_or(0)
                        != 0
                    {
                        return;
                    }
                    StreamError::DeviceGone
                }
                kAudioDevicePropertyIOStoppedAbnormally => StreamError::Os(sel as i64),
                kAudioDevicePropertyHogMode => {
                    if !hogged_by_other(dev) {
                        return;
                    }
                    StreamError::DeviceBusy
                }
                kAudioHardwarePropertyServiceRestarted => StreamError::Os(sel as i64),
                kAudioDevicePropertyLatency => {
                    if let Some(f) = &lat {
                        f();
                    }
                    return;
                }
                _ => return,
            };
            sh.emit(PlatformEvent::StreamFailed {
                stream: kind.clone(),
                error,
            });
        };
        if let Ok(l) = Listener::add(&shared.queue, o, a, cb) {
            v.push(l);
        }
    }
    v
}

pub fn open_output(
    shared: &Arc<Shared>,
    id: &DeviceId,
    _cfg: OutputConfig,
    handler: Box<dyn RenderHandler>,
) -> Result<Box<dyn Stream>, PlatformError> {
    let dev = device_by_uid(&id.0).ok_or_else(|| PlatformError::DeviceNotFound(id.clone()))?;
    if hogged_by_other(dev) {
        return Err(PlatformError::DeviceBusy(id.clone()));
    }
    // 1. Read everything needed; nothing is written to the device.
    let rate: f64 = get(dev, kAudioDevicePropertyNominalSampleRate, SCOPE_GLOBAL).unwrap_or(0.0);
    if rate <= 0.0 {
        return Err(os("nominal rate", 0));
    }
    check_float(dev, SCOPE_OUTPUT)?;
    let bufs = buffer_channels(dev, SCOPE_OUTPUT);
    let streams = stream_formats(dev, SCOPE_OUTPUT);
    let starting: Vec<u32> = streams
        .iter()
        .map(|(s, _)| {
            get::<u32>(*s, kAudioStreamPropertyStartingChannel, SCOPE_GLOBAL).unwrap_or(0)
        })
        .collect();
    let map = channel_map(&bufs, Some(&starting), preferred_stereo(dev))
        .ok_or(PlatformError::Unsupported("device has no output channels"))?;
    let after_ns = Arc::new(AtomicU64::new(after_timestamp_ns(dev)));
    let block: u32 = get(dev, kAudioDevicePropertyBufferFrameSize, SCOPE_GLOBAL).unwrap_or(0);
    // 2. Pre-allocated real-time state.
    let rt = Box::into_raw(Box::new(RenderRt::new(
        handler,
        map,
        rate,
        after_ns.clone(),
        shared.tb,
    )));
    let active = Arc::new(AtomicBool::new(true));
    let mut io = DeviceIo {
        active: active.clone(),
        listeners: Vec::new(),
        dev,
        proc_id: std::ptr::null_mut(),
        started: false,
        rt: rt as *mut c_void,
        free_rt: free_render,
        format: StreamFormat {
            rate,
            block_frames: block,
        },
    };
    // 3. IOProc + start.
    start_io(&mut io, render_ioproc)?;
    let lat = after_ns.clone();
    io.listeners = stream_listeners(
        shared,
        &active,
        dev,
        StreamKind::Output(id.clone()),
        SCOPE_OUTPUT,
        Some(Arc::new(move || {
            lat.store(after_timestamp_ns(dev), Ordering::Relaxed)
        })),
    );
    Ok(Box::new(io))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rt::test_util::{ts, Abl4};
    use std::sync::Mutex;

    struct Const(f32, f32);
    impl RenderHandler for Const {
        fn on_render(&mut self, stereo: &mut [f32], _info: &RenderInfo) {
            for f in stereo.chunks_exact_mut(2) {
                f[0] = self.0;
                f[1] = self.1;
            }
        }
    }

    type Calls = Arc<Mutex<Vec<(usize, u64, bool, Duration)>>>;
    struct Rec(Calls);
    impl RenderHandler for Rec {
        fn on_render(&mut self, stereo: &mut [f32], info: &RenderInfo) {
            self.0.lock().unwrap().push((
                stereo.len() / 2,
                info.host_time,
                info.discontinuity,
                info.after_timestamp,
            ));
        }
    }

    fn rt(h: Box<dyn RenderHandler>, map: ChannelMap) -> RenderRt {
        RenderRt::new(
            h,
            map,
            48_000.0,
            Arc::new(AtomicU64::new(5_000)),
            Timebase::identity(),
        )
    }

    #[test]
    fn map_simple_stereo_and_mono() {
        assert_eq!(
            channel_map(&[2], None, (1, 2)),
            Some(ChannelMap::Stereo {
                left: Slot {
                    buffer: 0,
                    offset: 0
                },
                right: Slot {
                    buffer: 0,
                    offset: 1
                }
            })
        );
        assert_eq!(
            channel_map(&[1], None, (1, 2)),
            Some(ChannelMap::Mono(Slot {
                buffer: 0,
                offset: 0
            }))
        );
        assert_eq!(channel_map(&[], None, (1, 2)), None);
        assert_eq!(channel_map(&[0], None, (1, 2)), None);
    }

    #[test]
    fn map_follows_preferred_channels_across_streams() {
        // Two stereo streams (channels 1-2 and 3-4), preferred stereo on 3/4.
        assert_eq!(
            channel_map(&[2, 2], Some(&[1, 3]), (3, 4)),
            Some(ChannelMap::Stereo {
                left: Slot {
                    buffer: 1,
                    offset: 0
                },
                right: Slot {
                    buffer: 1,
                    offset: 1
                }
            })
        );
        // Preferred L/R split across buffers (non-interleaved device).
        assert_eq!(
            channel_map(&[1, 1, 1, 1], None, (2, 4)),
            Some(ChannelMap::Stereo {
                left: Slot {
                    buffer: 1,
                    offset: 0
                },
                right: Slot {
                    buffer: 3,
                    offset: 0
                }
            })
        );
        // Preferred channels missing → first two channels.
        assert_eq!(
            channel_map(&[2], None, (7, 8)),
            Some(ChannelMap::Stereo {
                left: Slot {
                    buffer: 0,
                    offset: 0
                },
                right: Slot {
                    buffer: 0,
                    offset: 1
                }
            })
        );
        // Inconsistent StartingChannel list → running sum.
        assert_eq!(
            channel_map(&[2, 2], Some(&[0, 0]), (3, 4)),
            Some(ChannelMap::Stereo {
                left: Slot {
                    buffer: 1,
                    offset: 0
                },
                right: Slot {
                    buffer: 1,
                    offset: 1
                }
            })
        );
    }

    #[test]
    fn renders_into_mapped_channels_and_zeroes_the_rest() {
        let map = channel_map(&[4], None, (3, 4)).unwrap();
        let mut r = rt(Box::new(Const(0.5, -0.5)), map);
        let mut d = vec![9.0f32; 4 * 8];
        let mut abl = Abl4::new(&mut [(4, &mut d)]);
        unsafe { r.process(abl.ptr(), &ts(0.0, 0)) };
        for f in d.chunks_exact(4) {
            assert_eq!(f, &[0.0, 0.0, 0.5, -0.5]);
        }
    }

    #[test]
    fn mono_downmix_and_null_buffers() {
        let mut r = rt(
            Box::new(Const(0.5, 0.25)),
            ChannelMap::Mono(Slot {
                buffer: 0,
                offset: 0,
            }),
        );
        let mut d = vec![0.0f32; 8];
        let mut abl = Abl4::new(&mut [(1, &mut d)]);
        unsafe { r.process(abl.ptr(), &ts(0.0, 0)) };
        assert!(d.iter().all(|&v| v == 0.375));

        // Right buffer NULL: left still written, nothing crashes.
        let map = channel_map(&[1, 1], None, (1, 2)).unwrap();
        let mut r = rt(Box::new(Const(1.0, 2.0)), map);
        let mut l = vec![0.0f32; 8];
        let mut rr = vec![0.0f32; 8];
        let mut abl = Abl4::new(&mut [(1, &mut l), (1, &mut rr)]);
        abl.bufs[1].mData = std::ptr::null_mut();
        unsafe { r.process(abl.ptr(), &ts(0.0, 0)) };
        assert!(l.iter().all(|&v| v == 1.0));
        assert!(rr.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn discontinuity_split_and_after_timestamp() {
        let log = Arc::new(Mutex::new(Vec::new()));
        let map = channel_map(&[2], None, (1, 2)).unwrap();
        let mut r = rt(Box::new(Rec(log.clone())), map);
        let mut d = vec![0.0f32; 512 * 2];
        let mut abl = Abl4::new(&mut [(2, &mut d)]);
        unsafe { r.process(abl.ptr(), &ts(0.0, 100)) };
        unsafe { r.process(abl.ptr(), &ts(512.0, 200)) };
        unsafe { r.process(abl.ptr(), &ts(1536.0, 300)) };
        let mut big = vec![0.0f32; 5000 * 2];
        let mut abl = Abl4::new(&mut [(2, &mut big)]);
        unsafe { r.process(abl.ptr(), &ts(2048.0, 1_000)) };
        let g = log.lock().unwrap();
        assert_eq!(g[0], (512, 100, false, Duration::from_nanos(5_000)));
        assert!(!g[1].2);
        assert!(g[2].2);
        assert_eq!((g[3].0, g[3].2), (4096, false));
        assert_eq!(
            (g[4].0, g[4].1),
            (904, 1_000 + (4096.0 / 48_000.0 * 1e9) as u64)
        );
    }

    #[test]
    fn render_ioproc_does_not_allocate() {
        let map = channel_map(&[1, 1], None, (1, 2)).unwrap();
        let r = Box::into_raw(Box::new(RenderRt::new(
            Box::new(Const(0.1, 0.2)),
            map,
            44_100.0,
            Arc::new(AtomicU64::new(0)),
            Timebase::new(),
        )));
        let mut l = vec![0.0f32; 6000];
        let mut rr = vec![0.0f32; 6000];
        let mut abl = Abl4::new(&mut [(1, &mut l), (1, &mut rr)]);
        let t1 = ts(0.0, 1);
        let t2 = ts(7000.0, 2);
        assert_no_alloc::assert_no_alloc(|| unsafe {
            render_ioproc(
                0,
                &t1,
                std::ptr::null(),
                std::ptr::null(),
                abl.ptr(),
                &t1,
                r.cast(),
            );
            render_ioproc(
                0,
                &t2,
                std::ptr::null(),
                std::ptr::null(),
                abl.ptr(),
                &t2,
                r.cast(),
            );
            render_ioproc(
                0,
                &t2,
                std::ptr::null(),
                std::ptr::null(),
                std::ptr::null_mut(),
                &t2,
                r.cast(),
            );
        });
        unsafe { drop(Box::from_raw(r)) };
        assert!(rr.iter().all(|&v| v == 0.2));
    }
}
