//! Thin Core Audio HAL helpers: hand-written FFI for the plain C entry points (raw pointers, so
//! that NULL buffers from the HAL are never turned into `NonNull`), property getters/setters,
//! host-time conversion and listener registration on a serial dispatch queue.

#![allow(non_upper_case_globals, non_snake_case)]

use std::ffi::c_void;
use std::ptr::{self, NonNull};
use std::sync::Arc;

use block2::RcBlock;
use dispatch2::DispatchQueue;
use objc2_core_audio::{
    AudioObjectAddPropertyListenerBlock, AudioObjectPropertyAddress,
    AudioObjectRemovePropertyListenerBlock,
};
use objc2_core_audio_types::{AudioBuffer, AudioBufferList, AudioTimeStamp};
use objc2_core_foundation::{CFRetained, CFString};

pub type OSStatus = i32;
pub type AudioObjectID = u32;
pub type AudioDeviceIOProcID = *mut c_void;

/// C function-pointer IOProc. All pointers may be NULL depending on the direction of the device.
pub type AudioDeviceIOProc = unsafe extern "C" fn(
    AudioObjectID,
    *const AudioTimeStamp,
    *const AudioBufferList,
    *const AudioTimeStamp,
    *mut AudioBufferList,
    *const AudioTimeStamp,
    *mut c_void,
) -> OSStatus;

pub const fn fcc(s: &[u8; 4]) -> u32 {
    ((s[0] as u32) << 24) | ((s[1] as u32) << 16) | ((s[2] as u32) << 8) | (s[3] as u32)
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
struct MachTimebaseInfo {
    numer: u32,
    denom: u32,
}

#[link(name = "CoreFoundation", kind = "framework")]
extern "C" {}

#[link(name = "CoreAudio", kind = "framework")]
extern "C" {
    fn AudioObjectGetPropertyDataSize(
        id: AudioObjectID,
        addr: *const AudioObjectPropertyAddress,
        qs: u32,
        q: *const c_void,
        out: *mut u32,
    ) -> OSStatus;
    fn AudioObjectGetPropertyData(
        id: AudioObjectID,
        addr: *const AudioObjectPropertyAddress,
        qs: u32,
        q: *const c_void,
        io_size: *mut u32,
        out: *mut c_void,
    ) -> OSStatus;
    fn AudioObjectSetPropertyData(
        id: AudioObjectID,
        addr: *const AudioObjectPropertyAddress,
        qs: u32,
        q: *const c_void,
        size: u32,
        data: *const c_void,
    ) -> OSStatus;
    fn AudioObjectHasProperty(id: AudioObjectID, addr: *const AudioObjectPropertyAddress) -> u8;
    pub fn AudioDeviceCreateIOProcID(
        dev: AudioObjectID,
        p: AudioDeviceIOProc,
        client: *mut c_void,
        out: *mut AudioDeviceIOProcID,
    ) -> OSStatus;
    pub fn AudioDeviceDestroyIOProcID(dev: AudioObjectID, id: AudioDeviceIOProcID) -> OSStatus;
    pub fn AudioDeviceStart(dev: AudioObjectID, id: AudioDeviceIOProcID) -> OSStatus;
    pub fn AudioDeviceStop(dev: AudioObjectID, id: AudioDeviceIOProcID) -> OSStatus;
    pub fn AudioHardwareCreateAggregateDevice(
        desc: *const c_void,
        out: *mut AudioObjectID,
    ) -> OSStatus;
    pub fn AudioHardwareDestroyAggregateDevice(id: AudioObjectID) -> OSStatus;
    pub fn AudioHardwareDestroyProcessTap(id: AudioObjectID) -> OSStatus;
}

extern "C" {
    fn mach_absolute_time() -> u64;
    fn mach_timebase_info(info: *mut MachTimebaseInfo) -> i32;
}

pub const kAudioObjectSystemObject: AudioObjectID = 1;
pub const kAudioObjectUnknown: AudioObjectID = 0;
pub const SCOPE_GLOBAL: u32 = fcc(b"glob");
pub const SCOPE_INPUT: u32 = fcc(b"inpt");
pub const SCOPE_OUTPUT: u32 = fcc(b"outp");
pub const ELEMENT_MAIN: u32 = 0;

// System object
pub const kAudioHardwarePropertyDevices: u32 = fcc(b"dev#");
pub const kAudioHardwarePropertyDefaultOutputDevice: u32 = fcc(b"dOut");
pub const kAudioHardwarePropertyServiceRestarted: u32 = fcc(b"srst");
pub const kAudioHardwarePropertyProcessObjectList: u32 = fcc(b"prs#");
pub const kAudioHardwarePropertyTranslatePIDToProcessObject: u32 = fcc(b"id2p");
pub const kAudioHardwarePropertyTranslateUIDToDevice: u32 = fcc(b"uidd");
// Objects
pub const kAudioObjectPropertyName: u32 = fcc(b"lnam");
pub const kAudioObjectPropertyClass: u32 = fcc(b"clas");
// Devices
pub const kAudioDevicePropertyDeviceUID: u32 = fcc(b"uid ");
pub const kAudioDevicePropertyTransportType: u32 = fcc(b"tran");
pub const kAudioDevicePropertyNominalSampleRate: u32 = fcc(b"nsrt");
pub const kAudioDevicePropertyStreams: u32 = fcc(b"stm#");
pub const kAudioDevicePropertyStreamConfiguration: u32 = fcc(b"slay");
pub const kAudioDevicePropertyLatency: u32 = fcc(b"ltnc");
pub const kAudioDevicePropertySafetyOffset: u32 = fcc(b"saft");
pub const kAudioDevicePropertyBufferFrameSize: u32 = fcc(b"fsiz");
pub const kAudioDevicePropertyDeviceIsAlive: u32 = fcc(b"livn");
pub const kAudioDevicePropertyIsHidden: u32 = fcc(b"hidn");
pub const kAudioDevicePropertyHogMode: u32 = fcc(b"oink");
pub const kAudioDevicePropertyDeviceCanBeDefaultDevice: u32 = fcc(b"dflt");
pub const kAudioDevicePropertyPreferredChannelsForStereo: u32 = fcc(b"dch2");
pub const kAudioDevicePropertyDeviceHasChanged: u32 = fcc(b"diff");
pub const kAudioDevicePropertyDataSource: u32 = fcc(b"ssrc");
pub const kAudioDevicePropertyIOStoppedAbnormally: u32 = fcc(b"stpd");
pub const kAudioDevicePropertyDeviceIsRunningSomewhere: u32 = fcc(b"gone");
pub const kAudioDevicePropertyVolumeScalar: u32 = fcc(b"volm");
pub const kAudioDevicePropertyMute: u32 = fcc(b"mute");
// Streams
pub const kAudioStreamPropertyLatency: u32 = fcc(b"ltnc");
pub const kAudioStreamPropertyVirtualFormat: u32 = fcc(b"sfmt");
pub const kAudioStreamPropertyStartingChannel: u32 = fcc(b"schn");
// Taps
pub const kAudioTapPropertyFormat: u32 = fcc(b"tfmt");
pub const kAudioTapPropertyDescription: u32 = fcc(b"tdsc");
// Processes
pub const kAudioProcessPropertyPID: u32 = fcc(b"ppid");
pub const kAudioProcessPropertyBundleID: u32 = fcc(b"pbid");
pub const kAudioProcessPropertyDevices: u32 = fcc(b"pdv#");
pub const kAudioProcessPropertyIsRunningInput: u32 = fcc(b"piri");
pub const kAudioProcessPropertyIsRunningOutput: u32 = fcc(b"piro");
// Class / transport / format ids
pub const kAudioAggregateDeviceClassID: u32 = fcc(b"aagg");
pub const kAudioFormatLinearPCM: u32 = fcc(b"lpcm");
pub const kAudioFormatFlagIsFloat: u32 = 1;

// Status codes treated as "capture refused" by the permission probe.
pub const STATUS_PERMISSION: OSStatus = fcc(b"!hog") as OSStatus;
pub const STATUS_WHAT: OSStatus = fcc(b"wat?") as OSStatus;

pub fn addr(selector: u32, scope: u32) -> AudioObjectPropertyAddress {
    addr_el(selector, scope, ELEMENT_MAIN)
}

pub fn addr_el(selector: u32, scope: u32, element: u32) -> AudioObjectPropertyAddress {
    AudioObjectPropertyAddress {
        mSelector: selector,
        mScope: scope,
        mElement: element,
    }
}

pub fn has(id: AudioObjectID, selector: u32, scope: u32, element: u32) -> bool {
    let a = addr_el(selector, scope, element);
    // SAFETY: `a` is a valid address struct; the call only reads it.
    unsafe { AudioObjectHasProperty(id, &a) != 0 }
}

/// Reads a fixed-size property. `T` must be a plain-old-data type matching the property layout.
pub fn get<T: Copy>(id: AudioObjectID, selector: u32, scope: u32) -> Result<T, OSStatus> {
    get_el(id, selector, scope, ELEMENT_MAIN)
}

pub fn get_el<T: Copy>(
    id: AudioObjectID,
    selector: u32,
    scope: u32,
    element: u32,
) -> Result<T, OSStatus> {
    get_raw_q(id, addr_el(selector, scope, element), ptr::null(), 0)
}

/// Reads a fixed-size property with a qualifier.
pub fn get_q<T: Copy, Q>(
    id: AudioObjectID,
    selector: u32,
    scope: u32,
    q: &Q,
) -> Result<T, OSStatus> {
    get_raw_q(
        id,
        addr(selector, scope),
        q as *const Q as *const c_void,
        std::mem::size_of::<Q>() as u32,
    )
}

fn get_raw_q<T: Copy>(
    id: AudioObjectID,
    a: AudioObjectPropertyAddress,
    qp: *const c_void,
    qs: u32,
) -> Result<T, OSStatus> {
    let mut v = std::mem::MaybeUninit::<T>::zeroed();
    let mut size = std::mem::size_of::<T>() as u32;
    // SAFETY: `v` provides `size` writable bytes; the qualifier pointer/size pair is consistent.
    let st =
        unsafe { AudioObjectGetPropertyData(id, &a, qs, qp, &mut size, v.as_mut_ptr().cast()) };
    if st != 0 {
        return Err(st);
    }
    if size as usize != std::mem::size_of::<T>() {
        return Err(fcc(b"!siz") as OSStatus);
    }
    // SAFETY: the HAL filled exactly size_of::<T>() bytes of a POD type (zero-initialised before).
    Ok(unsafe { v.assume_init() })
}

pub fn set<T: Copy>(id: AudioObjectID, selector: u32, scope: u32, v: &T) -> Result<(), OSStatus> {
    let a = addr(selector, scope);
    // SAFETY: `v` points to size_of::<T>() readable bytes.
    let st = unsafe {
        AudioObjectSetPropertyData(
            id,
            &a,
            0,
            ptr::null(),
            std::mem::size_of::<T>() as u32,
            v as *const T as *const c_void,
        )
    };
    if st == 0 {
        Ok(())
    } else {
        Err(st)
    }
}

/// Reads a variable-length property as raw bytes (8-byte aligned storage).
pub fn get_bytes(id: AudioObjectID, selector: u32, scope: u32) -> Result<Vec<u64>, OSStatus> {
    let a = addr(selector, scope);
    let mut size = 0u32;
    // SAFETY: valid address and out pointer.
    let st = unsafe { AudioObjectGetPropertyDataSize(id, &a, 0, ptr::null(), &mut size) };
    if st != 0 {
        return Err(st);
    }
    let words = (size as usize).div_ceil(8).max(1);
    let mut buf = vec![0u64; words];
    // SAFETY: `buf` has at least `size` writable bytes.
    let st = unsafe {
        AudioObjectGetPropertyData(id, &a, 0, ptr::null(), &mut size, buf.as_mut_ptr().cast())
    };
    if st != 0 {
        return Err(st);
    }
    buf.truncate((size as usize).div_ceil(8).max(1));
    Ok(buf)
}

/// Reads an array property of POD elements (`AudioObjectID[]` etc.).
pub fn get_vec<T: Copy + Default>(
    id: AudioObjectID,
    selector: u32,
    scope: u32,
) -> Result<Vec<T>, OSStatus> {
    let a = addr(selector, scope);
    let mut size = 0u32;
    // SAFETY: valid address and out pointer.
    let st = unsafe { AudioObjectGetPropertyDataSize(id, &a, 0, ptr::null(), &mut size) };
    if st != 0 {
        return Err(st);
    }
    let el = std::mem::size_of::<T>().max(1);
    let mut v = vec![T::default(); size as usize / el];
    if v.is_empty() {
        return Ok(v);
    }
    let mut size = (v.len() * el) as u32;
    // SAFETY: `v` has `size` writable bytes of POD elements.
    let st = unsafe {
        AudioObjectGetPropertyData(id, &a, 0, ptr::null(), &mut size, v.as_mut_ptr().cast())
    };
    if st != 0 {
        return Err(st);
    }
    v.truncate(size as usize / el);
    Ok(v)
}

/// Reads a CFString property (the caller owns the returned reference per HAL convention).
pub fn get_string(id: AudioObjectID, selector: u32, scope: u32) -> Result<String, OSStatus> {
    let p: *const CFString = get(id, selector, scope)?;
    match NonNull::new(p as *mut CFString) {
        // SAFETY: the HAL returns a +1 retained CFStringRef for string properties.
        Some(nn) => Ok(unsafe { CFRetained::from_raw(nn) }.to_string()),
        None => Ok(String::new()),
    }
}

/// Number of channels in a device's stream configuration (sum over buffers) for `scope`.
pub fn channels(dev: AudioObjectID, scope: u32) -> u32 {
    match get_bytes(dev, kAudioDevicePropertyStreamConfiguration, scope) {
        Ok(raw) => {
            // SAFETY: the property is an AudioBufferList laid out in `raw` (8-byte aligned).
            unsafe { buffers(raw.as_ptr() as *const AudioBufferList) }
                .iter()
                .map(|b| b.mNumberChannels)
                .sum()
        }
        Err(_) => 0,
    }
}

/// The buffers of an AudioBufferList as a slice (empty for NULL).
///
/// # Safety
/// `abl` must be NULL or point to a valid AudioBufferList whose buffers stay valid for `'a`.
pub unsafe fn buffers<'a>(abl: *const AudioBufferList) -> &'a [AudioBuffer] {
    if abl.is_null() {
        return &[];
    }
    // SAFETY: guaranteed by the caller.
    unsafe {
        let n = (*abl).mNumberBuffers as usize;
        std::slice::from_raw_parts(ptr::addr_of!((*abl).mBuffers) as *const AudioBuffer, n)
    }
}

pub fn system_devices() -> Vec<AudioObjectID> {
    get_vec(
        kAudioObjectSystemObject,
        kAudioHardwarePropertyDevices,
        SCOPE_GLOBAL,
    )
    .unwrap_or_default()
}

pub fn default_output_id() -> AudioObjectID {
    get(
        kAudioObjectSystemObject,
        kAudioHardwarePropertyDefaultOutputDevice,
        SCOPE_GLOBAL,
    )
    .unwrap_or(kAudioObjectUnknown)
}

pub fn device_uid(dev: AudioObjectID) -> Option<String> {
    get_string(dev, kAudioDevicePropertyDeviceUID, SCOPE_GLOBAL)
        .ok()
        .filter(|s| !s.is_empty())
}

/// Resolves a device UID to its current AudioObjectID.
pub fn device_by_uid(uid: &str) -> Option<AudioObjectID> {
    let s = CFString::from_str(uid);
    let p: *const CFString = &*s;
    let id: AudioObjectID = get_q(
        kAudioObjectSystemObject,
        kAudioHardwarePropertyTranslateUIDToDevice,
        SCOPE_GLOBAL,
        &p,
    )
    .ok()?;
    if id != kAudioObjectUnknown {
        return Some(id);
    }
    // Fallback: linear search (TranslateUIDToDevice does not resolve every device class).
    system_devices()
        .into_iter()
        .find(|&d| device_uid(d).as_deref() == Some(uid))
}

pub fn own_process_object() -> AudioObjectID {
    let pid: i32 = std::process::id() as i32;
    get_q(
        kAudioObjectSystemObject,
        kAudioHardwarePropertyTranslatePIDToProcessObject,
        SCOPE_GLOBAL,
        &pid,
    )
    .unwrap_or(kAudioObjectUnknown)
}

/// Latency figures of one direction of a device, in frames.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Latency {
    pub rate: f64,
    pub device: u32,
    pub stream: u32,
    pub safety: u32,
    pub buffer: u32,
}

impl Latency {
    pub fn read(dev: AudioObjectID, scope: u32) -> Latency {
        let streams: Vec<AudioObjectID> =
            get_vec(dev, kAudioDevicePropertyStreams, scope).unwrap_or_default();
        Latency {
            rate: get(dev, kAudioDevicePropertyNominalSampleRate, SCOPE_GLOBAL).unwrap_or(0.0),
            device: get(dev, kAudioDevicePropertyLatency, scope).unwrap_or(0),
            stream: streams
                .first()
                .and_then(|&s| get::<u32>(s, kAudioStreamPropertyLatency, SCOPE_GLOBAL).ok())
                .unwrap_or(0),
            safety: get(dev, kAudioDevicePropertySafetyOffset, scope).unwrap_or(0),
            buffer: get(dev, kAudioDevicePropertyBufferFrameSize, SCOPE_GLOBAL).unwrap_or(0),
        }
    }

    /// From the IOProc timestamp to the sound leaving (or entering) the device, seconds.
    pub fn after_timestamp_s(&self) -> f64 {
        if self.rate > 0.0 {
            (self.device as f64 + self.stream as f64) / self.rate
        } else {
            0.0
        }
    }

    /// Whole path including the IO buffer and the safety offset, seconds.
    pub fn total_s(&self) -> f64 {
        if self.rate > 0.0 {
            self.after_timestamp_s() + (self.buffer as f64 + self.safety as f64) / self.rate
        } else {
            0.0
        }
    }
}

/// Host-tick to nanosecond conversion (mach timebase). Copyable, usable on real-time threads.
#[derive(Clone, Copy, Debug)]
pub struct Timebase {
    numer: u64,
    denom: u64,
}

impl Timebase {
    pub fn new() -> Timebase {
        let mut tb = MachTimebaseInfo::default();
        // SAFETY: valid out pointer.
        unsafe { mach_timebase_info(&mut tb) };
        if tb.numer == 0 || tb.denom == 0 {
            tb = MachTimebaseInfo { numer: 1, denom: 1 };
        }
        Timebase {
            numer: tb.numer as u64,
            denom: tb.denom as u64,
        }
    }

    /// Identity conversion (for tests).
    #[cfg(test)]
    pub fn identity() -> Timebase {
        Timebase { numer: 1, denom: 1 }
    }

    #[inline]
    pub fn ticks_to_ns(self, ticks: u64) -> u64 {
        ((ticks as u128 * self.numer as u128) / self.denom as u128) as u64
    }

    pub fn now_ns(self) -> u64 {
        // SAFETY: no preconditions.
        self.ticks_to_ns(unsafe { mach_absolute_time() })
    }
}

impl Default for Timebase {
    fn default() -> Self {
        Self::new()
    }
}

/// Converts `frames` at `rate` (scaled by `rate_scalar`) into nanoseconds. Real-time safe.
#[inline]
pub fn frames_to_ns(frames: f64, rate: f64, rate_scalar: f64) -> u64 {
    let rs = if rate_scalar > 0.0 { rate_scalar } else { 1.0 };
    if rate > 0.0 {
        (frames / (rate * rs) * 1e9).max(0.0) as u64
    } else {
        0
    }
}

pub type ListenerFn = dyn Fn(u32, NonNull<AudioObjectPropertyAddress>) + 'static;

/// A registered HAL property listener (block + serial queue). Unregisters on drop.
pub struct Listener {
    obj: AudioObjectID,
    addr: AudioObjectPropertyAddress,
    block: RcBlock<ListenerFn>,
    queue: Arc<QueueHandle>,
}

// SAFETY: the block closures only capture `Send + Sync` state (Arcs of thread-safe data); the
// block object itself is reference counted by the Objective-C runtime atomically.
unsafe impl Send for Listener {}

impl Listener {
    /// Registers `f` for `(obj, selector, scope, element)`; `f` receives the selector that fired.
    pub fn add(
        queue: &Arc<QueueHandle>,
        obj: AudioObjectID,
        a: AudioObjectPropertyAddress,
        f: impl Fn(u32) + Send + Sync + 'static,
    ) -> Result<Listener, OSStatus> {
        let block: RcBlock<ListenerFn> =
            RcBlock::new(move |n: u32, addrs: NonNull<AudioObjectPropertyAddress>| {
                for i in 0..n as usize {
                    // SAFETY: the HAL passes `n` valid addresses.
                    let a = unsafe { *addrs.as_ptr().add(i) };
                    f(a.mSelector);
                }
            });
        let mut a2 = a;
        // SAFETY: valid object, address, queue and block (the HAL copies the block).
        let st = unsafe {
            AudioObjectAddPropertyListenerBlock(
                obj,
                NonNull::from(&mut a2),
                Some(&queue.0),
                RcBlock::as_ptr(&block),
            )
        };
        if st != 0 {
            return Err(st);
        }
        Ok(Listener {
            obj,
            addr: a,
            block,
            queue: queue.clone(),
        })
    }
}

impl Drop for Listener {
    fn drop(&mut self) {
        let mut a = self.addr;
        // SAFETY: same object, address, queue and block pointer as registered.
        unsafe {
            AudioObjectRemovePropertyListenerBlock(
                self.obj,
                NonNull::from(&mut a),
                Some(&self.queue.0),
                RcBlock::as_ptr(&self.block),
            );
        }
    }
}

/// The adapter's private serial dispatch queue for property listeners.
pub struct QueueHandle(pub dispatch2::DispatchRetained<DispatchQueue>);

impl QueueHandle {
    pub fn new(label: &str) -> Arc<QueueHandle> {
        Arc::new(QueueHandle(DispatchQueue::new(
            label,
            dispatch2::DispatchQueueAttr::SERIAL,
        )))
    }
}
