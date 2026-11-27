//! Thread framework shared by all streams: COM apartment and MMCSS scopes, Win32 events, the
//! QPC clock, the per-stream thread with its one-shot initialization result, and small COM
//! helpers (strings, property store values, error mapping).

use std::marker::PhantomData;
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

use windows::Win32::Foundation::{
    CloseHandle, HANDLE, PROPERTYKEY, RPC_E_CHANGED_MODE, WAIT_FAILED, WAIT_OBJECT_0, WAIT_TIMEOUT,
};
use windows::Win32::Media::Audio::{
    AudioCategory_Other, AudioClientProperties, IAudioClient, IAudioClient2, IMMDeviceEnumerator,
    MMDeviceEnumerator, AUDCLNT_STREAMOPTIONS_NONE, WAVEFORMATEX, WAVEFORMATEXTENSIBLE,
};
use windows::Win32::Media::KernelStreaming::WAVE_FORMAT_EXTENSIBLE;
use windows::Win32::System::Com::StructuredStorage::{
    PropVariantClear, PropVariantToGUID, PropVariantToStringAlloc, PropVariantToUInt32, PROPVARIANT,
};
use windows::Win32::System::Com::{
    CoCreateInstance, CoInitializeEx, CoTaskMemFree, CoUninitialize, CLSCTX_ALL,
    COINIT_MULTITHREADED,
};
use windows::Win32::System::Performance::{QueryPerformanceCounter, QueryPerformanceFrequency};
use windows::Win32::System::Threading::{
    AvRevertMmThreadCharacteristics, AvSetMmThreadCharacteristicsW, CreateEventW, GetCurrentThread,
    SetEvent, SetThreadPriority, WaitForMultipleObjects, THREAD_PRIORITY_TIME_CRITICAL,
};
use windows::Win32::System::Variant::VT_BLOB;
use windows::Win32::UI::Shell::PropertiesSystem::IPropertyStore;
use windows_core::{Interface, PCWSTR, PWSTR};
use wye_platform::{
    EventSink, PlatformError, PlatformEvent, Stream, StreamError, StreamFormat, StreamKind,
};

use crate::classify::stream_error_from_hresult;
use crate::format::{RawWaveFormat, WaveFormat, TAG_EXTENSIBLE};
use crate::timing::qpc_ticks_to_ns;

/// Maps a failed OS call to a platform error.
pub(crate) fn os_err(call: &'static str, e: &windows_core::Error) -> PlatformError {
    PlatformError::Os {
        call,
        code: e.code().0 as i64,
    }
}

/// Maps a failed call inside a running stream to a stream error.
#[inline]
pub(crate) fn stream_err(e: &windows_core::Error) -> StreamError {
    stream_error_from_hresult(e.code().0)
}

/// Joins the multithreaded apartment for the lifetime of the value. A thread that is already in
/// a single-threaded apartment (e.g. a UI thread) keeps it; COM still works there.
pub(crate) struct ComScope {
    initialized: bool,
    _not_send: PhantomData<*const ()>,
}

impl ComScope {
    pub(crate) fn enter() -> Result<ComScope, PlatformError> {
        // SAFETY: plain COM initialization; balanced by `CoUninitialize` in `Drop`.
        let hr = unsafe { CoInitializeEx(None, COINIT_MULTITHREADED) };
        if hr.is_ok() {
            Ok(ComScope {
                initialized: true,
                _not_send: PhantomData,
            })
        } else if hr == RPC_E_CHANGED_MODE {
            Ok(ComScope {
                initialized: false,
                _not_send: PhantomData,
            })
        } else {
            Err(PlatformError::Os {
                call: "CoInitializeEx",
                code: hr.0 as i64,
            })
        }
    }
}

impl Drop for ComScope {
    fn drop(&mut self) {
        if self.initialized {
            // SAFETY: balances the successful `CoInitializeEx` on this thread.
            unsafe { CoUninitialize() };
        }
    }
}

/// Registers the current thread with MMCSS as "Pro Audio" (priority 23–26). Falls back to
/// `THREAD_PRIORITY_TIME_CRITICAL` when MMCSS is unavailable.
pub(crate) struct Mmcss {
    handle: Option<HANDLE>,
    _not_send: PhantomData<*const ()>,
}

impl Mmcss {
    pub(crate) fn enter() -> Mmcss {
        let mut task_index = 0u32;
        // SAFETY: valid task name and out pointer.
        let handle = match unsafe {
            AvSetMmThreadCharacteristicsW(windows_core::w!("Pro Audio"), &mut task_index)
        } {
            Ok(h) => Some(h),
            Err(_) => {
                // SAFETY: pseudo handle of the current thread.
                let _ =
                    unsafe { SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL) };
                None
            }
        };
        Mmcss {
            handle,
            _not_send: PhantomData,
        }
    }
}

impl Drop for Mmcss {
    fn drop(&mut self) {
        if let Some(h) = self.handle.take() {
            // SAFETY: handle returned by `AvSetMmThreadCharacteristicsW` on this thread.
            let _ = unsafe { AvRevertMmThreadCharacteristics(h) };
        }
    }
}

/// An owned Win32 event object.
pub(crate) struct Event(HANDLE);

// SAFETY: event handles may be used from any thread.
unsafe impl Send for Event {}
// SAFETY: `SetEvent`/waits on an event handle are thread-safe.
unsafe impl Sync for Event {}

impl Event {
    /// `manual_reset`: stays signalled until reset (stop events); otherwise auto-reset (WASAPI
    /// buffer events).
    pub(crate) fn new(manual_reset: bool) -> Result<Event, PlatformError> {
        // SAFETY: anonymous event, default security.
        unsafe { CreateEventW(None, manual_reset, false, PCWSTR::null()) }
            .map(Event)
            .map_err(|e| os_err("CreateEventW", &e))
    }

    pub(crate) fn handle(&self) -> HANDLE {
        self.0
    }

    pub(crate) fn set(&self) {
        // SAFETY: valid event handle owned by `self`.
        let _ = unsafe { SetEvent(self.0) };
    }
}

impl Drop for Event {
    fn drop(&mut self) {
        // SAFETY: handle owned by `self`, closed exactly once.
        let _ = unsafe { CloseHandle(self.0) };
    }
}

/// Result of a multi-object wait.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Wake {
    /// Index of the signalled handle.
    Signalled(usize),
    Timeout,
    Failed,
}

/// `WaitForMultipleObjects(handles, wait_all = false, timeout_ms)`.
#[inline]
pub(crate) fn wait_any(handles: &[HANDLE], timeout_ms: u32) -> Wake {
    // SAFETY: all handles are valid event handles owned by the caller.
    let r = unsafe { WaitForMultipleObjects(handles, false, timeout_ms) };
    if r == WAIT_TIMEOUT {
        Wake::Timeout
    } else if r == WAIT_FAILED {
        Wake::Failed
    } else {
        let index = r.0.wrapping_sub(WAIT_OBJECT_0.0) as usize;
        if index < handles.len() {
            Wake::Signalled(index)
        } else {
            // WAIT_ABANDONED_* (mutexes only) or anything unexpected.
            Wake::Failed
        }
    }
}

/// The process-wide host clock (QPC in nanoseconds). WASAPI timestamps are the same counter
/// expressed in 100 ns units.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Qpc {
    freq: u64,
}

impl Qpc {
    pub(crate) fn new() -> Qpc {
        let mut f = 0i64;
        // SAFETY: valid out pointer; cannot fail on Windows XP and later.
        let _ = unsafe { QueryPerformanceFrequency(&mut f) };
        Qpc {
            freq: f.max(1) as u64,
        }
    }

    #[inline]
    pub(crate) fn now_ns(&self) -> u64 {
        let mut t = 0i64;
        // SAFETY: valid out pointer.
        let _ = unsafe { QueryPerformanceCounter(&mut t) };
        qpc_ticks_to_ns(t.max(0) as u64, self.freq)
    }
}

/// Shared slot for the event sink registered by `Platform::watch`, so stream threads can report
/// `StreamFailed` after their audio loop ends.
pub(crate) type SinkSlot = Arc<Mutex<Option<Arc<EventSink>>>>;

/// Sends an event to the registered sink, if any. Never called from inside an audio loop.
pub(crate) fn emit(slot: &SinkSlot, event: PlatformEvent) {
    let sink = slot.lock().ok().and_then(|g| g.clone());
    if let Some(sink) = sink {
        sink(event);
    }
}

/// A stream whose COM objects live on its own thread.
pub(crate) trait StreamLoop {
    fn format(&self) -> StreamFormat;
    /// Runs until `stop` is signalled (`Ok`) or the stream fails (`Err`). Real-time code.
    fn run(&mut self, stop: &Event) -> Result<(), StreamError>;
}

/// Handle to a running stream thread; dropping it signals the stop event and joins the thread
/// (which releases every COM object it created).
pub(crate) struct StreamThread {
    stop: Arc<Event>,
    join: Option<JoinHandle<()>>,
    format: StreamFormat,
}

impl Stream for StreamThread {
    fn format(&self) -> StreamFormat {
        self.format
    }
}

impl Drop for StreamThread {
    fn drop(&mut self) {
        self.stop.set();
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

/// Spawns a stream thread: COM (MTA) → MMCSS → `init` (creates the COM objects on this thread) →
/// reports the result through a one-shot channel → `run`. Blocks until `init` finished.
/// If `run` fails, `StreamFailed { stream: kind, … }` is sent to the event sink.
pub(crate) fn spawn_stream<L, F>(
    name: &str,
    kind: StreamKind,
    sink: SinkSlot,
    init: F,
) -> Result<Box<dyn Stream>, PlatformError>
where
    L: StreamLoop,
    F: FnOnce() -> Result<L, PlatformError> + Send + 'static,
{
    let stop = Arc::new(Event::new(true)?);
    let thread_stop = stop.clone();
    let (ready_tx, ready_rx) = mpsc::sync_channel::<Result<StreamFormat, PlatformError>>(1);
    let join = std::thread::Builder::new()
        .name(name.to_owned())
        .spawn(move || {
            let com = match ComScope::enter() {
                Ok(c) => c,
                Err(e) => {
                    let _ = ready_tx.send(Err(e));
                    return;
                }
            };
            let mmcss = Mmcss::enter();
            let mut stream = match init() {
                Ok(s) => s,
                Err(e) => {
                    let _ = ready_tx.send(Err(e));
                    return;
                }
            };
            let _ = ready_tx.send(Ok(stream.format()));
            drop(ready_tx);
            let result = stream.run(&thread_stop);
            drop(stream);
            drop(mmcss);
            drop(com);
            if let Err(error) = result {
                emit(
                    &sink,
                    PlatformEvent::StreamFailed {
                        stream: kind,
                        error,
                    },
                );
            }
        })
        .map_err(|e| PlatformError::Os {
            call: "spawn",
            code: e.raw_os_error().unwrap_or(-1) as i64,
        })?;
    match ready_rx.recv() {
        Ok(Ok(format)) => Ok(Box::new(StreamThread {
            stop,
            join: Some(join),
            format,
        })),
        Ok(Err(e)) => {
            let _ = join.join();
            Err(e)
        }
        Err(_) => {
            let _ = join.join();
            Err(PlatformError::Os {
                call: "stream thread",
                code: -1,
            })
        }
    }
}

/// Creates the device enumerator (on the calling thread's apartment).
pub(crate) fn enumerator() -> Result<IMMDeviceEnumerator, PlatformError> {
    // SAFETY: standard COM activation of the MMDevice enumerator.
    unsafe { CoCreateInstance(&MMDeviceEnumerator, None, CLSCTX_ALL) }
        .map_err(|e| os_err("CoCreateInstance(MMDeviceEnumerator)", &e))
}

/// Takes ownership of a COM-allocated wide string.
pub(crate) fn take_pwstr(p: PWSTR) -> String {
    if p.is_null() {
        return String::new();
    }
    // SAFETY: `p` is a NUL-terminated string allocated with CoTaskMemAlloc; freed once here.
    unsafe {
        let s = p.to_string().unwrap_or_default();
        CoTaskMemFree(Some(p.0 as *const _));
        s
    }
}

/// Owned `PROPVARIANT` that is cleared on drop.
struct PropValue(PROPVARIANT);

impl Drop for PropValue {
    fn drop(&mut self) {
        // SAFETY: the value came from `IPropertyStore::GetValue`.
        let _ = unsafe { PropVariantClear(&mut self.0) };
    }
}

fn prop_value(store: &IPropertyStore, key: &PROPERTYKEY) -> Option<PropValue> {
    // SAFETY: valid key pointer.
    unsafe { store.GetValue(key) }.ok().map(PropValue)
}

pub(crate) fn prop_string(store: &IPropertyStore, key: &PROPERTYKEY) -> Option<String> {
    let v = prop_value(store, key)?;
    // SAFETY: `v` is a valid PROPVARIANT; the result is CoTaskMem-allocated.
    let p = unsafe { PropVariantToStringAlloc(&v.0) }.ok()?;
    let s = take_pwstr(p);
    (!s.is_empty()).then_some(s)
}

pub(crate) fn prop_u32(store: &IPropertyStore, key: &PROPERTYKEY) -> Option<u32> {
    let v = prop_value(store, key)?;
    // SAFETY: valid PROPVARIANT.
    unsafe { PropVariantToUInt32(&v.0) }.ok()
}

pub(crate) fn prop_guid(store: &IPropertyStore, key: &PROPERTYKEY) -> Option<u128> {
    let v = prop_value(store, key)?;
    // SAFETY: valid PROPVARIANT.
    unsafe { PropVariantToGUID(&v.0) }.ok().map(|g| g.to_u128())
}

/// Reads a `VT_BLOB` property holding a `WAVEFORMATEX(TENSIBLE)` (e.g. the device format).
pub(crate) fn prop_wave_format(store: &IPropertyStore, key: &PROPERTYKEY) -> Option<RawWaveFormat> {
    let v = prop_value(store, key)?;
    // SAFETY: union access guarded by the variant type; the blob is owned by `v`.
    unsafe {
        let inner = &v.0.Anonymous.Anonymous;
        if inner.vt != VT_BLOB {
            return None;
        }
        let blob = inner.Anonymous.blob;
        if blob.pBlobData.is_null() || (blob.cbSize as usize) < std::mem::size_of::<WAVEFORMATEX>()
        {
            return None;
        }
        Some(read_wave_format(
            blob.pBlobData as *const WAVEFORMATEX,
            blob.cbSize as usize,
        ))
    }
}

/// Reads a `WAVEFORMATEX` (and its extensible tail when `available` covers it).
///
/// # Safety
/// `p` must point to at least `available` readable bytes, `available >= size_of::<WAVEFORMATEX>()`.
pub(crate) unsafe fn read_wave_format(p: *const WAVEFORMATEX, available: usize) -> RawWaveFormat {
    // SAFETY: caller guarantees the header is readable; packed struct → unaligned read.
    let wf = unsafe { std::ptr::read_unaligned(p) };
    let tag = wf.wFormatTag;
    let mut raw = RawWaveFormat {
        tag,
        channels: wf.nChannels,
        rate: wf.nSamplesPerSec,
        block_align: wf.nBlockAlign,
        bits: wf.wBitsPerSample,
        extensible: None,
    };
    let ext_len = std::mem::size_of::<WAVEFORMATEXTENSIBLE>();
    if tag == TAG_EXTENSIBLE
        && u32::from(tag) == WAVE_FORMAT_EXTENSIBLE
        && available >= ext_len
        && wf.cbSize as usize + std::mem::size_of::<WAVEFORMATEX>() >= ext_len
    {
        // SAFETY: the extensible tail is within `available` bytes.
        let ext = unsafe { std::ptr::read_unaligned(p as *const WAVEFORMATEXTENSIBLE) };
        // SAFETY: plain-old-data union field.
        let valid = unsafe { ext.Samples.wValidBitsPerSample };
        let mask = ext.dwChannelMask;
        let sub = ext.SubFormat;
        raw.extensible = Some((valid, mask, sub.to_u128()));
    }
    raw
}

/// The mix format of an audio client (CoTaskMem allocation, freed on drop).
pub(crate) struct MixFormat {
    ptr: *mut WAVEFORMATEX,
    pub(crate) format: WaveFormat,
}

impl MixFormat {
    pub(crate) fn of(client: &IAudioClient) -> Result<MixFormat, PlatformError> {
        // SAFETY: returns a CoTaskMem-allocated WAVEFORMATEX(TENSIBLE).
        let ptr = unsafe { client.GetMixFormat() }.map_err(|e| os_err("GetMixFormat", &e))?;
        if ptr.is_null() {
            return Err(PlatformError::Os {
                call: "GetMixFormat",
                code: -1,
            });
        }
        // SAFETY: header is readable; the extensible tail is present iff cbSize says so.
        let header = unsafe { std::ptr::read_unaligned(ptr) };
        let available = std::mem::size_of::<WAVEFORMATEX>() + header.cbSize as usize;
        // SAFETY: `available` bytes were allocated by the audio engine.
        let raw = unsafe { read_wave_format(ptr, available) };
        match WaveFormat::parse(&raw) {
            Some(format) => Ok(MixFormat { ptr, format }),
            None => {
                // SAFETY: allocated by GetMixFormat; not retained.
                unsafe { CoTaskMemFree(Some(ptr as *const _)) };
                Err(PlatformError::Os {
                    call: "GetMixFormat (unsupported sample format)",
                    code: crate::classify::hr::AUDCLNT_E_UNSUPPORTED_FORMAT as i64,
                })
            }
        }
    }

    pub(crate) fn as_ptr(&self) -> *const WAVEFORMATEX {
        self.ptr
    }
}

impl Drop for MixFormat {
    fn drop(&mut self) {
        // SAFETY: allocated by GetMixFormat with CoTaskMemAlloc; freed once.
        unsafe { CoTaskMemFree(Some(self.ptr as *const _)) };
    }
}

/// Tags the client's stream as `AudioCategory_Other` (never `Communications`, which would switch
/// Bluetooth headsets to the hands-free profile). Must be called before `Initialize`; ignored on
/// systems without `IAudioClient2`.
pub(crate) fn set_media_category(client: &IAudioClient) {
    if let Ok(c2) = client.cast::<IAudioClient2>() {
        let props = AudioClientProperties {
            cbSize: std::mem::size_of::<AudioClientProperties>() as u32,
            bIsOffload: false.into(),
            eCategory: AudioCategory_Other,
            Options: AUDCLNT_STREAMOPTIONS_NONE,
        };
        // SAFETY: valid properties struct.
        let _ = unsafe { c2.SetClientProperties(&props) };
    }
}

/// Default engine period of a client in 100 ns units (10 ms when unknown).
pub(crate) fn default_period_hns(client: &IAudioClient) -> i64 {
    let mut def = 0i64;
    // SAFETY: valid out pointer.
    match unsafe { client.GetDevicePeriod(Some(&mut def), None) } {
        Ok(()) if def > 0 => def,
        _ => 100_000,
    }
}

/// Frames in `hns` 100 ns units at `rate`.
pub(crate) fn hns_to_frames(hns: i64, rate: u32) -> u32 {
    ((hns.max(0) as u128 * rate as u128) / 10_000_000) as u32
}
