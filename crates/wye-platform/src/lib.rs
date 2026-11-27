//! Platform abstraction for Wye.
//!
//! This crate is the **only** contract between the platform-independent core (`wye-core`) and the
//! OS adapters (`wye-mac`, `wye-win`). It has no dependencies besides `thiserror`.
//!
//! Real-time rules: every method of [`CaptureHandler`] and
//! [`RenderHandler`] is called on an OS real-time audio thread. Implementations must not allocate,
//! free, lock, block, log, or perform I/O. Adapters must call them with pre-allocated buffers and
//! must never hold a lock while calling them.

#![forbid(unsafe_op_in_unsafe_fn)]

use std::fmt;
use std::time::Duration;

/// Monotonic host time in nanoseconds (macOS: `mach_absolute_time` converted with the timebase;
/// Windows: `QueryPerformanceCounter` converted). Same clock for every stream in the process.
pub type HostNanos = u64;

/// Stable, persistable device identifier.
/// macOS: `kAudioDevicePropertyDeviceUID`. Windows: `IMMDevice::GetId` endpoint ID string.
#[derive(Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(transparent)
)]
pub struct DeviceId(pub String);

impl fmt::Debug for DeviceId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "DeviceId({})", self.0)
    }
}
impl fmt::Display for DeviceId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(rename_all = "snake_case")
)]
pub enum Transport {
    BuiltIn,
    Usb,
    Bluetooth,
    BluetoothLe,
    Hdmi,
    DisplayPort,
    AirPlay,
    Thunderbolt,
    Pci,
    Virtual,
    Aggregate,
    Other,
}

impl Transport {
    pub fn is_wireless(self) -> bool {
        matches!(
            self,
            Transport::Bluetooth | Transport::BluetoothLe | Transport::AirPlay
        )
    }
}

/// Operating mode of an output device as far as Wye cares.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(rename_all = "snake_case")
)]
pub enum DeviceMode {
    Normal,
    /// Bluetooth headset switched to the hands-free (HFP/HSP) profile because some app opened its
    /// microphone: low sample rate (8/16/24 kHz), often mono, poor quality.
    BluetoothHandsFree,
}

/// Output latency as reported by the OS, split at the stream timestamp.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct ReportedLatency {
    /// From the render timestamp passed in [`RenderInfo::host_time`] to sound leaving the device.
    /// macOS: (kAudioDevicePropertyLatency + kAudioStreamPropertyLatency) / rate.
    /// Windows: derived from IAudioClock position (+ calibration).
    pub after_timestamp: Duration,
    /// Whole output path from "the app hands over a buffer" to sound (buffer + safety + after_timestamp).
    /// Used only for feasibility estimates and UI.
    pub total: Duration,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DeviceInfo {
    pub id: DeviceId,
    /// Localized human-readable name.
    pub name: String,
    pub transport: Transport,
    /// Current nominal sample rate of the device (Hz).
    pub nominal_rate: f64,
    pub output_channels: u16,
    pub mode: DeviceMode,
    pub reported_latency: ReportedLatency,
    /// Identifier shared by endpoints of the same physical product (Windows ContainerId;
    /// macOS: `None`). Used to merge a Bluetooth headset's stereo and hands-free endpoints.
    pub physical_group: Option<String>,
    /// True if the device can be selected as system default output.
    pub can_be_default: bool,
    /// Current IO buffer size of the device (frames) — gives T_out_block before a stream runs.
    pub buffer_frames: u32,
    /// Another process holds the device exclusively (macOS hog mode / WASAPI exclusive).
    pub busy: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(
    feature = "serde",
    derive(serde::Serialize, serde::Deserialize),
    serde(rename_all = "snake_case")
)]
pub enum PermissionState {
    /// Not asked yet; the first capture start will show the OS prompt.
    NotDetermined,
    Granted,
    Denied,
    /// The platform cannot tell (e.g. macOS without the private TCC preflight call). The adapter
    /// then infers "Denied" from a capture stream that delivers only exact zeros while
    /// [`PlatformEvent::SystemAudioActive`] says something is playing.
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimestampQuality {
    /// Device/driver timestamps (Core Audio AudioTimeStamp, WASAPI QPC positions).
    Device,
    /// Only the time the callback was invoked is known.
    CallbackClock,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StreamFormat {
    /// Rate at which the callbacks deliver/consume frames (Hz).
    pub rate: f64,
    /// Typical frames per callback (informational; callbacks may vary).
    pub block_frames: u32,
}

/// Passed with every capture callback.
#[derive(Clone, Copy, Debug)]
pub struct CaptureInfo {
    /// Host time at which the first frame of this block was captured (rendered by the system mix).
    pub host_time: HostNanos,
    pub quality: TimestampQuality,
    /// The stream timeline broke before this block (frames were lost or the device restarted).
    /// Adapters that can count the lost frames must instead insert silence and leave this false.
    pub discontinuity: bool,
}

/// Passed with every render callback.
#[derive(Clone, Copy, Debug)]
pub struct RenderInfo {
    /// Host time at which the first frame of this buffer reaches the device's timestamp point
    /// (macOS: `inOutputTime.mHostTime`). Add [`ReportedLatency::after_timestamp`] (or the
    /// calibrated value) to get the acoustic time.
    pub host_time: HostNanos,
    pub quality: TimestampQuality,
    /// The device timeline broke since the previous callback (overload, skipped cycle, restart).
    pub discontinuity: bool,
    /// Current reported latency after the timestamp, in case it changed at runtime.
    pub after_timestamp: Duration,
}

/// Real-time capture callback. `stereo` is interleaved L/R f32, already converted from the
/// device layout by the adapter (pre-allocated scratch). Length = frames × 2.
pub trait CaptureHandler: Send + 'static {
    fn on_capture(&mut self, stereo: &[f32], info: &CaptureInfo);
}

/// Real-time render callback. The handler fills `stereo` (interleaved L/R f32, frames × 2,
/// pre-zeroed by the adapter); the adapter maps it to the device's channels.
pub trait RenderHandler: Send + 'static {
    fn on_render(&mut self, stereo: &mut [f32], info: &RenderInfo);
}

/// Real-time input callback for calibration recording (mono, first channel).
pub trait InputHandler: Send + 'static {
    fn on_input(&mut self, mono: &[f32], info: &CaptureInfo);
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CaptureConfig {
    /// Mute the captured audio on the system output while Wye reads it (macOS
    /// `CATapMutedWhenTapped`; muting ends automatically when Wye stops or crashes). Requires
    /// [`Capabilities::can_mute_capture`]. This is what "takeover mode" uses.
    pub mute_original: bool,
    /// Preferred frames per callback (adapters choose the nearest supported).
    pub preferred_block_frames: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OutputConfig {
    pub preferred_block_frames: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Capabilities {
    /// Can capture the system mix while muting it on the original device (takeover mode).
    pub can_mute_capture: bool,
    /// Can change the system default output device.
    pub can_set_default_output: bool,
    /// Render/capture callbacks carry device timestamps.
    pub device_timestamps: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub enum PlatformEvent {
    /// Output device list changed (added/removed/renamed). Re-enumerate.
    DevicesChanged,
    /// System default output device changed.
    DefaultOutputChanged(Option<DeviceId>),
    /// Nominal rate / channel layout / latency of a device changed; reopen its stream.
    DeviceFormatChanged(DeviceId),
    /// Bluetooth profile switched (A2DP ⇄ HFP). `occupant` = display name of the app holding the
    /// headset microphone, if known.
    DeviceModeChanged {
        id: DeviceId,
        mode: DeviceMode,
        occupant: Option<String>,
    },
    /// System volume / mute of an output device changed (only emitted for the default device).
    VolumeChanged {
        id: DeviceId,
        scalar: Option<f32>,
        muted: Option<bool>,
    },
    /// Whether any process is currently producing audio (best effort; used for diagnostics).
    SystemAudioActive(bool),
    /// A running stream failed (device unplugged, invalidated, IO stopped abnormally).
    StreamFailed {
        stream: StreamKind,
        error: StreamError,
    },
    /// Capture permission changed while running (macOS applies it only after a relaunch).
    PermissionChanged(PermissionState),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum StreamKind {
    Capture,
    Output(DeviceId),
    Input(DeviceId),
}

#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum StreamError {
    #[error("device disappeared")]
    DeviceGone,
    #[error("device format changed")]
    FormatChanged,
    #[error("device in use exclusively by another application")]
    DeviceBusy,
    #[error("OS error {0}")]
    Os(i64),
}

#[derive(Debug, thiserror::Error)]
pub enum PlatformError {
    #[error("device not found: {0}")]
    DeviceNotFound(DeviceId),
    #[error("system audio capture permission denied")]
    PermissionDenied,
    #[error("operating system version not supported: {0}")]
    UnsupportedOs(String),
    #[error("not supported on this platform: {0}")]
    Unsupported(&'static str),
    #[error("device busy: {0}")]
    DeviceBusy(DeviceId),
    #[error("OS call {call} failed with {code}")]
    Os { call: &'static str, code: i64 },
}

/// Running stream. Dropping it stops the stream and releases every OS object it created
/// (macOS: IOProc, aggregate device, process tap). Must be dropped on a non-real-time thread.
pub trait Stream: Send {
    fn format(&self) -> StreamFormat;
}

/// Event callback; called on an adapter-owned non-real-time thread. Must not block for long.
pub type EventSink = Box<dyn Fn(PlatformEvent) + Send + Sync + 'static>;

/// Keeps OS listeners registered; dropping it unregisters them.
pub trait WatchGuard: Send {}

/// The platform adapter. One instance per process, created by the app at startup.
pub trait Platform: Send + Sync + 'static {
    fn capabilities(&self) -> Capabilities;
    /// Output devices usable by Wye (filters out aggregate/virtual helper devices that Wye itself
    /// creates, devices without output channels, and hidden devices).
    fn output_devices(&self) -> Result<Vec<DeviceInfo>, PlatformError>;
    fn default_output(&self) -> Result<Option<DeviceId>, PlatformError>;
    fn set_default_output(&self, id: &DeviceId) -> Result<(), PlatformError>;
    /// Input devices that are safe for calibration (never a Bluetooth headset microphone).
    fn calibration_inputs(&self) -> Result<Vec<DeviceInfo>, PlatformError>;
    fn watch(&self, sink: EventSink) -> Result<Box<dyn WatchGuard>, PlatformError>;
    fn capture_permission(&self) -> PermissionState;
    /// Opens the OS settings page where the user can grant capture permission.
    fn open_permission_settings(&self) -> Result<(), PlatformError>;
    /// Opens the microphone privacy page (calibration).
    fn open_microphone_settings(&self) -> Result<(), PlatformError>;
    /// Opens the system sound settings (used where `set_default_output` is unsupported).
    fn open_sound_settings(&self) -> Result<(), PlatformError>;
    /// Starts capturing the system mix, excluding this process's own output. Blocks until the
    /// permission probe has a verdict (≈2 s after IO starts); returns
    /// `PermissionDenied` when capture is refused.
    fn open_capture(
        &self,
        cfg: CaptureConfig,
        handler: Box<dyn CaptureHandler>,
    ) -> Result<Box<dyn Stream>, PlatformError>;
    fn open_output(
        &self,
        id: &DeviceId,
        cfg: OutputConfig,
        handler: Box<dyn RenderHandler>,
    ) -> Result<Box<dyn Stream>, PlatformError>;
    fn open_input(
        &self,
        id: &DeviceId,
        handler: Box<dyn InputHandler>,
    ) -> Result<Box<dyn Stream>, PlatformError>;
    /// Reported input latency of a calibration microphone (device + stream latency, input scope).
    /// `CaptureInfo::host_time` from `open_input` is the raw device timestamp (not compensated).
    fn input_latency(&self, id: &DeviceId) -> Result<Duration, PlatformError>;
    /// System volume scalar (0..=1) and mute state of an output device, if it has such controls.
    fn output_volume(&self, id: &DeviceId) -> Option<(f32, bool)>;
    /// Microphone permission (calibration only).
    fn microphone_permission(&self) -> PermissionState;
    /// Current host time (same clock as the callback timestamps).
    fn now(&self) -> HostNanos;
}
