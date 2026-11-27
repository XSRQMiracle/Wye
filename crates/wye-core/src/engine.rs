//! Control-thread engine: owns the platform, the device registry, settings, the alignment plan
//! and every stream. UI and CLI talk to it only through [`EngineCommand`] and [`EngineSnapshot`].
//!
//! Serialization contract: every enum is `#[serde(tag = "kind", rename_all =
//! "snake_case")]`, durations are f64 milliseconds, `DeviceId` is a plain string.

use crate::align::{AlignPolicy, Mode};
use serde::Serialize;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Instant;
use wye_platform::{DeviceId, Platform, TimestampQuality, Transport};

#[derive(Clone, Debug, PartialEq, serde::Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum EngineCommand {
    SetEnabled {
        id: DeviceId,
        on: bool,
    },
    /// Slider position 0..=1.
    SetVolume {
        id: DeviceId,
        volume: f32,
    },
    /// Milliseconds, positive = later.
    SetOffset {
        id: DeviceId,
        ms: f32,
    },
    SetPolicy {
        policy: AlignPolicy,
    },
    /// Ask the OS to make this the default output (button in a notice).
    MakeDefault {
        id: DeviceId,
    },
    StartCalibration {
        id: DeviceId,
    },
    /// Calibrate every checked device in one session (reference first).
    CalibrateAll,
    CancelCalibration,
    ClearCalibration {
        id: DeviceId,
    },
    /// Play/stop the click train used for manual alignment.
    AlignmentClicks {
        on: bool,
    },
    SetPaused {
        paused: bool,
    },
    SetHighQuality {
        on: bool,
    },
    SetShowVirtual {
        on: bool,
    },
    SetFollowSystemVolume {
        on: bool,
    },
    /// Persisted setting: show the debug section.
    SetShowDebug {
        on: bool,
    },
    /// Transient: stream debug statistics at 4 Hz (only while the panel is visible).
    SetDebugStreaming {
        on: bool,
    },
    /// Remove a remembered (absent) device from the settings.
    ForgetDevice {
        id: DeviceId,
    },
    OpenMicSettings,
    OpenSoundSettings,
    SetLanguage {
        tag: String,
    },
    OpenPermissionSettings,
    /// Re-enumerate devices and re-plan (also done automatically on platform events).
    Refresh,
    Shutdown,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum EngineStatus {
    /// Nothing for Wye to render: no capture running (zero CPU).
    Idle,
    Paused,
    /// Capture is being opened (may wait for the TCC prompt).
    Starting,
    NeedsPermission,
    Running {
        mode: Mode,
    },
    Recovering {
        attempt: u32,
    },
    /// Unrecoverable for now; `reason` is an i18n key suffix (e.g. "os_too_old", "rdp").
    Failed {
        reason: &'static str,
    },
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum DeviceState {
    Off,
    /// Played natively by the system (the default device in reference mode).
    SystemPlays,
    Starting,
    Playing,
    /// Present in settings but not connected.
    Absent,
    /// Bluetooth headset in hands-free mode.
    HandsFree,
    /// Held exclusively by another application.
    Busy,
    Error {
        message: String,
    },
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct DeviceDebug {
    pub lambda_ms: f64,
    pub target_ms: f64,
    pub error_ms: f64,
    pub delta_ppm: f64,
    pub buffered_ms: f64,
    pub underruns: u64,
    pub resyncs: u64,
    pub overflows: u64,
    pub saturated: u64,
    pub rate_in: f64,
    pub rate_out: f64,
    pub block_frames: u32,
    pub reported_latency_ms: f64,
    pub calibrated_latency_ms: Option<f64>,
    pub b_min_ms: f64,
    pub jitter_ms: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct CaptureDebug {
    pub mode: Option<Mode>,
    pub rate: f64,
    pub callbacks: u64,
    pub peak_dbfs: f64,
    pub zero_secs: f64,
    pub quality: &'static str,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct DeviceView {
    pub id: DeviceId,
    pub name: String,
    pub transport: Transport,
    pub is_default: bool,
    pub enabled: bool,
    /// Slider position 0..=1; `None` when the system controls the volume (reference default).
    pub volume: Option<f32>,
    pub offset_ms: f32,
    pub calibrated: bool,
    pub needs_calibration: bool,
    /// after-timestamp latency > 500 ms (AirPlay, some TVs).
    pub high_latency: bool,
    pub state: DeviceState,
    /// > 0 when the device cannot be aligned and plays this much late.
    pub late_by_ms: f64,
    pub debug: Option<DeviceDebug>,
}

/// User-facing notices; the GUI maps `notice.<kind>` to localized text.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Notice {
    PermissionNeeded,
    /// Permission changed; macOS applies it only after a relaunch.
    RestartRequired,
    MicPermissionNeeded,
    /// Capture open has not returned after 3 s (TCC prompt pending or another tap blocking).
    CaptureSlow,
    MirrorLate {
        device: DeviceId,
        name: String,
        ms: f64,
    },
    LipSync {
        ms: f64,
        suggest: DeviceId,
        suggest_name: String,
    },
    HandsFree {
        device: DeviceId,
        name: String,
        app: Option<String>,
    },
    DeviceDisconnected {
        name: String,
    },
    /// The previous default device keeps playing, now mirrored by Wye.
    FormerDefaultMirrored {
        name: String,
    },
    CannotMuteDefault {
        device: DeviceId,
        name: String,
    },
    ClockOutOfRange {
        device: DeviceId,
        name: String,
    },
    RecalibrateSuggested {
        device: DeviceId,
        name: String,
    },
    TooManyDevices {
        max: u8,
    },
    CalibrationProgress {
        device: DeviceId,
        name: String,
        step: u8,
        of: u8,
    },
    CalibrationDone {
        device: DeviceId,
        name: String,
        latency_ms: f64,
    },
    /// reason ∈ "too_quiet" | "inconsistent" | "no_mic" | "mic_denied" | "cancelled"
    CalibrationFailed {
        device: DeviceId,
        name: String,
        reason: &'static str,
    },
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct EngineSnapshot {
    pub status: EngineStatus,
    pub devices: Vec<DeviceView>,
    pub notices: Vec<Notice>,
    pub policy: AlignPolicy,
    pub capture: Option<CaptureDebug>,
}

pub type SnapshotSink = Box<dyn Fn(EngineSnapshot) + Send + Sync + 'static>;

/// Time source for every engine timer (debounce, backoff, health checks). Tests inject a
/// manual clock.
pub trait Clock: Send + Sync {
    fn now(&self) -> Instant;
}

pub struct SystemClock;
impl Clock for SystemClock {
    fn now(&self) -> Instant {
        Instant::now()
    }
}

pub struct EngineOptions {
    pub settings_path: PathBuf,
    pub clock: Arc<dyn Clock>,
    /// When `Some`, the engine does not run its own 10 Hz ticker; each `()` received is one
    /// tick (tests drive it manually).
    pub manual_tick: Option<std::sync::mpsc::Receiver<()>>,
}

pub struct Engine {
    _private: (),
}

impl Engine {
    /// Spawns the control thread "wye-engine". `on_change` is called on that thread whenever the
    /// snapshot changes (≤ 10 Hz; debug fields 4 Hz while debug is on).
    pub fn start(
        _platform: Arc<dyn Platform>,
        _opts: EngineOptions,
        _on_change: SnapshotSink,
    ) -> Result<Engine, crate::Error> {
        todo!("engine")
    }
    pub fn send(&self, _cmd: EngineCommand) {
        todo!()
    }
    pub fn snapshot(&self) -> EngineSnapshot {
        todo!()
    }
}

/// Helper used by the GUI: quality label for the debug panel.
pub fn quality_label(q: TimestampQuality) -> &'static str {
    match q {
        TimestampQuality::Device => "device",
        TimestampQuality::CallbackClock => "callback",
    }
}
