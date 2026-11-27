//! Persisted settings (JSON). Loaded/saved on the control thread only.

use crate::align::AlignPolicy;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub const SETTINGS_VERSION: u32 = 1;
pub const APP_ID: &str = "io.github.xsrqmiracle.wye";

/// Settings file location shared by the app and the CLI (no Tauri dependency):
/// macOS `~/Library/Application Support/<APP_ID>/settings.json`,
/// Windows `%APPDATA%\\<APP_ID>\\settings.json`.
pub fn default_path() -> Option<std::path::PathBuf> {
    #[cfg(target_os = "macos")]
    {
        std::env::var_os("HOME").map(|h| {
            std::path::PathBuf::from(h)
                .join("Library/Application Support")
                .join(APP_ID)
                .join("settings.json")
        })
    }
    #[cfg(windows)]
    {
        std::env::var_os("APPDATA").map(|a| {
            std::path::PathBuf::from(a)
                .join(APP_ID)
                .join("settings.json")
        })
    }
    #[cfg(not(any(target_os = "macos", windows)))]
    {
        std::env::var_os("HOME").map(|h| {
            std::path::PathBuf::from(h)
                .join(".config")
                .join(APP_ID)
                .join("settings.json")
        })
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    pub version: u32,
    /// Keyed by `DeviceId.0` (Core Audio UID / Windows endpoint ID).
    pub devices: BTreeMap<String, DeviceSettings>,
    pub policy: AlignPolicy,
    pub launch_at_login: bool,
    /// "zh-Hans" | "en" | "system".
    pub language: String,
    pub show_advanced: bool,
    /// sinc256 instead of sinc128.
    pub high_quality: bool,
    /// Show transport = Virtual devices (BlackHole etc.).
    pub show_virtual: bool,
    /// Multiply mirror gains with the default device's system volume.
    pub follow_system_volume: bool,
    pub show_debug: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct DeviceSettings {
    /// Last known display name (for showing remembered-but-absent devices).
    pub name: String,
    pub enabled: bool,
    /// 0.0 ..= 1.0 slider position (gain = position³).
    pub volume: f32,
    /// User fine-tune, milliseconds, positive = later.
    pub offset_ms: f32,
    /// Result of acoustic calibration: measured output-timestamp → acoustic latency (ms), if any.
    pub calibrated_latency_ms: Option<f32>,
    /// Reported latency at calibration time; calibration is discarded when it changes by > 5 ms.
    pub calibrated_reported_ms: Option<f32>,
    /// Calibration done < 30 min after a Bluetooth connection (warm-up drift).
    pub calibration_provisional: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            version: SETTINGS_VERSION,
            devices: BTreeMap::new(),
            policy: AlignPolicy::Auto,
            launch_at_login: false,
            language: "system".into(),
            show_advanced: false,
            high_quality: false,
            show_virtual: false,
            follow_system_volume: true,
            show_debug: false,
        }
    }
}

impl Default for DeviceSettings {
    fn default() -> Self {
        DeviceSettings {
            name: String::new(),
            enabled: false,
            volume: 1.0,
            offset_ms: 0.0,
            calibrated_latency_ms: None,
            calibrated_reported_ms: None,
            calibration_provisional: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn roundtrip_and_forward_compat() {
        let mut s = Settings::default();
        s.devices.insert(
            "BuiltInSpeakerDevice".into(),
            DeviceSettings {
                name: "MacBook Air 扬声器".into(),
                enabled: true,
                volume: 0.8,
                offset_ms: -3.5,
                ..Default::default()
            },
        );
        let j = serde_json::to_string_pretty(&s).unwrap();
        let back: Settings = serde_json::from_str(&j).unwrap();
        assert_eq!(s, back);
        // unknown fields are ignored, missing fields defaulted
        let old: Settings =
            serde_json::from_str(r#"{"version":1,"devices":{"x":{"enabled":true}},"future":1}"#)
                .unwrap();
        assert_eq!(old.devices["x"].volume, 1.0);
    }
}
