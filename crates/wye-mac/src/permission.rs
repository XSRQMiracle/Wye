//! Privacy permissions (system audio recording, microphone) and System Settings deep links.

use objc2::msg_send;
use objc2::runtime::AnyClass;
use objc2_foundation::NSString;
use wye_platform::{PermissionState, PlatformError};

#[link(name = "AVFoundation", kind = "framework")]
extern "C" {}

pub const AUDIO_CAPTURE_SETTINGS: &str =
    "x-apple.systempreferences:com.apple.preference.security?Privacy_AudioCapture";
pub const MICROPHONE_SETTINGS: &str =
    "x-apple.systempreferences:com.apple.preference.security?Privacy_Microphone";
pub const SOUND_SETTINGS: &str = "x-apple.systempreferences:com.apple.preference.sound";

/// `AVAuthorizationStatus` → `PermissionState`.
pub fn map_av_status(status: isize) -> PermissionState {
    match status {
        0 => PermissionState::NotDetermined,
        1 | 2 => PermissionState::Denied, // restricted, denied
        3 => PermissionState::Granted,
        _ => PermissionState::Unknown,
    }
}

/// `+[AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeAudio]`.
pub fn microphone_permission() -> PermissionState {
    let Some(cls) = AnyClass::get(c"AVCaptureDevice") else {
        return PermissionState::Unknown;
    };
    // AVMediaTypeAudio is the constant string "soun".
    let media = NSString::from_str("soun");
    // SAFETY: documented class method taking an NSString and returning NSInteger.
    let status: isize = unsafe { msg_send![cls, authorizationStatusForMediaType: &*media] };
    map_av_status(status)
}

/// Opens a System Settings deep link with `open(1)` (non-blocking).
pub fn open_url(url: &str) -> Result<(), PlatformError> {
    std::process::Command::new("/usr/bin/open")
        .arg(url)
        .spawn()
        .map(|mut child| {
            // Reap the short-lived `open` process off the caller's thread.
            std::thread::spawn(move || {
                let _ = child.wait();
            });
        })
        .map_err(|e| PlatformError::Os {
            call: "open",
            code: e.raw_os_error().unwrap_or(-1) as i64,
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn av_status_mapping() {
        assert_eq!(map_av_status(0), PermissionState::NotDetermined);
        assert_eq!(map_av_status(1), PermissionState::Denied);
        assert_eq!(map_av_status(2), PermissionState::Denied);
        assert_eq!(map_av_status(3), PermissionState::Granted);
        assert_eq!(map_av_status(9), PermissionState::Unknown);
    }

    #[test]
    fn microphone_permission_does_not_crash() {
        // Only reads the status; never prompts.
        let _ = microphone_permission();
    }
}
