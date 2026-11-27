//! macOS adapter: Core Audio HAL IOProcs + Process Tap (macOS 14.4+).
#![cfg_attr(not(target_os = "macos"), allow(unused))]

#[cfg(target_os = "macos")]
pub struct MacPlatform {
    _private: (),
}

#[cfg(target_os = "macos")]
impl MacPlatform {
    pub fn new() -> Result<Self, wye_platform::PlatformError> {
        todo!("macOS adapter")
    }
}
