//! Windows adapter: WASAPI loopback capture + shared-mode event-driven render.
#![cfg_attr(not(windows), allow(unused))]

#[cfg(windows)]
pub struct WinPlatform {
    _private: (),
}

#[cfg(windows)]
impl WinPlatform {
    pub fn new() -> Result<Self, wye_platform::PlatformError> {
        todo!("Windows adapter")
    }
}
