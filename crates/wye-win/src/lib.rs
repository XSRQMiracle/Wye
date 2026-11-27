//! Windows adapter for Wye: WASAPI endpoint loopback capture (with a silent keep-alive stream
//! and synthesized silence while the system is quiet) and per-device event-driven shared-mode
//! render streams.
//!
//! Every stream runs on its own thread that joins the multithreaded COM apartment, registers with
//! MMCSS ("Pro Audio"), creates its COM objects on that thread (windows-rs COM interfaces are not
//! `Send`) and then loops on `WaitForMultipleObjects([stop, …, audio])`, calling the Wye handler
//! once per wake-up — the same callback model as a Core Audio IOProc.
//!
//! On other targets the crate compiles to nothing (only the platform-independent helpers are built
//! for unit tests).

#![cfg_attr(not(windows), allow(dead_code, unused))]

#[cfg(any(windows, test))]
mod classify;
#[cfg(any(windows, test))]
mod format;
#[cfg(any(windows, test))]
mod timing;

#[cfg(test)]
mod no_alloc_tests;

#[cfg(windows)]
mod capture;
#[cfg(windows)]
mod com;
#[cfg(windows)]
mod devices;
#[cfg(windows)]
mod hfp;
#[cfg(windows)]
mod input;
#[cfg(windows)]
mod platform;
#[cfg(windows)]
mod render;
#[cfg(windows)]
mod watch;

#[cfg(windows)]
pub use platform::WinPlatform;
