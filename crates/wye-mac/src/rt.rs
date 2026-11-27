//! Pieces shared by the real-time IOProc processors.

use objc2_core_audio_types::{AudioTimeStamp, AudioTimeStampFlags};

/// Largest block handed to a handler in one call; longer HAL buffers are split.
pub const MAX_FRAMES: usize = 4096;

/// Sample-time continuity tracker for one IOProc direction.
#[derive(Clone, Copy, Debug, Default)]
pub struct Continuity {
    next: f64,
    valid: bool,
}

/// Result of comparing a block's sample time with the expected one.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Gap {
    /// Contiguous (or the first block).
    None,
    /// `n` frames are missing before this block.
    Missing(u64),
    /// The timeline jumped backwards or by an amount that cannot be filled.
    Break,
}

impl Continuity {
    /// Checks the block starting at `ts` with `frames` frames; `max_fill` = largest gap (frames)
    /// that is reported as `Missing` rather than `Break`.
    #[inline]
    pub fn check(&mut self, ts: &AudioTimeStamp, frames: usize, max_fill: f64) -> Gap {
        if !ts.mFlags.contains(AudioTimeStampFlags::SampleTimeValid) {
            self.valid = false;
            return Gap::None;
        }
        let gap = if self.valid {
            let d = ts.mSampleTime - self.next;
            if d.abs() < 0.5 {
                Gap::None
            } else if d > 0.0 && d < max_fill {
                Gap::Missing(d.round() as u64)
            } else {
                Gap::Break
            }
        } else {
            Gap::None
        };
        self.next = ts.mSampleTime + frames as f64;
        self.valid = true;
        gap
    }
}

#[inline]
pub fn rate_scalar(ts: &AudioTimeStamp) -> f64 {
    if ts.mFlags.contains(AudioTimeStampFlags::RateScalarValid) && ts.mRateScalar > 0.0 {
        ts.mRateScalar
    } else {
        1.0
    }
}

#[cfg(test)]
pub mod test_util {
    use objc2_core_audio_types::{
        AudioBuffer, AudioBufferList, AudioTimeStamp, AudioTimeStampFlags,
    };
    use std::ffi::c_void;

    pub fn ts(sample: f64, host: u64) -> AudioTimeStamp {
        // SAFETY: AudioTimeStamp is plain old data; all-zero is a valid value.
        let mut t: AudioTimeStamp = unsafe { std::mem::zeroed() };
        t.mSampleTime = sample;
        t.mHostTime = host;
        t.mRateScalar = 1.0;
        t.mFlags = AudioTimeStampFlags::SampleTimeValid
            | AudioTimeStampFlags::HostTimeValid
            | AudioTimeStampFlags::RateScalarValid;
        t
    }

    /// An AudioBufferList with up to 4 buffers (layout-compatible header + inline array).
    #[repr(C)]
    pub struct Abl4 {
        pub n: u32,
        pub bufs: [AudioBuffer; 4],
    }

    impl Abl4 {
        pub fn new(bufs: &mut [(u32, &mut [f32])]) -> Abl4 {
            let mut a = Abl4 {
                n: bufs.len() as u32,
                bufs: [AudioBuffer {
                    mNumberChannels: 0,
                    mDataByteSize: 0,
                    mData: std::ptr::null_mut(),
                }; 4],
            };
            for (i, (ch, data)) in bufs.iter_mut().enumerate() {
                a.bufs[i] = AudioBuffer {
                    mNumberChannels: *ch,
                    mDataByteSize: (data.len() * 4) as u32,
                    mData: data.as_mut_ptr() as *mut c_void,
                };
            }
            a
        }

        pub fn ptr(&mut self) -> *mut AudioBufferList {
            self as *mut Abl4 as *mut AudioBufferList
        }
    }
}
