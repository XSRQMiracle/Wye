//! Platform-independent sample format handling: parsing a WAVEFORMATEX(TENSIBLE) description,
//! picking the front-left/front-right channels from a channel mask, and converting between the
//! device's interleaved layout and Wye's interleaved stereo `f32`.
//!
//! Everything here is pure, allocation-free and panic-free so it can run on the audio threads and
//! be unit-tested on any host.

/// `WAVE_FORMAT_PCM`.
pub const TAG_PCM: u16 = 1;
/// `WAVE_FORMAT_IEEE_FLOAT`.
pub const TAG_IEEE_FLOAT: u16 = 3;
/// `WAVE_FORMAT_EXTENSIBLE`.
pub const TAG_EXTENSIBLE: u16 = 0xFFFE;

/// `KSDATAFORMAT_SUBTYPE_PCM` as `u128`.
pub const SUBTYPE_PCM: u128 = 0x00000001_0000_0010_8000_00aa00389b71;
/// `KSDATAFORMAT_SUBTYPE_IEEE_FLOAT` as `u128`.
pub const SUBTYPE_IEEE_FLOAT: u128 = 0x00000003_0000_0010_8000_00aa00389b71;

/// `SPEAKER_FRONT_LEFT`.
pub const SPEAKER_FRONT_LEFT: u32 = 0x1;
/// `SPEAKER_FRONT_RIGHT`.
pub const SPEAKER_FRONT_RIGHT: u32 = 0x2;

/// Sample encoding of one channel value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SampleFormat {
    F32,
    I16,
    /// Packed 3-byte little-endian signed integer.
    I24,
    /// 4-byte little-endian signed integer (also used for 24-in-32 left-justified data).
    I32,
}

impl SampleFormat {
    pub fn bytes(self) -> usize {
        match self {
            SampleFormat::F32 | SampleFormat::I32 => 4,
            SampleFormat::I16 => 2,
            SampleFormat::I24 => 3,
        }
    }
}

/// The parts of a `WAVEFORMATEX(TENSIBLE)` that Wye needs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WaveFormat {
    pub rate: u32,
    pub channels: u16,
    /// Bytes per frame (all channels).
    pub block_align: u16,
    pub sample: SampleFormat,
    /// Speaker positions of the channels in order (0 = unknown / not extensible).
    pub channel_mask: u32,
}

/// Raw fields of a `WAVEFORMATEX` plus the extensible tail, if present.
#[derive(Clone, Copy, Debug, Default)]
pub struct RawWaveFormat {
    pub tag: u16,
    pub channels: u16,
    pub rate: u32,
    pub block_align: u16,
    pub bits: u16,
    /// `(wValidBitsPerSample, dwChannelMask, SubFormat)` when `tag == WAVE_FORMAT_EXTENSIBLE`.
    pub extensible: Option<(u16, u32, u128)>,
}

impl WaveFormat {
    /// Interprets the raw fields. Returns `None` for encodings Wye does not handle
    /// (compressed formats, 8-bit PCM, inconsistent block alignment).
    pub fn parse(raw: &RawWaveFormat) -> Option<WaveFormat> {
        let (is_float, mask) = match (raw.tag, raw.extensible) {
            (TAG_IEEE_FLOAT, _) => (true, 0),
            (TAG_PCM, _) => (false, 0),
            (TAG_EXTENSIBLE, Some((_, mask, sub))) if sub == SUBTYPE_IEEE_FLOAT => (true, mask),
            (TAG_EXTENSIBLE, Some((_, mask, sub))) if sub == SUBTYPE_PCM => (false, mask),
            _ => return None,
        };
        let sample = match (is_float, raw.bits) {
            (true, 32) => SampleFormat::F32,
            (false, 16) => SampleFormat::I16,
            (false, 24) => SampleFormat::I24,
            (false, 32) => SampleFormat::I32,
            _ => return None,
        };
        if raw.channels == 0 || raw.rate == 0 {
            return None;
        }
        let expected = sample.bytes() * raw.channels as usize;
        if raw.block_align as usize != expected {
            return None;
        }
        Some(WaveFormat {
            rate: raw.rate,
            channels: raw.channels,
            block_align: raw.block_align,
            sample,
            channel_mask: mask,
        })
    }
}

/// Which device channels carry Wye's left and right signal.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StereoPick {
    pub left: usize,
    pub right: usize,
}

impl StereoPick {
    /// Front-left / front-right channel indices for a device with `channels` channels whose
    /// speaker positions are given by `mask` (channels appear in ascending mask-bit order).
    /// Mono devices map both sides to channel 0; devices without a usable mask use 0 and 1.
    pub fn from_mask(channels: u16, mask: u32) -> StereoPick {
        let n = channels as usize;
        if n <= 1 {
            return StereoPick { left: 0, right: 0 };
        }
        let index_of = |bit: u32| -> Option<usize> {
            if mask & bit == 0 {
                return None;
            }
            let idx = (mask & (bit - 1)).count_ones() as usize;
            (idx < n).then_some(idx)
        };
        match (index_of(SPEAKER_FRONT_LEFT), index_of(SPEAKER_FRONT_RIGHT)) {
            (Some(l), Some(r)) => StereoPick { left: l, right: r },
            _ => StereoPick { left: 0, right: 1 },
        }
    }

    pub fn is_mono(self) -> bool {
        self.left == self.right
    }
}

#[inline]
fn read_sample(bytes: &[u8], at: usize, fmt: SampleFormat) -> f32 {
    match fmt {
        SampleFormat::F32 => match bytes.get(at..at + 4) {
            Some(b) => f32::from_le_bytes([b[0], b[1], b[2], b[3]]),
            None => 0.0,
        },
        SampleFormat::I16 => match bytes.get(at..at + 2) {
            Some(b) => i16::from_le_bytes([b[0], b[1]]) as f32 / 32768.0,
            None => 0.0,
        },
        SampleFormat::I24 => match bytes.get(at..at + 3) {
            Some(b) => {
                let v = i32::from_le_bytes([0, b[0], b[1], b[2]]) >> 8;
                v as f32 / 8_388_608.0
            }
            None => 0.0,
        },
        SampleFormat::I32 => match bytes.get(at..at + 4) {
            Some(b) => {
                (i32::from_le_bytes([b[0], b[1], b[2], b[3]]) as f64 / 2_147_483_648.0) as f32
            }
            None => 0.0,
        },
    }
}

#[inline]
fn write_sample(bytes: &mut [u8], at: usize, fmt: SampleFormat, x: f32) {
    let x = if x.is_finite() {
        x.clamp(-1.0, 1.0)
    } else {
        0.0
    };
    match fmt {
        SampleFormat::F32 => {
            if let Some(b) = bytes.get_mut(at..at + 4) {
                b.copy_from_slice(&x.to_le_bytes());
            }
        }
        SampleFormat::I16 => {
            if let Some(b) = bytes.get_mut(at..at + 2) {
                b.copy_from_slice(&((x * 32767.0) as i16).to_le_bytes());
            }
        }
        SampleFormat::I24 => {
            if let Some(b) = bytes.get_mut(at..at + 3) {
                let v = (x * 8_388_607.0) as i32;
                let le = v.to_le_bytes();
                b.copy_from_slice(&le[..3]);
            }
        }
        SampleFormat::I32 => {
            if let Some(b) = bytes.get_mut(at..at + 4) {
                let v = (x as f64 * 2_147_483_647.0) as i32;
                b.copy_from_slice(&v.to_le_bytes());
            }
        }
    }
}

/// Converts `frames` device frames from `src` into interleaved stereo in `dst`.
/// Returns the number of frames converted (bounded by both buffers).
pub fn decode_to_stereo(
    src: &[u8],
    frames: usize,
    fmt: &WaveFormat,
    pick: StereoPick,
    dst: &mut [f32],
) -> usize {
    let block = fmt.block_align as usize;
    if block == 0 {
        return 0;
    }
    let n = frames.min(src.len() / block).min(dst.len() / 2);
    let sb = fmt.sample.bytes();
    for (f, out) in dst.chunks_exact_mut(2).take(n).enumerate() {
        let base = f * block;
        out[0] = read_sample(src, base + pick.left * sb, fmt.sample);
        out[1] = read_sample(src, base + pick.right * sb, fmt.sample);
    }
    n
}

/// Extracts one channel of `frames` device frames from `src` into `dst` (mono).
pub fn decode_channel(
    src: &[u8],
    frames: usize,
    fmt: &WaveFormat,
    channel: usize,
    dst: &mut [f32],
) -> usize {
    let block = fmt.block_align as usize;
    if block == 0 || channel >= fmt.channels as usize {
        return 0;
    }
    let n = frames.min(src.len() / block).min(dst.len());
    let sb = fmt.sample.bytes();
    for (f, out) in dst.iter_mut().take(n).enumerate() {
        *out = read_sample(src, f * block + channel * sb, fmt.sample);
    }
    n
}

/// Writes interleaved stereo `src` into the device buffer `dst` (`frames` device frames).
/// Left/right go to the picked channels (mono devices get the average); every other channel is
/// silent. Returns the number of frames written.
pub fn encode_from_stereo(
    src: &[f32],
    frames: usize,
    fmt: &WaveFormat,
    pick: StereoPick,
    dst: &mut [u8],
) -> usize {
    let block = fmt.block_align as usize;
    if block == 0 {
        return 0;
    }
    let n = frames.min(dst.len() / block).min(src.len() / 2);
    let sb = fmt.sample.bytes();
    if let Some(region) = dst.get_mut(..n * block) {
        region.fill(0);
    }
    for (f, lr) in src.chunks_exact(2).take(n).enumerate() {
        let base = f * block;
        if pick.is_mono() {
            write_sample(
                dst,
                base + pick.left * sb,
                fmt.sample,
                0.5 * (lr[0] + lr[1]),
            );
        } else {
            write_sample(dst, base + pick.left * sb, fmt.sample, lr[0]);
            write_sample(dst, base + pick.right * sb, fmt.sample, lr[1]);
        }
    }
    n
}

#[cfg(test)]
mod tests {
    use super::*;

    fn float_ext(channels: u16, mask: u32) -> RawWaveFormat {
        RawWaveFormat {
            tag: TAG_EXTENSIBLE,
            channels,
            rate: 48_000,
            block_align: 4 * channels,
            bits: 32,
            extensible: Some((32, mask, SUBTYPE_IEEE_FLOAT)),
        }
    }

    #[test]
    fn parses_common_mix_formats() {
        let f = WaveFormat::parse(&float_ext(2, 0x3)).unwrap();
        assert_eq!(f.sample, SampleFormat::F32);
        assert_eq!(f.channel_mask, 0x3);
        let pcm16 = RawWaveFormat {
            tag: TAG_PCM,
            channels: 2,
            rate: 44_100,
            block_align: 4,
            bits: 16,
            extensible: None,
        };
        assert_eq!(WaveFormat::parse(&pcm16).unwrap().sample, SampleFormat::I16);
        let pcm24 = RawWaveFormat {
            tag: TAG_EXTENSIBLE,
            channels: 2,
            rate: 96_000,
            block_align: 6,
            bits: 24,
            extensible: Some((24, 0x3, SUBTYPE_PCM)),
        };
        assert_eq!(WaveFormat::parse(&pcm24).unwrap().sample, SampleFormat::I24);
    }

    #[test]
    fn rejects_unsupported_formats() {
        let mut bad = float_ext(2, 0x3);
        bad.extensible = Some((32, 0x3, 0x1234));
        assert!(WaveFormat::parse(&bad).is_none());
        let mut misaligned = float_ext(2, 0x3);
        misaligned.block_align = 6;
        assert!(WaveFormat::parse(&misaligned).is_none());
        let pcm8 = RawWaveFormat {
            tag: TAG_PCM,
            channels: 1,
            rate: 8000,
            block_align: 1,
            bits: 8,
            extensible: None,
        };
        assert!(WaveFormat::parse(&pcm8).is_none());
    }

    #[test]
    fn picks_front_channels_by_mask() {
        // 5.1: FL FR FC LFE BL BR
        assert_eq!(
            StereoPick::from_mask(6, 0x3F),
            StereoPick { left: 0, right: 1 }
        );
        // Mask with FC first (FC = 0x4 is after FL/FR in bit order, so still 0/1).
        assert_eq!(
            StereoPick::from_mask(3, 0x7),
            StereoPick { left: 0, right: 1 }
        );
        // Exotic: FR missing → fallback to 0/1.
        assert_eq!(
            StereoPick::from_mask(2, 0x1 | 0x4),
            StereoPick { left: 0, right: 1 }
        );
        // Mono.
        assert!(StereoPick::from_mask(1, 0x4).is_mono());
        // No mask.
        assert_eq!(
            StereoPick::from_mask(8, 0),
            StereoPick { left: 0, right: 1 }
        );
    }

    #[test]
    fn decodes_multichannel_float_to_stereo() {
        let fmt = WaveFormat::parse(&float_ext(6, 0x3F)).unwrap();
        let mut src = Vec::new();
        for f in 0..3 {
            for c in 0..6 {
                src.extend_from_slice(&((f * 10 + c) as f32).to_le_bytes());
            }
        }
        let mut dst = [0.0f32; 6];
        let n = decode_to_stereo(&src, 3, &fmt, StereoPick::from_mask(6, 0x3F), &mut dst);
        assert_eq!(n, 3);
        assert_eq!(dst, [0.0, 1.0, 10.0, 11.0, 20.0, 21.0]);
    }

    #[test]
    fn decode_is_bounded_by_buffers() {
        let fmt = WaveFormat::parse(&float_ext(2, 0x3)).unwrap();
        let src = [0u8; 8 * 10];
        let mut dst = [0.0f32; 4];
        assert_eq!(
            decode_to_stereo(&src, 10, &fmt, StereoPick { left: 0, right: 1 }, &mut dst),
            2
        );
        assert_eq!(
            decode_to_stereo(
                &src[..12],
                10,
                &fmt,
                StereoPick { left: 0, right: 1 },
                &mut dst
            ),
            1
        );
    }

    #[test]
    fn integer_roundtrip() {
        for (sample, bits) in [
            (SampleFormat::I16, 16),
            (SampleFormat::I24, 24),
            (SampleFormat::I32, 32),
        ] {
            let raw = RawWaveFormat {
                tag: TAG_PCM,
                channels: 2,
                rate: 48_000,
                block_align: (sample.bytes() * 2) as u16,
                bits,
                extensible: None,
            };
            let fmt = WaveFormat::parse(&raw).unwrap();
            let pick = StereoPick { left: 0, right: 1 };
            let src = [0.5f32, -0.25, 1.5, -2.0];
            let mut bytes = vec![0u8; fmt.block_align as usize * 2];
            assert_eq!(encode_from_stereo(&src, 2, &fmt, pick, &mut bytes), 2);
            let mut back = [0.0f32; 4];
            decode_to_stereo(&bytes, 2, &fmt, pick, &mut back);
            let expect = [0.5f32, -0.25, 1.0, -1.0];
            for (a, b) in back.iter().zip(expect) {
                assert!((a - b).abs() < 1e-3, "{sample:?}: {a} vs {b}");
            }
        }
    }

    #[test]
    fn encode_maps_to_front_channels_and_silences_others() {
        let fmt = WaveFormat::parse(&float_ext(4, 0x33)).unwrap(); // FL FR BL BR
        let mut bytes = vec![0xFFu8; 16];
        encode_from_stereo(
            &[0.25, -0.5],
            1,
            &fmt,
            StereoPick::from_mask(4, 0x33),
            &mut bytes,
        );
        let vals: Vec<f32> = bytes
            .chunks_exact(4)
            .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
            .collect();
        assert_eq!(vals, [0.25, -0.5, 0.0, 0.0]);
    }

    #[test]
    fn encode_mono_device_averages() {
        let fmt = WaveFormat::parse(&float_ext(1, 0x4)).unwrap();
        let mut bytes = [0u8; 4];
        encode_from_stereo(
            &[0.5, 0.25],
            1,
            &fmt,
            StereoPick::from_mask(1, 0x4),
            &mut bytes,
        );
        assert_eq!(f32::from_le_bytes(bytes), 0.375);
    }

    #[test]
    fn decode_single_channel() {
        let fmt = WaveFormat::parse(&float_ext(2, 0x3)).unwrap();
        let mut src = Vec::new();
        for v in [1.0f32, 2.0, 3.0, 4.0] {
            src.extend_from_slice(&v.to_le_bytes());
        }
        let mut dst = [0.0f32; 2];
        assert_eq!(decode_channel(&src, 2, &fmt, 0, &mut dst), 2);
        assert_eq!(dst, [1.0, 3.0]);
        assert_eq!(decode_channel(&src, 2, &fmt, 5, &mut dst), 0);
    }
}
