//! Capture anchor: the latest (frame index, host time) pair published by the capture callback and
//! read by every render callback. Seqlock: the writer never waits, readers retry a bounded
//! number of times. Real-time safe on both sides.

use std::sync::atomic::{fence, AtomicU64, Ordering};

#[derive(Default)]
pub struct Anchor {
    seq: AtomicU64,
    frame: AtomicU64,
    host_ns: AtomicU64,
}

impl Anchor {
    pub const fn new() -> Self {
        Anchor {
            seq: AtomicU64::new(0),
            frame: AtomicU64::new(0),
            host_ns: AtomicU64::new(0),
        }
    }

    /// Single writer (capture callback): frame index *after* the last written frame, and the host
    /// time at which that frame boundary was captured.
    pub fn publish(&self, frame: u64, host_ns: u64) {
        let s = self.seq.load(Ordering::Relaxed);
        self.seq.store(s.wrapping_add(1), Ordering::Relaxed);
        fence(Ordering::Release);
        self.frame.store(frame, Ordering::Relaxed);
        self.host_ns.store(host_ns, Ordering::Relaxed);
        self.seq.store(s.wrapping_add(2), Ordering::Release);
    }

    /// Returns `None` until the first publish, or if 8 attempts raced with the writer.
    pub fn read(&self) -> Option<(u64, u64)> {
        for _ in 0..8 {
            let s1 = self.seq.load(Ordering::Acquire);
            if s1 & 1 == 1 {
                std::hint::spin_loop();
                continue;
            }
            let f = self.frame.load(Ordering::Relaxed);
            let t = self.host_ns.load(Ordering::Relaxed);
            fence(Ordering::Acquire);
            if self.seq.load(Ordering::Relaxed) == s1 {
                return if s1 == 0 { None } else { Some((f, t)) };
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn publish_read() {
        let a = Anchor::new();
        assert_eq!(a.read(), None);
        a.publish(512, 1_000);
        assert_eq!(a.read(), Some((512, 1_000)));
    }
    #[test]
    fn concurrent_consistency() {
        let a = std::sync::Arc::new(Anchor::new());
        let w = a.clone();
        let h = std::thread::spawn(move || {
            for i in 1..200_000u64 {
                w.publish(i, i * 3);
            }
        });
        for _ in 0..200_000 {
            if let Some((f, t)) = a.read() {
                assert_eq!(t, f * 3);
            }
        }
        h.join().unwrap();
    }
}
