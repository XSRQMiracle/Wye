//! Debug harness: system-mix capture → lock-free ring → one output device, plus an optional
//! input stream. Prints callback counts, rates, block sizes, peaks and timestamps every second.
//!
//! Silent by default when the output is a virtual device such as BlackHole:
//! `passthrough --secs 10 --out BlackHole2ch_UID --input BlackHole2ch_UID [--mute] [--block 256]`
//!
//! `passthrough --tone 0.1 --out BlackHole2ch_UID --secs 10` only renders a 1 kHz sine into the
//! output device (no capture); run it in a second process to give the tap something to capture.
//!
//! Capture needs the system-audio-recording permission, so run it from an app bundle that holds
//! the permission (the binary must be the responsible process's child).

#[cfg(target_os = "macos")]
mod harness {
    use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
    use std::sync::Arc;
    use std::time::{Duration, Instant};

    use wye_mac::MacPlatform;
    use wye_platform::{
        CaptureConfig, CaptureHandler, CaptureInfo, DeviceId, InputHandler, OutputConfig, Platform,
        RenderHandler, RenderInfo,
    };

    #[derive(Default)]
    pub struct Stats {
        callbacks: AtomicU64,
        frames: AtomicU64,
        min_block: AtomicU64,
        max_block: AtomicU64,
        peak_bits: AtomicU32,
        first_host: AtomicU64,
        last_host: AtomicU64,
        disc: AtomicU64,
        dropped: AtomicU64,
        after_ns: AtomicU64,
    }

    impl Stats {
        fn new() -> Arc<Stats> {
            let s = Stats::default();
            s.min_block.store(u64::MAX, Ordering::Relaxed);
            Arc::new(s)
        }

        fn record(&self, frames: usize, host: u64, disc: bool, samples: &[f32]) {
            let n = frames as u64;
            if self.callbacks.fetch_add(1, Ordering::Relaxed) == 0 {
                self.first_host.store(host, Ordering::Relaxed);
            }
            self.frames.fetch_add(n, Ordering::Relaxed);
            self.min_block.fetch_min(n, Ordering::Relaxed);
            self.max_block.fetch_max(n, Ordering::Relaxed);
            self.last_host.store(host, Ordering::Relaxed);
            if disc {
                self.disc.fetch_add(1, Ordering::Relaxed);
            }
            let peak = samples.iter().fold(0.0f32, |m, v| m.max(v.abs()));
            self.peak_bits.fetch_max(peak.to_bits(), Ordering::Relaxed);
        }

        fn line(&self, name: &str, prev: &mut (u64, u64), dt: f64, now_ns: u64) -> String {
            let cb = self.callbacks.load(Ordering::Relaxed);
            let fr = self.frames.load(Ordering::Relaxed);
            let first = self.first_host.load(Ordering::Relaxed);
            let last = self.last_host.load(Ordering::Relaxed);
            // Average rate of the device timeline: frames per host second since the first block.
            let span = last.saturating_sub(first) as f64 * 1e-9;
            let s = format!(
                "{name:<7} cb/s={:7.1} frames/s={:8.1} rate(ts)={:9.2} block={}..{} peak={:.4} disc={} dropped={} host-now={:+.2} ms after={:.2} ms",
                (cb - prev.0) as f64 / dt,
                (fr - prev.1) as f64 / dt,
                if span > 0.0 && cb > 1 { (fr as f64 - fr as f64 / cb as f64) / span } else { 0.0 },
                if cb > 0 { self.min_block.load(Ordering::Relaxed) } else { 0 },
                self.max_block.load(Ordering::Relaxed),
                f32::from_bits(self.peak_bits.load(Ordering::Relaxed)),
                self.disc.load(Ordering::Relaxed),
                self.dropped.load(Ordering::Relaxed),
                (last as f64 - now_ns as f64) * 1e-6,
                self.after_ns.load(Ordering::Relaxed) as f64 * 1e-6,
            );
            *prev = (cb, fr);
            s
        }
    }

    struct Cap {
        ring: rtrb::Producer<f32>,
        stats: Arc<Stats>,
    }

    impl CaptureHandler for Cap {
        fn on_capture(&mut self, stereo: &[f32], info: &CaptureInfo) {
            self.stats
                .record(stereo.len() / 2, info.host_time, info.discontinuity, stereo);
            match self.ring.write_chunk_uninit(stereo.len()) {
                Ok(c) => {
                    c.fill_from_iter(stereo.iter().copied());
                }
                Err(_) => {
                    self.stats
                        .dropped
                        .fetch_add((stereo.len() / 2) as u64, Ordering::Relaxed);
                }
            }
        }
    }

    struct Out {
        ring: rtrb::Consumer<f32>,
        primed: bool,
        prefill: usize,
        stats: Arc<Stats>,
    }

    impl RenderHandler for Out {
        fn on_render(&mut self, stereo: &mut [f32], info: &RenderInfo) {
            if !self.primed && self.ring.slots() >= self.prefill {
                self.primed = true;
            }
            if self.primed {
                let n = stereo.len().min(self.ring.slots());
                if let Ok(c) = self.ring.read_chunk(n) {
                    let (a, b) = c.as_slices();
                    stereo[..a.len()].copy_from_slice(a);
                    stereo[a.len()..a.len() + b.len()].copy_from_slice(b);
                    c.commit_all();
                }
                if n < stereo.len() {
                    // Underrun: the rest stays zero (pre-zeroed by the adapter).
                    self.stats
                        .dropped
                        .fetch_add(((stereo.len() - n) / 2) as u64, Ordering::Relaxed);
                }
            }
            self.stats
                .after_ns
                .store(info.after_timestamp.as_nanos() as u64, Ordering::Relaxed);
            self.stats
                .record(stereo.len() / 2, info.host_time, info.discontinuity, stereo);
        }
    }

    struct Tone {
        phase: f64,
        step: f64,
        amp: f32,
        stats: Arc<Stats>,
    }

    impl RenderHandler for Tone {
        fn on_render(&mut self, stereo: &mut [f32], info: &RenderInfo) {
            for f in stereo.chunks_exact_mut(2) {
                let v = (self.phase.sin() as f32) * self.amp;
                self.phase = (self.phase + self.step) % std::f64::consts::TAU;
                f[0] = v;
                f[1] = v;
            }
            self.stats
                .record(stereo.len() / 2, info.host_time, info.discontinuity, stereo);
        }
    }

    fn run_tone(p: &MacPlatform, uid: &str, amp: f32, secs: f64) {
        let stats = Stats::new();
        let id = DeviceId(uid.to_string());
        let rate = p
            .output_devices()
            .unwrap_or_default()
            .into_iter()
            .find(|d| d.id == id)
            .map(|d| d.nominal_rate)
            .unwrap_or(48_000.0);
        let s = p
            .open_output(
                &id,
                OutputConfig {
                    preferred_block_frames: 256,
                },
                Box::new(Tone {
                    phase: 0.0,
                    step: std::f64::consts::TAU * 1000.0 / rate,
                    amp,
                    stats: stats.clone(),
                }),
            )
            .expect("open_output");
        println!("tone {amp} on {uid}: {:?}", s.format());
        let start = Instant::now();
        let mut prev = (0, 0);
        while start.elapsed().as_secs_f64() < secs {
            std::thread::sleep(Duration::from_secs(1));
            println!("  {}", stats.line("tone", &mut prev, 1.0, p.now()));
        }
    }

    struct Inp {
        stats: Arc<Stats>,
    }

    impl InputHandler for Inp {
        fn on_input(&mut self, mono: &[f32], info: &CaptureInfo) {
            self.stats
                .record(mono.len(), info.host_time, info.discontinuity, mono);
        }
    }

    fn arg<'a>(args: &'a [String], k: &str) -> Option<&'a str> {
        args.iter()
            .position(|a| a == k)
            .and_then(|i| args.get(i + 1))
            .map(|s| s.as_str())
    }

    /// Names of aggregate devices visible to this process whose name starts with "Wye".
    fn wye_aggregates() -> Vec<String> {
        #[repr(C)]
        struct Addr {
            sel: u32,
            scope: u32,
            el: u32,
        }
        #[link(name = "CoreAudio", kind = "framework")]
        extern "C" {
            fn AudioObjectGetPropertyDataSize(
                id: u32,
                a: *const Addr,
                qs: u32,
                q: *const std::ffi::c_void,
                out: *mut u32,
            ) -> i32;
            fn AudioObjectGetPropertyData(
                id: u32,
                a: *const Addr,
                qs: u32,
                q: *const std::ffi::c_void,
                size: *mut u32,
                out: *mut std::ffi::c_void,
            ) -> i32;
        }
        #[link(name = "CoreFoundation", kind = "framework")]
        extern "C" {
            fn CFStringGetCString(s: *const std::ffi::c_void, b: *mut u8, n: isize, e: u32) -> u8;
            fn CFRelease(s: *const std::ffi::c_void);
        }
        let fcc = |s: &[u8; 4]| u32::from_be_bytes(*s);
        let glob = fcc(b"glob");
        let mut out = Vec::new();
        unsafe {
            let a = Addr {
                sel: fcc(b"dev#"),
                scope: glob,
                el: 0,
            };
            let mut size = 0u32;
            if AudioObjectGetPropertyDataSize(1, &a, 0, std::ptr::null(), &mut size) != 0 {
                return out;
            }
            let mut ids = vec![0u32; size as usize / 4];
            if AudioObjectGetPropertyData(
                1,
                &a,
                0,
                std::ptr::null(),
                &mut size,
                ids.as_mut_ptr().cast(),
            ) != 0
            {
                return out;
            }
            for id in ids {
                let a = Addr {
                    sel: fcc(b"lnam"),
                    scope: glob,
                    el: 0,
                };
                let mut p: *const std::ffi::c_void = std::ptr::null();
                let mut sz = std::mem::size_of::<*const std::ffi::c_void>() as u32;
                if AudioObjectGetPropertyData(
                    id,
                    &a,
                    0,
                    std::ptr::null(),
                    &mut sz,
                    (&mut p as *mut *const std::ffi::c_void).cast(),
                ) != 0
                    || p.is_null()
                {
                    continue;
                }
                let mut buf = [0u8; 256];
                let ok =
                    CFStringGetCString(p, buf.as_mut_ptr(), buf.len() as isize, 0x0800_0100) != 0;
                CFRelease(p);
                if ok {
                    let end = buf.iter().position(|&b| b == 0).unwrap_or(0);
                    let name = String::from_utf8_lossy(&buf[..end]).into_owned();
                    if name.starts_with("Wye") {
                        out.push(format!("#{id} {name}"));
                    }
                }
            }
        }
        out
    }

    pub fn run() {
        let args: Vec<String> = std::env::args().collect();
        let secs: f64 = arg(&args, "--secs")
            .and_then(|s| s.parse().ok())
            .unwrap_or(10.0);
        let out_uid = arg(&args, "--out")
            .unwrap_or("BlackHole2ch_UID")
            .to_string();
        let in_uid = arg(&args, "--input").map(str::to_string);
        let block: u32 = arg(&args, "--block")
            .and_then(|s| s.parse().ok())
            .unwrap_or(256);
        let mute = args.iter().any(|a| a == "--mute");

        let p = MacPlatform::new().expect("platform");
        p.set_show_virtual(true);
        if let Some(amp) = arg(&args, "--tone").and_then(|s| s.parse::<f32>().ok()) {
            run_tone(&p, &out_uid, amp, secs);
            return;
        }
        println!(
            "pid {}  mute_original={mute}  preferred block {block}",
            std::process::id()
        );
        let _watch = p
            .watch(Box::new(|ev| println!("event: {ev:?}")))
            .expect("watch");

        let (prod, cons) = rtrb::RingBuffer::<f32>::new(48_000 * 2 * 4);
        let cap_stats = Stats::new();
        let out_stats = Stats::new();
        let in_stats = Stats::new();

        let t = Instant::now();
        let cap = p.open_capture(
            CaptureConfig {
                mute_original: mute,
                preferred_block_frames: block,
            },
            Box::new(Cap {
                ring: prod,
                stats: cap_stats.clone(),
            }),
        );
        let probe = p.last_capture_probe();
        println!("open_capture took {:?}; probe = {probe:?}", t.elapsed());
        if let Some(pr) = &probe {
            println!(
                "st1 = {} (0x{:08x})  st2 = {} (0x{:08x})  own process object = {}",
                pr.st1, pr.st1 as u32, pr.st2, pr.st2 as u32, pr.own_process_object
            );
        }
        let cap = match cap {
            Ok(s) => s,
            Err(e) => {
                println!("open_capture failed: {e}");
                return;
            }
        };
        println!("capture format: {:?}", cap.format());
        println!("aggregates while capturing: {:?}", wye_aggregates());

        let out = p
            .open_output(
                &DeviceId(out_uid.clone()),
                OutputConfig {
                    preferred_block_frames: block,
                },
                Box::new(Out {
                    ring: cons,
                    primed: false,
                    prefill: 2048 * 2,
                    stats: out_stats.clone(),
                }),
            )
            .expect("open_output");
        println!("output {out_uid} format: {:?}", out.format());

        let input = in_uid.as_ref().map(|u| {
            let s = p
                .open_input(
                    &DeviceId(u.clone()),
                    Box::new(Inp {
                        stats: in_stats.clone(),
                    }),
                )
                .expect("open_input");
            println!(
                "input {u} format: {:?}; input latency {:?}",
                s.format(),
                p.input_latency(&DeviceId(u.clone()))
            );
            s
        });

        let start = Instant::now();
        let mut last = Instant::now();
        let (mut pc, mut po, mut pi) = ((0, 0), (0, 0), (0, 0));
        let stop = Arc::new(AtomicBool::new(false));
        while start.elapsed().as_secs_f64() < secs && !stop.load(Ordering::Relaxed) {
            std::thread::sleep(Duration::from_secs(1));
            let dt = last.elapsed().as_secs_f64();
            last = Instant::now();
            let now = p.now();
            println!("[{:5.1}s]", start.elapsed().as_secs_f64());
            println!("  {}", cap_stats.line("capture", &mut pc, dt, now));
            println!("  {}", out_stats.line("render", &mut po, dt, now));
            if input.is_some() {
                println!("  {}", in_stats.line("input", &mut pi, dt, now));
            }
        }
        drop(input);
        drop(out);
        let t = Instant::now();
        drop(cap);
        println!("capture dropped in {:?}", t.elapsed());
        std::thread::sleep(Duration::from_millis(300));
        let left = wye_aggregates();
        println!(
            "aggregates after drop: {left:?} -> {}",
            if left.is_empty() { "clean" } else { "LEAK" }
        );
    }
}

#[cfg(target_os = "macos")]
fn main() {
    harness::run();
}

#[cfg(not(target_os = "macos"))]
fn main() {}
