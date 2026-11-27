//! Reference implementation of Wye's per-output-device latency controller.
//!
//! Platform-free, allocation-free, lock-free (plain struct, f64 math). Parameters were
//! chosen by simulation (timestamp-anchored latency measurement, 24 h runs).
//!
//! Plant model:
//!   e  = measured latency error (s) = λ_meas − λ_target   (positive ⇒ mirror plays late)
//!   r  = (f_out_nom / f_in_nom) · (1 + δ)                  (resampler ratio, out/in)
//!   de/dt = δ − δ*,   δ* ≈ ε_out − ε_in                     (true relative clock offset)
//! Control law (PI on EMA-filtered error, conditional-integration anti-windup):
//!   δ = −(Kp·e_f + Ki·∫e_f dt),  Kp = 2ζωn,  Ki = ωn²
//! Closed loop: x'' + Kp x' + Ki x = −δ*  ⇒  steady state e = 0, δ = δ*.

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControllerParams {
    /// EMA pre-filter time constant on the error, seconds.
    pub tau_filter: f64,
    /// Closed-loop natural frequency, rad/s.
    pub omega_n: f64,
    /// Damping ratio.
    pub zeta: f64,
    /// |δ| limit (1e-3 = 1000 ppm).
    pub delta_max: f64,
    /// Innovation gate (s): samples with |e − e_f| > gate are held as suspected outliers.
    pub gate: f64,
    /// Consecutive outliers needed to accept a genuine step.
    pub confirm: u32,
    /// If the *filtered* error ever exceeds this (s) — only possible when |δ*| > δmax —
    /// it is removed by a hard resync. (Confirmed steps are always resynced.)
    pub resync_threshold: f64,
}

impl ControllerParams {
    pub const DEFAULT: ControllerParams = ControllerParams {
        tau_filter: 1.0,
        omega_n: 0.1,
        zeta: 1.0,
        delta_max: 1.0e-3,
        gate: 0.004,
        confirm: 8,
        resync_threshold: 0.010,
    };
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Correction {
    None,
    /// Hard resync by `seconds` of source material. Positive: mirror is late ⇒ skip
    /// forward. Negative: mirror is early ⇒ insert |seconds| of silence (hold).
    Resync {
        seconds: f64,
    },
}

#[derive(Clone, Debug)]
pub struct Controller {
    p: ControllerParams,
    kp: f64,
    ki: f64,
    e_f: f64,
    integ: f64,
    delta: f64,
    outliers: u32,
    primed: bool,
    pub stats: ControllerStats,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ControllerStats {
    pub outliers_rejected: u64,
    pub steps_accepted: u64,
    pub resyncs: u64,
    pub runaway_resyncs: u64,
    pub saturated_updates: u64,
}

impl Controller {
    pub fn new(p: ControllerParams) -> Self {
        Controller {
            kp: 2.0 * p.zeta * p.omega_n,
            ki: p.omega_n * p.omega_n,
            p,
            e_f: 0.0,
            integ: 0.0,
            delta: 0.0,
            outliers: 0,
            primed: false,
            stats: ControllerStats::default(),
        }
    }

    pub fn delta(&self) -> f64 {
        self.delta
    }
    pub fn filtered_error(&self) -> f64 {
        self.e_f
    }

    /// Optional feed-forward: seed the integrator so that δ starts at `delta0`.
    pub fn seed_delta(&mut self, delta0: f64) {
        let d = delta0.clamp(-self.p.delta_max, self.p.delta_max);
        self.integ = -d / self.ki;
        self.delta = d;
    }

    /// Call once per output callback with the raw latency error `e` (s), the nominal
    /// callback period `dt` (s) and `discontinuity` = the platform saw a timeline break on
    /// this stream since the last call (lost/overloaded cycle, sample-time jump, stream
    /// restart). Returns the new δ and an optional hard correction.
    /// Real-time safe: no allocation, no locks, bounded work.
    pub fn update(&mut self, e: f64, dt: f64, discontinuity: bool) -> (f64, Correction) {
        if !self.primed {
            self.e_f = e;
            self.primed = true;
        }
        let innov = e - self.e_f;
        let mut step = false;
        if innov.abs() > self.p.gate {
            if discontinuity {
                step = true; // platform told us the timeline broke: trust it at once
            } else {
                self.outliers += 1;
                if self.outliers < self.p.confirm {
                    self.stats.outliers_rejected += 1;
                    return (self.delta, Correction::None); // hold δ while unsure
                }
                step = true;
            }
        }
        if step {
            // The level really moved (glitch, underrun, lost cycle). Remove the step with
            // one crossfaded jump; keep the integrator (the clock ratio δ* did not change).
            self.outliers = 0;
            self.stats.resyncs += 1;
            self.e_f = 0.0;
            return (self.delta, Correction::Resync { seconds: e });
        }
        self.outliers = 0;
        let a = 1.0 - (-dt / self.p.tau_filter).exp();
        self.e_f += a * innov;

        // A filtered error that grows beyond the resync threshold (e.g. |δ*| > δmax)
        // is also corrected by a jump; PI alone could never catch up.
        if self.e_f.abs() > self.p.resync_threshold {
            self.stats.resyncs += 1;
            self.stats.runaway_resyncs += 1;
            let s = self.e_f;
            self.e_f = 0.0;
            return (self.delta, Correction::Resync { seconds: s });
        }

        let dmax = self.p.delta_max;
        let integ_try = self.integ + self.e_f * dt;
        let u_try = -(self.kp * self.e_f + self.ki * integ_try);
        let pushes_further = (u_try > dmax && self.e_f < 0.0) || (u_try < -dmax && self.e_f > 0.0);
        if !pushes_further {
            self.integ = integ_try;
        }
        // keep the integral term itself inside the actuator range
        let imax = dmax / self.ki;
        self.integ = self.integ.clamp(-imax, imax);
        let u = -(self.kp * self.e_f + self.ki * self.integ);
        if u.abs() > dmax {
            self.stats.saturated_updates += 1;
        }
        self.delta = u.clamp(-dmax, dmax);
        (self.delta, Correction::None)
    }

    /// Forget the filter state but keep the drift estimate (integrator). Use after a
    /// resync that the caller performed for its own reasons (e.g. offset slider moved).
    pub fn reset_error(&mut self) {
        self.e_f = 0.0;
        self.outliers = 0;
        self.primed = false;
    }
}

/// Second-order DLL for smoothing callback wake-up timestamps (Adriaensen, "Using a DLL
/// to filter time", LAC 2005). Used only when the platform gives no device timestamps.
#[derive(Clone, Debug)]
pub struct TimeDll {
    b: f64,
    c: f64,
    t_pred: f64,
    period: f64,
    nominal_period: f64,
    late_clamp: f64,
    reset_threshold: f64,
    primed: bool,
}

impl TimeDll {
    /// `bandwidth_hz`: loop bandwidth; `nominal_period`: block duration (s).
    pub fn new(bandwidth_hz: f64, nominal_period: f64) -> Self {
        let w = 2.0 * std::f64::consts::PI * bandwidth_hz * nominal_period;
        TimeDll {
            b: std::f64::consts::SQRT_2 * w,
            c: w * w,
            t_pred: 0.0,
            period: nominal_period,
            nominal_period,
            late_clamp: 0.002,
            reset_threshold: 0.050,
            primed: false,
        }
    }
    /// A block boundary passed without a measurement (lost cycle): advance the prediction.
    pub fn skip_one(&mut self) -> f64 {
        let t = self.t_pred;
        self.t_pred += self.period;
        t
    }
    /// Feed the measured time of this block boundary; returns the filtered time.
    pub fn update(&mut self, t_meas: f64) -> f64 {
        if !self.primed {
            self.t_pred = t_meas;
            self.period = self.nominal_period;
            self.primed = true;
            let t = self.t_pred;
            self.t_pred += self.period;
            return t;
        }
        let mut err = t_meas - self.t_pred;
        if err.abs() > self.reset_threshold {
            // discontinuity (stall with data loss, device restart): re-prime
            self.t_pred = t_meas;
            self.period = self.nominal_period;
            let t = self.t_pred;
            self.t_pred += self.period;
            return t;
        }
        // Wake-ups are only ever late, never early: clamp positive outliers.
        if err > self.late_clamp {
            err = self.late_clamp;
        }
        // Adriaensen's form: the filtered time of this boundary is the prior prediction.
        let t_filt = self.t_pred;
        self.t_pred += self.b * err + self.period;
        self.period += self.c * err;
        t_filt
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Nominal render callback period used by the tests: 512 frames at 48 kHz.
    const DT: f64 = 512.0 / 48_000.0;

    /// Ideal plant from the module documentation: `de/dt = δ − δ*`, where `δ*` is the true
    /// relative clock offset between output and capture.
    struct Plant {
        e: f64,
        drift: f64,
    }

    impl Plant {
        fn step(&mut self, delta: f64, corr: Correction) {
            if let Correction::Resync { seconds } = corr {
                self.e -= seconds;
            }
            self.e += (delta - self.drift) * DT;
        }
    }

    fn ctl() -> Controller {
        Controller::new(ControllerParams::DEFAULT)
    }

    fn run(c: &mut Controller, p: &mut Plant, seconds: f64) {
        for _ in 0..(seconds / DT) as usize {
            let (d, corr) = c.update(p.e, DT, false);
            p.step(d, corr);
        }
    }

    #[test]
    fn drift_step_300ppm_settles() {
        let mut c = ctl();
        let mut p = Plant {
            e: 0.0,
            drift: 300e-6,
        };
        let mut peak: f64 = 0.0;
        let mut entered_at = None;
        for i in 0..(120.0 / DT) as usize {
            let (d, corr) = c.update(p.e, DT, false);
            assert_eq!(corr, Correction::None, "no resync expected at step {i}");
            p.step(d, corr);
            let t = (i + 1) as f64 * DT;
            peak = peak.max(p.e.abs());
            if p.e.abs() <= 0.5e-3 {
                entered_at.get_or_insert(t);
            } else {
                entered_at = None;
            }
        }
        // Analytically the PI loop alone peaks at δ*/(ωn·e) ≈ 1.10 ms; the 1 s error
        // pre-filter adds lag and the exact peak is 1.2026 ms independent of the callback
        // period. The bound is 1.2 ms at 0.01 ms resolution.
        assert!(peak <= 1.21e-3, "peak error {:.4} ms", peak * 1e3);
        let t_in = entered_at.expect("never settled inside ±0.5 ms");
        assert!(t_in <= 30.0, "entered ±0.5 ms for good at {t_in:.1} s");
        // Steady state: δ tracks the clock offset and the error vanishes.
        assert!((c.delta() - 300e-6).abs() < 1e-6, "δ = {}", c.delta());
        assert!(p.e.abs() < 1e-5);
    }

    #[test]
    fn latency_step_resyncs_and_keeps_integrator() {
        let mut c = ctl();
        let mut p = Plant {
            e: 0.0,
            drift: 200e-6,
        };
        run(&mut c, &mut p, 200.0);
        let delta_before = c.delta();
        let integ_before = c.integ;
        assert!((delta_before - 200e-6).abs() < 1e-6);

        // A 20 ms step (e.g. an underrun): the first confirm−1 samples are held as outliers.
        let e_step = p.e + 0.020;
        for _ in 0..ControllerParams::DEFAULT.confirm - 1 {
            let (d, corr) = c.update(e_step, DT, false);
            assert_eq!(corr, Correction::None);
            assert_eq!(d, delta_before, "δ must be held while unsure");
        }
        let (d, corr) = c.update(e_step, DT, false);
        match corr {
            Correction::Resync { seconds } => {
                assert!((seconds - 0.020).abs() < 1e-4, "resync {seconds}")
            }
            other => panic!("expected resync, got {other:?}"),
        }
        assert_eq!(d, delta_before);
        assert_eq!(c.integ, integ_before, "integrator must survive a resync");
        assert_eq!(c.filtered_error(), 0.0);
        assert_eq!(c.stats.resyncs, 1);
        assert_eq!(c.stats.runaway_resyncs, 0);

        // The plant applies the jump (removing the step) and the loop simply continues.
        p.e = e_step;
        p.step(d, corr);
        let (_, corr) = c.update(p.e, DT, false);
        assert_eq!(corr, Correction::None);
    }

    #[test]
    fn discontinuity_resyncs_on_single_update() {
        let mut c = ctl();
        for _ in 0..100 {
            c.update(0.0, DT, false);
        }
        let (_, corr) = c.update(0.015, DT, true);
        assert_eq!(corr, Correction::Resync { seconds: 0.015 });
        assert_eq!(c.stats.outliers_rejected, 0);
        // A negative step (mirror early) also resyncs at once, asking for silence.
        let (_, corr) = c.update(-0.012, DT, true);
        assert_eq!(corr, Correction::Resync { seconds: -0.012 });
        // The flag alone, without a step, does not force a resync.
        let (_, corr) = c.update(0.0001, DT, true);
        assert_eq!(corr, Correction::None);
    }

    #[test]
    fn saturation_anti_windup() {
        let mut c = ctl();
        let dmax = ControllerParams::DEFAULT.delta_max;
        let imax = dmax / c.ki;
        // 1500 ppm drift exceeds the actuator: δ saturates and the error runs away.
        let mut p = Plant {
            e: 0.0,
            drift: 1500e-6,
        };
        let mut saturated_seen = false;
        for _ in 0..(120.0 / DT) as usize {
            let (d, corr) = c.update(p.e, DT, false);
            assert!(c.integ.abs() <= imax * (1.0 + 1e-12), "integrator wound up");
            saturated_seen |= d == dmax;
            p.step(d, corr);
        }
        assert!(saturated_seen);
        assert!(c.stats.saturated_updates > 0);
        assert!(
            c.stats.runaway_resyncs > 0,
            "runaway error must be resynced"
        );
        assert!(p.e.abs() <= 0.011, "|e| = {:.2} ms", p.e * 1e3);

        // Drift returns into range: without wind-up δ recovers promptly instead of staying
        // pinned at the limit.
        p.drift = 0.0;
        let mut recovered_at = None;
        for i in 0..(120.0 / DT) as usize {
            let (d, corr) = c.update(p.e, DT, false);
            p.step(d, corr);
            if recovered_at.is_none() && d.abs() < 100e-6 && p.e.abs() < 0.5e-3 {
                recovered_at = Some(i as f64 * DT);
            }
        }
        let t = recovered_at.expect("did not recover after saturation");
        assert!(t < 60.0, "recovered after {t:.1} s");
        assert!(c.delta().abs() < 1e-5);
    }

    #[test]
    fn delta_always_within_limit() {
        let dmax = ControllerParams::DEFAULT.delta_max;
        let mut c = ctl();
        // Deterministic xorshift error sequence: small and large swings, random flags.
        let mut x: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut rnd = move || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            (x >> 11) as f64 / (1u64 << 53) as f64
        };
        for i in 0..200_000 {
            let scale = if i % 1000 < 500 { 0.008 } else { 0.2 };
            let e = (rnd() - 0.5) * scale;
            let disc = rnd() < 0.01;
            let (d, _) = c.update(e, DT, disc);
            assert!(d.abs() <= dmax, "δ = {d} at {i}");
            assert_eq!(d, c.delta());
        }
        // Seeding is clamped as well.
        c.seed_delta(5e-3);
        assert_eq!(c.delta(), dmax);
        c.seed_delta(-5e-3);
        assert_eq!(c.delta(), -dmax);
        let (d, _) = c.update(-1.0e-3, DT, false);
        assert!(d.abs() <= dmax);
    }

    #[test]
    fn reset_error_keeps_drift_estimate() {
        let mut c = ctl();
        let mut p = Plant {
            e: 0.0,
            drift: -150e-6,
        };
        run(&mut c, &mut p, 200.0);
        let d0 = c.delta();
        let integ0 = c.integ;
        c.reset_error();
        assert_eq!(c.filtered_error(), 0.0);
        assert_eq!(c.integ, integ0);
        // After a reset the next sample primes the filter instead of counting as an outlier.
        let (_, corr) = c.update(0.003, DT, false);
        assert_eq!(corr, Correction::None);
        assert_eq!(c.stats.outliers_rejected, 0);
        assert!((c.filtered_error() - 0.003).abs() < 1e-12);
        assert!((c.integ - integ0).abs() < 1e-3);
        assert!(d0 < 0.0);
    }
}
