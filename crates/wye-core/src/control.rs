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
