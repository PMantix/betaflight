import numpy as np
import matplotlib.pyplot as plt

def half_cos_ramp(t, T, wmax, up=True):
    x = np.clip(t / T, 0.0, 1.0)
    if up:
        return 0.5 * wmax * (1 - np.cos(np.pi * x))
    else:
        return 0.5 * wmax * (1 + np.cos(np.pi * x))

def build_setpoint(t, Tr, Th, wmax):
    w = np.zeros_like(t)
    for i, ti in enumerate(t):
        if ti < Tr:
            w[i] = half_cos_ramp(ti, Tr, wmax, up=True)
        elif ti < Tr + Th:
            w[i] = wmax
        elif ti < 2 * Tr + Th:
            w[i] = half_cos_ramp(ti - (Tr + Th), Tr, wmax, up=False)
        else:
            w[i] = 0.0
    return w

def sat(x, lo, hi):
    return np.minimum(np.maximum(x, lo), hi)

class LPF1_RT:
    """First-order low-pass filter for real-time (sample-by-sample) use."""
    def __init__(self, dt, fc_hz):
        self.dt = dt
        self.fc_hz = fc_hz
        if fc_hz > 0:
            rc = 1.0 / (2.0 * np.pi * fc_hz)
            self.alpha = dt / (rc + dt)
        else:
            self.alpha = 1.0  # passthrough
        self.state = 0.0
        self.initialized = False
    
    def update(self, x):
        if not self.initialized:
            self.state = x
            self.initialized = True
        else:
            self.state += self.alpha * (x - self.state)
        return self.state
    
    def reset(self):
        self.state = 0.0
        self.initialized = False

class LPF2_RT:
    """Second-order (biquad) low-pass filter for real-time use.
    
    Butterworth design for maximally flat passband.
    Q = 1/sqrt(2) ≈ 0.707 for Butterworth response.
    """
    def __init__(self, dt, fc_hz, Q=0.7071):
        self.dt = dt
        self.fc_hz = fc_hz
        self.Q = Q
        
        if fc_hz <= 0 or fc_hz >= 0.5/dt:
            # Passthrough or invalid
            self.b0 = 1.0
            self.b1 = 0.0
            self.b2 = 0.0
            self.a1 = 0.0
            self.a2 = 0.0
        else:
            # Bilinear transform of analog 2nd-order LPF
            omega = 2.0 * np.pi * fc_hz
            omega_d = 2.0 / dt * np.tan(omega * dt / 2.0)  # Prewarp
            
            k = omega_d / (2.0 / dt)
            k2 = k * k
            norm = 1.0 / (1.0 + k/Q + k2)
            
            self.b0 = k2 * norm
            self.b1 = 2.0 * self.b0
            self.b2 = self.b0
            self.a1 = 2.0 * (k2 - 1.0) * norm
            self.a2 = (1.0 - k/Q + k2) * norm
        
        # State variables (direct form II transposed)
        self.z1 = 0.0
        self.z2 = 0.0
        self.initialized = False
    
    def update(self, x):
        if not self.initialized:
            # Initialize to avoid transient
            self.z1 = x * (1.0 - self.b0)
            self.z2 = x * (1.0 - self.b0)
            self.initialized = True
        
        # Direct form II transposed
        y = self.b0 * x + self.z1
        self.z1 = self.b1 * x - self.a1 * y + self.z2
        self.z2 = self.b2 * x - self.a2 * y
        return y
    
    def reset(self):
        self.z1 = 0.0
        self.z2 = 0.0
        self.initialized = False

class GyroFilterChain:
    """Multi-stage gyro filter chain simulating Betaflight's filter stack.
    
    Real Betaflight gyro filtering includes:
    1. Hardware LPF on gyro chip (~250Hz typically)
    2. Software LPF1 - O1 (configurable, typically 150-250Hz)  
    3. Software LPF2 - O2 biquad (configurable, typically ~500Hz)
    4. Dynamic notch filters (add ~2-3ms group delay)
    5. RPM filters (add ~1-2ms group delay)
    
    Total typical delay: 4-8ms of group delay
    """
    def __init__(self, dt, lpf1_hz=250.0, lpf2_hz=150.0, lpf2_o2_hz=500.0, 
                 lpf3_hz=100.0, use_all_stages=True):
        self.filters = []
        # Stage 1: Hardware/chip filter (high cutoff) - O1
        self.filters.append(LPF1_RT(dt, lpf1_hz))
        if use_all_stages:
            # Stage 2: Software LPF1 - O1
            self.filters.append(LPF1_RT(dt, lpf2_hz))
            # Stage 3: Software LPF2 - O2 biquad (gyro_lpf2)
            self.filters.append(LPF2_RT(dt, lpf2_o2_hz))
            # Stage 4: Additional LPF representing dynamic notches' group delay
            self.filters.append(LPF1_RT(dt, lpf3_hz))
        
    def update(self, x):
        y = x
        for f in self.filters:
            y = f.update(y)
        return y
    
    def reset(self):
        for f in self.filters:
            f.reset()

class DtermFilterChain:
    """D-term filter chain simulating Betaflight's dterm filtering.
    
    Real Betaflight D-term filtering includes:
    1. dterm_lpf1 - O1 LPF (configurable, typically 75-100Hz)
    2. dterm_lpf2 - O2 biquad (configurable, typically 150-250Hz)
    """
    def __init__(self, dt, lpf1_hz=75.0, lpf2_hz=250.0):
        self.lpf1 = LPF1_RT(dt, lpf1_hz)
        self.lpf2 = LPF2_RT(dt, lpf2_hz)
    
    def update(self, x):
        y = self.lpf1.update(x)
        y = self.lpf2.update(y)
        return y
    
    def reset(self):
        self.lpf1.reset()
        self.lpf2.reset()

def lpf1(x, dt, fc_hz):
    """First-order low-pass filter."""
    if fc_hz <= 0:
        return x.copy()
    rc = 1.0 / (2.0 * np.pi * fc_hz)
    a = dt / (rc + dt)
    y = np.zeros_like(x)
    y[0] = x[0]
    for k in range(1, len(x)):
        y[k] = y[k-1] + a * (x[k] - y[k-1])
    return y

def simulate(cfg):
    dt = cfg["dt"]
    t = np.arange(0, cfg["T"], dt)

    # Truth
    E_true = cfg["E_true"]
    b_true = cfg["b_true"]
    tau_m = cfg["tau_m"]

    # Controller gains
    Kp, Ki, Kd = cfg["Kp"], cfg["Ki"], cfg["Kd"]
    D_on_error = cfg["D_on_error"]
    kff = cfg["kff"]

    # FF model params used online
    E_hat = cfg["E_hat"]
    b_hat = cfg["b_hat"]
    use_ff = cfg["use_ff"]
    
    # RPM-dependent effectiveness (new)
    use_rpm_dependent_E = cfg.get("use_rpm_dependent_E", False)
    E0_hat = cfg.get("E0_hat", E_hat)  # Base effectiveness
    E1_hat = cfg.get("E1_hat", 0.0)    # RPM coefficient

    # Maneuver
    w_sp = build_setpoint(t, cfg["Tr"], cfg["Th"], cfg["wmax"])
    # "intent"
    alpha_cmd_raw = np.gradient(w_sp, dt)
    alpha_cmd = lpf1(alpha_cmd_raw, dt, cfg["alpha_cmd_lpf_hz"])
    alpha_min = cfg["alpha_min"]

    # Motor mixing
    m0 = cfg["m0"]
    umax = cfg["u_axis_max"]
    eps = cfg["sat_eps"]
    
    # RPM² nonlinearity parameters
    use_rpm_sq = cfg.get("use_rpm_sq", False)
    rpm_max = cfg.get("rpm_max", 25000.0)        # Max motor RPM
    tau_rpm = cfg.get("tau_rpm", 0.015)          # Motor RPM time constant (s)
    k_torque = cfg.get("k_torque", 1.0)          # Torque coefficient

    # Noise + measurement pipeline
    gyro_noise_std = cfg["gyro_noise_std"]
    gyro_lpf_hz = cfg["gyro_lpf_hz"]
    alpha_est_lpf_hz = cfg["alpha_est_lpf_hz"]
    
    # Gyro filter chain (simulates Betaflight's multi-stage filtering)
    # This adds realistic delay to the feedback path
    gyro_filter_lpf1_hz = cfg.get("gyro_filter_lpf1_hz", 250.0)   # Hardware LPF - O1
    gyro_filter_lpf2_hz = cfg.get("gyro_filter_lpf2_hz", 150.0)   # Software LPF1 - O1
    gyro_filter_lpf2_o2_hz = cfg.get("gyro_filter_lpf2_o2_hz", 500.0)  # Software LPF2 - O2 biquad
    gyro_filter_lpf3_hz = cfg.get("gyro_filter_lpf3_hz", 100.0)   # Dynamic notch delay approx
    use_gyro_delay = cfg.get("use_gyro_delay", True)  # Enable realistic gyro delay
    
    gyro_filter = GyroFilterChain(dt, gyro_filter_lpf1_hz, gyro_filter_lpf2_hz, 
                                   gyro_filter_lpf2_o2_hz, gyro_filter_lpf3_hz,
                                   use_all_stages=use_gyro_delay)
    
    # D-term filter chain (Betaflight's dterm_lpf1 + dterm_lpf2)
    # This adds additional delay to the D-term path
    dterm_lpf1_hz = cfg.get("dterm_lpf1_hz", 75.0)   # dterm_lpf1 - O1 LPF
    dterm_lpf2_hz = cfg.get("dterm_lpf2_hz", 250.0)  # dterm_lpf2 - O2 biquad
    dterm_filter = DtermFilterChain(dt, dterm_lpf1_hz, dterm_lpf2_hz)

    # State
    w = 0.0           # True angular rate (plant state)
    w_filtered = 0.0  # Filtered gyro (what controller sees)
    ua = 0.0
    ua_torque = 0.0  # Actual torque for RPM² model
    I = 0.0
    e_prev = 0.0
    w_filt_prev = 0.0  # Previous filtered gyro (for D-term)
    D_raw = 0.0       # Unfiltered D-term
    
    # Motor RPM state (for RPM² model)
    rpm = np.array([0.0, 0.0, 0.0, 0.0])  # 4 motors

    # Logs
    W = np.zeros_like(t)
    Alpha_true = np.zeros_like(t)
    Ureq = np.zeros_like(t)
    Uapplied = np.zeros_like(t)
    Utorque = np.zeros_like(t)
    Ua = np.zeros_like(t)
    M = np.zeros((len(t), 4))
    PID = np.zeros_like(t)
    P_term = np.zeros_like(t)
    I_term = np.zeros_like(t)
    D_term = np.zeros_like(t)
    FF = np.zeros_like(t)
    RPM_avg = np.zeros_like(t)
    RPM_sq_avg = np.zeros_like(t)
    E_effective = np.zeros_like(t)  # Effective E at each timestep

    # For measurement-based alpha estimate
    w_meas = np.zeros_like(t)
    W_filtered = np.zeros_like(t)  # Log filtered gyro

    rng = np.random.default_rng(cfg["seed"])

    for k, tk in enumerate(t):
        # Add sensor noise to true rate
        w_noisy = w + rng.normal(0.0, gyro_noise_std)
        
        # Apply gyro filter chain (this is what the controller sees)
        # Multiple filter stages add phase delay - this is realistic!
        w_filtered = gyro_filter.update(w_noisy)
        
        # PID uses filtered gyro, NOT true rate
        # This is key - the feedback path has delay!
        e = w_sp[k] - w_filtered
        I += e * dt

        if D_on_error:
            de = (e - e_prev) / dt
            D_raw = de
        else:
            dw = (w_filtered - w_filt_prev) / dt
            D_raw = -dw
        
        # Apply D-term lowpass filter (adds more delay to D path)
        D = dterm_filter.update(D_raw)

        u_pid = Kp * e + Ki * I + Kd * D

        if use_ff and (abs(alpha_cmd[k]) > alpha_min):
            if use_rpm_dependent_E:
                # RPM-dependent effectiveness: E(x) = E0 + E1 * x
                x_current = np.mean(rpm ** 2) / 1e6 if use_rpm_sq else 0.0
                E_current = E0_hat + E1_hat * x_current
                E_current = max(E_current, 0.1)  # Safety floor
            else:
                E_current = E_hat
            # FF uses filtered gyro for damping term (same as PID sees)
            u_ff = (alpha_cmd[k] + b_hat * w_filtered) / max(E_current, 1e-9)
        else:
            u_ff = 0.0

        u_req = u_pid + kff * u_ff
        u_req = sat(u_req, -umax, umax)

        # Motor mix (toy): 2 up, 2 down for roll axis
        # Motor commands are 0-100 representing throttle %
        m1_cmd = m0 + u_req
        m2_cmd = m0 + u_req
        m3_cmd = m0 - u_req
        m4_cmd = m0 - u_req

        # Saturate motor commands
        m_cmd = np.array([m1_cmd, m2_cmd, m3_cmd, m4_cmd])
        m_cmd_sat = sat(m_cmd, 0, 100)
        
        if use_rpm_sq:
            # ===== RPM² NONLINEAR MODEL =====
            # Motor command (0-100) maps to target RPM linearly
            # RPM has first-order dynamics
            rpm_target = (m_cmd_sat / 100.0) * rpm_max
            rpm += (rpm_target - rpm) * (dt / tau_rpm)
            rpm = np.maximum(rpm, 0)  # No negative RPM
            
            # Thrust is proportional to RPM²
            # T = k * RPM²
            # 
            # For differential roll torque with motors at RPM1,2 (up) and RPM3,4 (down):
            # τ_roll ∝ (RPM1² + RPM2²) - (RPM3² + RPM4²)
            #
            # At hover (all RPM = RPM_hover), a small Δcmd causes:
            #   RPM_up = RPM_hover + ΔRPM,  RPM_down = RPM_hover - ΔRPM
            #   τ ∝ 2*(RPM_hover + ΔRPM)² - 2*(RPM_hover - ΔRPM)²
            #     = 2*[4*RPM_hover*ΔRPM] = 8*RPM_hover*ΔRPM
            #
            # So effectiveness ∝ RPM_hover, which means E ∝ √(avg_RPM²)
            
            rpm_sq = rpm ** 2
            thrust = k_torque * rpm_sq / (rpm_max ** 2)  # Normalized 0-1
            
            # Differential torque for roll axis
            torque_diff = (thrust[0] + thrust[1]) - (thrust[2] + thrust[3])
            
            # The key insight: at different throttle positions, the same Δcmd
            # produces different torque. We want u_applied to represent the
            # "command" that the learner sees, while the plant has varying effectiveness.
            #
            # For the learner to work, we keep u_applied as the linear command difference,
            # but the actual angular acceleration depends on RPM.
            
            # Compute linear command (what controller thinks it's commanding)
            u_applied = 0.5 * ((m_cmd_sat[0] + m_cmd_sat[1]) - (m_cmd_sat[2] + m_cmd_sat[3]))
            
            # But actual torque depends on thrust differences
            # This creates a throttle-dependent effectiveness that the learner must identify
            actual_torque = torque_diff * 100.0  # Scale to similar range
            
        else:
            # ===== ORIGINAL LINEAR MODEL =====
            # Applied axis actuation from saturated motors
            u_applied = 0.5 * ((m_cmd_sat[0] + m_cmd_sat[1]) - (m_cmd_sat[2] + m_cmd_sat[3]))
            actual_torque = u_applied  # Linear: torque equals command

        # Actuator lag on applied axis actuation (for logging/learning)
        ua += (u_applied - ua) * (dt / tau_m)
        
        # For RPM² model, the actual torque drives the plant
        # Apply separate lag to track what the plant actually experiences
        if use_rpm_sq:
            ua_torque += (actual_torque - ua_torque) * (dt / tau_m)
        else:
            ua_torque = ua

        # Plant dynamics - use actual torque (which may differ from command in RPM² model)
        alpha_true = E_true * ua_torque - b_true * w
        w += alpha_true * dt

        # Measurement
        w_meas[k] = w + rng.normal(0.0, gyro_noise_std)

        # Log
        W[k] = w
        Alpha_true[k] = alpha_true
        Ureq[k] = u_req
        Uapplied[k] = u_applied
        Utorque[k] = actual_torque
        Ua[k] = ua
        M[k] = m_cmd_sat
        PID[k] = u_pid
        P_term[k] = Kp * e
        I_term[k] = Ki * I
        D_term[k] = Kd * D
        FF[k] = u_ff
        RPM_avg[k] = np.mean(rpm)
        RPM_sq_avg[k] = np.mean(rpm ** 2)
        
        # Effective E: ratio of actual torque to commanded
        if abs(u_applied) > 0.1:
            E_effective[k] = actual_torque / u_applied
        else:
            E_effective[k] = E_effective[k-1] if k > 0 else 1.0
        
        # Log filtered gyro (what controller sees)
        W_filtered[k] = w_filtered

        e_prev = e
        w_filt_prev = w_filtered  # Track filtered gyro for D-term

    # Measurement pipeline: gyro LPF then differentiate then LPF
    w_meas_f = lpf1(w_meas, dt, gyro_lpf_hz)
    alpha_est_raw = np.gradient(w_meas_f, dt)
    alpha_est = lpf1(alpha_est_raw, dt, alpha_est_lpf_hz)

    # Saturation mask
    unsat = (M.min(axis=1) > eps) & (M.max(axis=1) < (100 - eps))
    intent = np.abs(alpha_cmd) > alpha_min

    return {
        "t": t,
        "w_sp": w_sp,
        "alpha_cmd": alpha_cmd,
        "w_true": W,
        "w_filtered": W_filtered,  # What controller sees (delayed)
        "alpha_true": Alpha_true,
        "w_meas": w_meas,
        "w_meas_f": w_meas_f,
        "alpha_est": alpha_est,
        "u_req": Ureq,
        "u_applied": Uapplied,
        "u_torque": Utorque,
        "u_a": Ua,
        "motors": M,
        "pid": PID,
        "P": P_term,
        "I": I_term,
        "D": D_term,
        "u_ff": FF,
        "unsat": unsat,
        "intent": intent,
        "rpm_avg": RPM_avg,
        "rpm_sq_avg": RPM_sq_avg,
        "E_effective": E_effective,
    }

def learn_E_b(sim, use_alpha_est=True, add_intercept=True):
    mask = sim["intent"] & sim["unsat"]
    if use_alpha_est:
        y = sim["alpha_est"][mask]
    else:
        y = sim["alpha_true"][mask]
    ua = sim["u_a"][mask]
    w = sim["w_meas_f"][mask]  # use filtered gyro as omega for learning
    if add_intercept:
        X = np.vstack([ua, -w, np.ones_like(ua)]).T
        theta, *_ = np.linalg.lstsq(X, y, rcond=None)
        E_hat, b_hat, c_hat = theta
        return float(E_hat), float(b_hat), float(c_hat), int(mask.sum())
    else:
        X = np.vstack([ua, -w]).T
        theta, *_ = np.linalg.lstsq(X, y, rcond=None)
        E_hat, b_hat = theta
        return float(E_hat), float(b_hat), 0.0, int(mask.sum())

def learn_E_rpm_binned(sim, use_alpha_est=True, num_bins=8, min_samples=5):
    """
    Learn RPM-dependent effectiveness: E(x) = E0 + E1 * x
    where x = RPM² / 1e6
    
    Uses bin-averaging method:
    1. Partition samples by x into bins
    2. Compute average g = alpha/u in each bin
    3. Fit line through bin centers vs bin averages
    
    Returns: E0, E1, bin_info dict, n_samples_used
    """
    mask = sim["intent"] & sim["unsat"]
    
    if use_alpha_est:
        alpha = sim["alpha_est"][mask]
    else:
        alpha = sim["alpha_true"][mask]
    
    ua = sim["u_a"][mask]
    rpm_sq = sim["rpm_sq_avg"][mask]
    
    # x = RPM² / 1e6 (normalized)
    x = rpm_sq / 1e6
    
    # Filter out samples with too-small commands (avoid noisy g estimates)
    valid = np.abs(ua) > 1.0
    alpha = alpha[valid]
    ua = ua[valid]
    x = x[valid]
    
    if len(alpha) == 0:
        return 1.0, 0.0, {"bins": [], "counts": [], "g_avg": []}, 0
    
    # Compute per-sample effectiveness: g = alpha / u
    g = alpha / ua
    
    # Determine bin edges based on x range
    x_min, x_max = x.min(), x.max()
    # Add small margin to ensure all samples fall within bins
    bin_width = (x_max - x_min + 0.01) / num_bins
    if bin_width < 0.01:
        bin_width = 0.01  # Minimum bin width
    
    # Bin the samples
    bin_sums = np.zeros(num_bins)
    bin_counts = np.zeros(num_bins, dtype=int)
    
    for i in range(len(x)):
        bin_idx = int((x[i] - x_min) / bin_width)
        bin_idx = min(bin_idx, num_bins - 1)  # Handle edge case
        bin_sums[bin_idx] += g[i]
        bin_counts[bin_idx] += 1
    
    # Compute bin averages and centers for bins with enough samples
    bin_centers = []
    bin_averages = []
    bin_weights = []
    
    for i in range(num_bins):
        if bin_counts[i] >= min_samples:
            center = x_min + (i + 0.5) * bin_width
            avg = bin_sums[i] / bin_counts[i]
            bin_centers.append(center)
            bin_averages.append(avg)
            bin_weights.append(bin_counts[i])
    
    bin_info = {
        "x_min": x_min,
        "x_max": x_max,
        "bin_width": bin_width,
        "counts": bin_counts.tolist(),
        "centers": bin_centers,
        "averages": bin_averages,
        "n_active_bins": len(bin_centers),
    }
    
    # Need at least 2 bins to fit a line
    if len(bin_centers) < 2:
        # Fall back to constant E (average of all valid bins)
        if len(bin_averages) > 0:
            E0 = np.mean(bin_averages)
        else:
            E0 = 1.0
        return E0, 0.0, bin_info, int(valid.sum())
    
    # Weighted least squares fit: g = E0 + E1 * x
    bin_centers = np.array(bin_centers)
    bin_averages = np.array(bin_averages)
    bin_weights = np.array(bin_weights)
    
    # Weighted normal equations
    sum_w = np.sum(bin_weights)
    sum_wx = np.sum(bin_weights * bin_centers)
    sum_wy = np.sum(bin_weights * bin_averages)
    sum_wxx = np.sum(bin_weights * bin_centers ** 2)
    sum_wxy = np.sum(bin_weights * bin_centers * bin_averages)
    
    det = sum_w * sum_wxx - sum_wx * sum_wx
    if abs(det) < 1e-9:
        E0 = np.mean(bin_averages)
        return E0, 0.0, bin_info, int(valid.sum())
    
    E0 = (sum_wxx * sum_wy - sum_wx * sum_wxy) / det
    E1 = (sum_w * sum_wxy - sum_wx * sum_wy) / det
    
    return float(E0), float(E1), bin_info, int(valid.sum())

def ramp_metrics(sim, label, alpha_min):
    # Evaluate over intent windows; report both all-intent and (intent & unsat)
    t = sim["t"]
    dt = t[1] - t[0]
    e = sim["w_sp"] - sim["w_true"]
    intent = np.abs(sim["alpha_cmd"]) > alpha_min
    unsat = sim["unsat"]

    def stats(mask):
        if mask.sum() == 0:
            return dict(n=0, rms_err=np.nan, iae=np.nan, rms_pid=np.nan, sat_frac=np.nan)
        rms_err = np.sqrt(np.mean(e[mask] ** 2))
        iae = np.sum(np.abs(e[mask])) * dt
        rms_pid = np.sqrt(np.mean(sim["pid"][mask] ** 2))
        sat_frac = 1.0 - np.mean(unsat[mask])  # within this mask, how often saturated
        return dict(n=int(mask.sum()), rms_err=float(rms_err), iae=float(iae), rms_pid=float(rms_pid), sat_frac=float(sat_frac))

    return {
        "label": label,
        "intent_all": stats(intent),
        "intent_unsat": stats(intent & unsat),
        "samples_used_for_learning": int((intent & unsat).sum()),
    }

# -------------------------
# Run experiment
# Parameter reasoning:
# - wmax = 400 deg/s (target max rate)
# - u_axis_max = 45 (available authority from mixer)
# - At steady state: E*u = b*w, so E*45 = b*400
# - Choose E = 20.0 (high effectiveness for responsive plant)
# - Then b = E*45/400 = 20*45/400 = 2.25 → use 2.0 for margin
# - Steady state check: E*u_max=900, b*wmax=800 ✓ achievable

cfg = dict(
    dt=0.001, T=2.0,
    Tr=0.35, Th=0.40, wmax=400.0,     # deg/s
    E_true=20.0, b_true=2.0, tau_m=0.010,  # Much more responsive plant
    Kp=1.5, Ki=4.0, Kd=0.020,         # Much higher gains for aggressive tracking
    D_on_error=True,                   # Will be varied in comparison
    kff=1.0,
    # motor mix / saturation
    m0=50.0, u_axis_max=45.0, sat_eps=3.0,
    # gating
    alpha_min=200.0,
    # measurement pipeline
    gyro_noise_std=3.0,          # rate noise (deg/s)
    gyro_lpf_hz=150.0,
    alpha_est_lpf_hz=80.0,
    alpha_cmd_lpf_hz=80.0,
    seed=1,
    # Gyro filter chain (realistic Betaflight-like delay)
    # Multiple LPF stages simulate hardware + software filtering
    # Total group delay ~4-6ms at typical settings
    use_gyro_delay=True,            # Enable realistic feedback delay
    gyro_filter_lpf1_hz=250.0,      # Hardware LPF (gyro chip) - O1
    gyro_filter_lpf2_hz=150.0,      # Software gyro_lpf1 - O1
    gyro_filter_lpf2_o2_hz=500.0,   # Software gyro_lpf2 - O2 biquad
    gyro_filter_lpf3_hz=100.0,      # Dynamic notch group delay approximation
    # D-term filtering (adds extra delay to D path)
    dterm_lpf1_hz=75.0,             # Betaflight dterm_lpf1 - O1
    dterm_lpf2_hz=250.0,            # Betaflight dterm_lpf2 - O2 biquad
    # initial model guess (for fixed FF - intentionally wrong by ~40%)
    E_hat=12.0, b_hat=1.0,
    use_ff=False,
    # RPM² nonlinearity (new)
    use_rpm_sq=False,            # Will be toggled for comparison
    rpm_max=25000.0,             # Max motor RPM
    tau_rpm=0.015,               # Motor RPM time constant
    k_torque=1.0,                # Torque coefficient
)

# =========================================================================
# PID QUALITY STRESS TEST: Good PIDs vs Bad PIDs
# =========================================================================
# The concern: if baseline is already good, learning has little to offer.
# Also: does learning actually help, or make things worse?
# 
# We test with:
#   - Good PIDs: Kp=1.5, Ki=4.0, Kd=0.020 (well-tuned)
#   - Bad PIDs:  Kp=0.4, Ki=1.0, Kd=0.005 (weak, sluggish response)
#   - Linear vs RPM² motor model
#   - D_on_error vs D_on_gyro

print("\n" + "="*70)
print("PID QUALITY STRESS TEST: Does Learning Actually Help?")
print("="*70)
print("Testing ALL combinations: PIDs x Motor Model x D-term Mode")

# Good PIDs (current config)
cfg_good_pid = cfg.copy()

# Bad PIDs (much weaker - sluggish response, larger errors)
cfg_bad_pid = cfg.copy()
cfg_bad_pid["Kp"] = 0.4      # Was 1.5 - now ~4x weaker
cfg_bad_pid["Ki"] = 1.0      # Was 4.0 - now 4x weaker
cfg_bad_pid["Kd"] = 0.005    # Was 0.020 - now 4x weaker

def run_pid_quality_test(cfg_test, label, use_rpm_binned=False):
    """Test baseline vs learned FF for a given PID configuration."""
    # Baseline (no FF)
    cfg_base = cfg_test.copy()
    cfg_base["use_ff"] = False
    sim_base = simulate(cfg_base)
    m_base = ramp_metrics(sim_base, f"{label}_baseline", cfg_test["alpha_min"])
    
    if use_rpm_binned and cfg_test.get("use_rpm_sq", False):
        # Use RPM-binned learning for RPM² model
        E0_learned, E1_learned, bin_info, n_samples = learn_E_rpm_binned(sim_base)
        
        # Apply RPM-binned learned FF
        cfg_ff = cfg_test.copy()
        cfg_ff["use_ff"] = True
        cfg_ff["use_rpm_dependent_E"] = True
        cfg_ff["E0_hat"] = E0_learned
        cfg_ff["E1_hat"] = E1_learned
        cfg_ff["b_hat"] = 0.5  # Use reasonable default
        sim_ff = simulate(cfg_ff)
        m_ff = ramp_metrics(sim_ff, f"{label}_learned_ff", cfg_test["alpha_min"])
        
        return {
            "baseline": {"sim": sim_base, "metrics": m_base},
            "learned": {"sim": sim_ff, "metrics": m_ff, "E0": E0_learned, "E1": E1_learned},
            "n_samples": n_samples,
        }
    else:
        # Standard constant-E learning
        E_learned, b_learned, c_learned, n_samples = learn_E_b(sim_base, use_alpha_est=True, add_intercept=True)
        
        # Apply learned FF
        cfg_ff = cfg_test.copy()
        cfg_ff["use_ff"] = True
        cfg_ff["E_hat"] = E_learned
        cfg_ff["b_hat"] = b_learned
        sim_ff = simulate(cfg_ff)
        m_ff = ramp_metrics(sim_ff, f"{label}_learned_ff", cfg_test["alpha_min"])
        
        return {
            "baseline": {"sim": sim_base, "metrics": m_base},
            "learned": {"sim": sim_ff, "metrics": m_ff, "E": E_learned, "b": b_learned},
            "n_samples": n_samples,
        }

# Store all results for plotting
pid_stress_results = {}

# Simplified test matrix: Only RPM² motor model + D_on_error
# 2 cases: Good PIDs vs Bad PIDs
test_configs = [
    ("Good PIDs", cfg_good_pid),
    ("Bad PIDs", cfg_bad_pid),
]

print("\n" + "-"*70)
print(f"{'Config':<35} {'Baseline':>10} {'Learned':>10} {'Improve':>10}")
print("-"*70)

all_results = []
for pid_name, pid_cfg in test_configs:
    # Use LINEAR motor model for clearer demonstration of FF benefit
    # (RPM² model has complex nonlinearities that obscure the delay compensation benefit)
    test_cfg = pid_cfg.copy()
    test_cfg["use_rpm_sq"] = False  # Linear motor model
    test_cfg["D_on_error"] = True
    
    label = f"{pid_name}_Linear_D_on_error"
    
    # Run test with standard learning (not RPM-binned since linear)
    result = run_pid_quality_test(test_cfg, label, use_rpm_binned=False)
    
    base_rms = result["baseline"]["metrics"]["intent_all"]["rms_err"]
    learn_rms = result["learned"]["metrics"]["intent_all"]["rms_err"]
    improve = (base_rms - learn_rms) / base_rms * 100
    
    config_label = f"{pid_name} + Linear + D_on_error"
    status = "[OK]" if improve > 0 else "[WORSE!]"
    print(f"  {config_label:<33} {base_rms:>8.2f} {learn_rms:>10.2f} {improve:>+8.1f}% {status}")
    
    all_results.append({
        "pid": pid_name,
        "motor": "Linear",
        "d_mode": "D_on_error",
        "baseline_rms": base_rms,
        "learned_rms": learn_rms,
        "improvement": improve,
    })
    
    # Store for plotting
    key = pid_name  # Simplified key
    pid_stress_results[key] = result

print("-"*70)

# Summary
improved = sum(1 for r in all_results if r["improvement"] > 0)
total = len(all_results)
print(f"\nSUMMARY: {improved}/{total} improved")

# For backward compatibility
good_pid_results = pid_stress_results.get("Good PIDs")
bad_pid_results = pid_stress_results.get("Bad PIDs")

if good_pid_results:
    g_base = good_pid_results["baseline"]["metrics"]["intent_all"]["rms_err"]
    g_learned = good_pid_results["learned"]["metrics"]["intent_all"]["rms_err"]
    g_improve = (g_base - g_learned) / g_base * 100
    print(f"  Good PIDs: {g_base:.2f} -> {g_learned:.2f} ({g_improve:+.1f}%)")

if bad_pid_results:
    b_base = bad_pid_results["baseline"]["metrics"]["intent_all"]["rms_err"]
    b_learned = bad_pid_results["learned"]["metrics"]["intent_all"]["rms_err"]
    b_improve = (b_base - b_learned) / b_base * 100
    print(f"  Bad PIDs:  {b_base:.2f} -> {b_learned:.2f} ({b_improve:+.1f}%)")

print("="*70)

# =========================================================================
# RPM² NONLINEARITY COMPARISON
# =========================================================================
print("\n" + "="*70)
print("RPM² NONLINEARITY COMPARISON: Linear vs RPM² Thrust Model")
print("="*70)

def run_full_comparison(cfg_base, use_rpm_sq, label):
    """Run baseline, fixed FF, and learned FF with given motor model."""
    cfg_test = cfg_base.copy()
    cfg_test["use_rpm_sq"] = use_rpm_sq
    
    # Baseline (no FF)
    cfg_test["use_ff"] = False
    sim_base = simulate(cfg_test)
    m_base = ramp_metrics(sim_base, f"{label}_baseline", cfg_base["alpha_min"])
    
    # Learn from baseline
    E_learned, b_learned, c_learned, n_samples = learn_E_b(sim_base, use_alpha_est=True, add_intercept=True)
    
    # Fixed FF (wrong guess)
    cfg_test["use_ff"] = True
    cfg_test["E_hat"] = cfg_base["E_hat"]
    cfg_test["b_hat"] = cfg_base["b_hat"]
    sim_fixed = simulate(cfg_test)
    m_fixed = ramp_metrics(sim_fixed, f"{label}_fixed", cfg_base["alpha_min"])
    
    # Learned FF
    cfg_test["E_hat"] = E_learned
    cfg_test["b_hat"] = b_learned
    sim_learned = simulate(cfg_test)
    m_learned = ramp_metrics(sim_learned, f"{label}_learned", cfg_base["alpha_min"])
    
    return {
        "label": label,
        "baseline": {"sim": sim_base, "metrics": m_base},
        "fixed": {"sim": sim_fixed, "metrics": m_fixed},
        "learned": {"sim": sim_learned, "metrics": m_learned},
        "E_learned": E_learned,
        "b_learned": b_learned,
    }

# Run both models
linear_results = run_full_comparison(cfg, use_rpm_sq=False, label="Linear")
rpm_sq_results = run_full_comparison(cfg, use_rpm_sq=True, label="RPM²")

print("\n--- LINEAR Motor Model ---")
print(f"  Learned: E={linear_results['E_learned']:.3f}, b={linear_results['b_learned']:.3f}")
for scenario in ["baseline", "fixed", "learned"]:
    m = linear_results[scenario]["metrics"]["intent_all"]
    print(f"  {scenario:10s}: RMS_err={m['rms_err']:6.2f}, IAE={m['iae']:6.2f}, PID={m['rms_pid']:6.2f}")

print("\n--- RPM² Motor Model (Constant E Learning - WRONG) ---")
print(f"  Learned: E={rpm_sq_results['E_learned']:.3f}, b={rpm_sq_results['b_learned']:.3f}")
for scenario in ["baseline", "fixed", "learned"]:
    m = rpm_sq_results[scenario]["metrics"]["intent_all"]
    print(f"  {scenario:10s}: RMS_err={m['rms_err']:6.2f}, IAE={m['iae']:6.2f}, PID={m['rms_pid']:6.2f}")

# =========================================================================
# RPM-BINNED LEARNING (the correct approach for RPM² model)
# =========================================================================
print("\n" + "-"*70)
print("RPM² Motor Model with RPM-BINNED Learning (CORRECT approach)")
print("-"*70)

# Run baseline with RPM² model
cfg_rpm = cfg.copy()
cfg_rpm["use_rpm_sq"] = True
cfg_rpm["use_ff"] = False
sim_rpm_base = simulate(cfg_rpm)

# Learn E(x) = E0 + E1 * x using binned method
E0_learned, E1_learned, bin_info, n_samples = learn_E_rpm_binned(
    sim_rpm_base, use_alpha_est=True, num_bins=8, min_samples=5
)

print(f"\n  Binned learning results:")
print(f"    E(x) = {E0_learned:.3f} + {E1_learned:.4f} * x   (x = RPM²/1e6)")
print(f"    Active bins: {bin_info['n_active_bins']} / 8")
print(f"    Samples used: {n_samples}")
print(f"    x range: [{bin_info['x_min']:.2f}, {bin_info['x_max']:.2f}]")

# Run with RPM-dependent learned FF
cfg_rpm_learned = cfg_rpm.copy()
cfg_rpm_learned["use_ff"] = True
cfg_rpm_learned["use_rpm_dependent_E"] = True
cfg_rpm_learned["E0_hat"] = E0_learned
cfg_rpm_learned["E1_hat"] = E1_learned
cfg_rpm_learned["b_hat"] = 0.0  # Ignoring damping as requested
sim_rpm_learned = simulate(cfg_rpm_learned)

# Metrics
m_rpm_base = ramp_metrics(sim_rpm_base, "rpm_baseline", cfg["alpha_min"])
m_rpm_learned = ramp_metrics(sim_rpm_learned, "rpm_binned_learned", cfg["alpha_min"])

print(f"\n  Performance comparison:")
print(f"    Baseline (no FF):     RMS_err={m_rpm_base['intent_all']['rms_err']:.2f}")
print(f"    Constant E learned:   RMS_err={rpm_sq_results['learned']['metrics']['intent_all']['rms_err']:.2f} (WORSE)")
print(f"    RPM-binned E learned: RMS_err={m_rpm_learned['intent_all']['rms_err']:.2f}")

base_rms = m_rpm_base['intent_all']['rms_err']
const_rms = rpm_sq_results['learned']['metrics']['intent_all']['rms_err']
binned_rms = m_rpm_learned['intent_all']['rms_err']

print(f"\n  Improvement from baseline:")
print(f"    Constant E: {(base_rms - const_rms) / base_rms * 100:+.1f}%")
print(f"    Binned E:   {(base_rms - binned_rms) / base_rms * 100:+.1f}%")

# Store for plotting
rpm_binned_results = {
    "baseline": {"sim": sim_rpm_base, "metrics": m_rpm_base},
    "learned": {"sim": sim_rpm_learned, "metrics": m_rpm_learned},
    "E0": E0_learned,
    "E1": E1_learned,
    "bin_info": bin_info,
}

# Calculate improvement ratios
print("\n--- Learning Benefit Comparison ---")
for model_name, results in [("Linear", linear_results), ("RPM^2", rpm_sq_results)]:
    base_rms = results["baseline"]["metrics"]["intent_all"]["rms_err"]
    learned_rms = results["learned"]["metrics"]["intent_all"]["rms_err"]
    improvement = (base_rms - learned_rms) / base_rms * 100
    print(f"  {model_name:8s}: {base_rms:.2f} -> {learned_rms:.2f} ({improvement:.1f}% improvement)")

print(f"  RPM^2+bin: {m_rpm_base['intent_all']['rms_err']:.2f} -> {m_rpm_learned['intent_all']['rms_err']:.2f} "
      f"({(m_rpm_base['intent_all']['rms_err'] - m_rpm_learned['intent_all']['rms_err']) / m_rpm_base['intent_all']['rms_err'] * 100:.1f}% improvement)")

print("="*70)

# -------------------------
# D-TERM COMPARISON: D-on-error vs D-on-gyro
# -------------------------
print("\n" + "="*70)
print("D-TERM COMPARISON: D-on-error vs D-on-gyro (measurement)")
print("="*70)

def run_d_term_comparison(cfg_base, label_prefix, use_ff, E_hat=None, b_hat=None):
    """Run same config with both D modes and compare."""
    results = {}
    
    for d_on_error in [True, False]:
        d_label = "D_on_error" if d_on_error else "D_on_gyro"
        cfg_test = cfg_base.copy()
        cfg_test["D_on_error"] = d_on_error
        cfg_test["use_ff"] = use_ff
        if E_hat is not None:
            cfg_test["E_hat"] = E_hat
        if b_hat is not None:
            cfg_test["b_hat"] = b_hat
        
        sim = simulate(cfg_test)
        metrics = ramp_metrics(sim, f"{label_prefix}_{d_label}", cfg_base["alpha_min"])
        
        # Also learn from this run
        E_learned, b_learned, c_learned, n_samples = learn_E_b(sim, use_alpha_est=True, add_intercept=True)
        
        results[d_label] = {
            "sim": sim,
            "metrics": metrics,
            "E_learned": E_learned,
            "b_learned": b_learned,
        }
    
    return results

# Compare for baseline (no FF)
print("\n--- Baseline (no FF) ---")
baseline_cmp = run_d_term_comparison(cfg, "baseline", use_ff=False)
for d_mode, data in baseline_cmp.items():
    m = data["metrics"]["intent_all"]
    print(f"  {d_mode:12s}: RMS_err={m['rms_err']:6.2f}, IAE={m['iae']:6.2f}, PID={m['rms_pid']:6.2f}")

# Compare for fixed FF (wrong guess)  
print("\n--- Fixed FF (E=12, b=1 - wrong guess) ---")
fixed_ff_cmp = run_d_term_comparison(cfg, "fixed_ff", use_ff=True, E_hat=12.0, b_hat=1.0)
for d_mode, data in fixed_ff_cmp.items():
    m = data["metrics"]["intent_all"]
    print(f"  {d_mode:12s}: RMS_err={m['rms_err']:6.2f}, IAE={m['iae']:6.2f}, PID={m['rms_pid']:6.2f}")

# Learn from baseline with D_on_error, then test learned FF with both D modes
E_learned_err = baseline_cmp["D_on_error"]["E_learned"]
b_learned_err = baseline_cmp["D_on_error"]["b_learned"]
E_learned_gyro = baseline_cmp["D_on_gyro"]["E_learned"]
b_learned_gyro = baseline_cmp["D_on_gyro"]["b_learned"]

print(f"\n--- Learning quality comparison ---")
print(f"  D_on_error learned: E={E_learned_err:.3f} (true: 20.0), b={b_learned_err:.3f} (true: 2.0)")
print(f"  D_on_gyro  learned: E={E_learned_gyro:.3f} (true: 20.0), b={b_learned_gyro:.3f} (true: 2.0)")

# Compare learned FF with both D modes
print("\n--- Learned FF (from D_on_error baseline) ---")
learned_ff_cmp = run_d_term_comparison(cfg, "learned_ff", use_ff=True, E_hat=E_learned_err, b_hat=b_learned_err)
for d_mode, data in learned_ff_cmp.items():
    m = data["metrics"]["intent_all"]
    print(f"  {d_mode:12s}: RMS_err={m['rms_err']:6.2f}, IAE={m['iae']:6.2f}, PID={m['rms_pid']:6.2f}")

print("="*70)

# For the rest of the script, use D_on_error as default
# Baseline
sim0 = simulate(cfg)

# Learn from baseline
E1, b1, c1, n1 = learn_E_b(sim0, use_alpha_est=True, add_intercept=True)

# Fixed FF run (wrong-ish guess)
cfg_fixed = cfg.copy()
cfg_fixed.update(use_ff=True, E_hat=cfg["E_hat"], b_hat=cfg["b_hat"])
sim_fixed = simulate(cfg_fixed)

# Learned FF run
cfg_learned = cfg.copy()
cfg_learned.update(use_ff=True, E_hat=E1, b_hat=b1)
sim_learned = simulate(cfg_learned)

# -------------------------
# Iterative learning: run multiple cycles to show convergence
# -------------------------
NUM_ITERATIONS = 10

# Track learning history
iter_history = {
    "iteration": [],
    "E_hat": [],
    "b_hat": [],
    "rms_err": [],
    "iae": [],
    "rms_pid": [],
    "E_error_pct": [],
    "b_error_pct": [],
}

# Start with initial wrong guess
E_iter = cfg["E_hat"]
b_iter = cfg["b_hat"]

print("\n" + "="*70)
print("ITERATIVE LEARNING CONVERGENCE")
print("="*70)
print(f"Truth: E={cfg['E_true']:.3f}, b={cfg['b_true']:.3f}")
print(f"Initial guess: E={E_iter:.3f}, b={b_iter:.3f}")
print("-"*70)

for i in range(NUM_ITERATIONS):
    # Run with current estimates
    cfg_iter = cfg.copy()
    cfg_iter.update(use_ff=True, E_hat=E_iter, b_hat=b_iter)
    sim_iter = simulate(cfg_iter)
    
    # Get metrics
    m_iter = ramp_metrics(sim_iter, f"iter_{i}", cfg["alpha_min"])
    rms_err = m_iter["intent_all"]["rms_err"]
    iae = m_iter["intent_all"]["iae"]
    rms_pid = m_iter["intent_all"]["rms_pid"]
    
    # Calculate errors
    E_error = 100 * abs(E_iter - cfg["E_true"]) / cfg["E_true"]
    b_error = 100 * abs(b_iter - cfg["b_true"]) / cfg["b_true"]
    
    # Log
    iter_history["iteration"].append(i)
    iter_history["E_hat"].append(E_iter)
    iter_history["b_hat"].append(b_iter)
    iter_history["rms_err"].append(rms_err)
    iter_history["iae"].append(iae)
    iter_history["rms_pid"].append(rms_pid)
    iter_history["E_error_pct"].append(E_error)
    iter_history["b_error_pct"].append(b_error)
    
    print(f"Iter {i:2d}: E={E_iter:7.3f} ({E_error:5.1f}% err), b={b_iter:6.3f} ({b_error:5.1f}% err) | "
          f"RMS_e={rms_err:6.2f}, IAE={iae:6.2f}, PID={rms_pid:6.2f}")
    
    # Learn from this run's data
    E_new, b_new, c_new, n_samples = learn_E_b(sim_iter, use_alpha_est=True, add_intercept=True)
    
    # Blend new learning with old (exponential smoothing for stability)
    alpha_learn = 0.5  # Learning rate - how much to trust new estimate
    E_iter = (1 - alpha_learn) * E_iter + alpha_learn * E_new
    b_iter = (1 - alpha_learn) * b_iter + alpha_learn * b_new

print("-"*70)
print(f"Final:  E={E_iter:.3f} (truth: {cfg['E_true']:.3f}), b={b_iter:.3f} (truth: {cfg['b_true']:.3f})")
print("="*70)

# Metrics
m0_stats = ramp_metrics(sim0, "baseline (no FF)", cfg["alpha_min"])
mF_stats = ramp_metrics(sim_fixed, "fixed FF (wrong guess)", cfg["alpha_min"])
mL_stats = ramp_metrics(sim_learned, "learned FF (from baseline)", cfg["alpha_min"])

print("Truth:    E_true=%.5f, b_true=%.3f" % (cfg["E_true"], cfg["b_true"]))
print("Learned:  E_hat =%.5f, b_hat =%.3f, c_hat=%.3f (used %d samples)" % (E1, b1, c1, n1))
print("Initial:  E_hat0=%.5f, b_hat0=%.3f" % (cfg["E_hat"], cfg["b_hat"]))

def print_metrics(ms):
    ia = ms["intent_all"]
    iu = ms["intent_unsat"]
    print("\n%s" % ms["label"])
    print("  intent(all):     n=%d  rms_err=%.3f  IAE=%.3f  rms_pid=%.3f  sat_frac=%.2f"
          % (ia["n"], ia["rms_err"], ia["iae"], ia["rms_pid"], ia["sat_frac"]))
    print("  intent(unsat):   n=%d  rms_err=%.3f  IAE=%.3f  rms_pid=%.3f  sat_frac=%.2f"
          % (iu["n"], iu["rms_err"], iu["iae"], iu["rms_pid"], iu["sat_frac"]))

print_metrics(m0_stats)
print_metrics(mF_stats)
print_metrics(mL_stats)

# -------------------------
# Plots (separate figures, no manual colors)
def plot_rates():
    plt.figure()
    plt.title("Rate tracking")
    plt.plot(sim0["t"], sim0["w_sp"], label="setpoint")
    plt.plot(sim0["t"], sim0["w_true"], label="baseline")
    plt.plot(sim_fixed["t"], sim_fixed["w_true"], label="fixed FF")
    plt.plot(sim_learned["t"], sim_learned["w_true"], label="learned FF")
    plt.xlabel("t (s)")
    plt.ylabel("rate ω")
    plt.legend()
    plt.grid(True)

def plot_error():
    plt.figure()
    plt.title("Rate error (setpoint - ω)")
    plt.plot(sim0["t"], sim0["w_sp"] - sim0["w_true"], label="baseline")
    plt.plot(sim_fixed["t"], sim_fixed["w_sp"] - sim_fixed["w_true"], label="fixed FF")
    plt.plot(sim_learned["t"], sim_learned["w_sp"] - sim_learned["w_true"], label="learned FF")
    plt.xlabel("t (s)")
    plt.ylabel("error e")
    plt.legend()
    plt.grid(True)

def plot_pid_ff():
    plt.figure()
    plt.title("Controller contributions (axis command components)")
    plt.plot(sim0["t"], sim0["pid"], label="PID baseline")
    plt.plot(sim_fixed["t"], sim_fixed["pid"], label="PID fixed FF")
    plt.plot(sim_learned["t"], sim_learned["pid"], label="PID learned FF")
    plt.xlabel("t (s)")
    plt.ylabel("u_pid")
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.title("Feedforward term (u_ff)")
    plt.plot(sim_fixed["t"], sim_fixed["u_ff"], label="fixed FF u_ff")
    plt.plot(sim_learned["t"], sim_learned["u_ff"], label="learned FF u_ff")
    plt.xlabel("t (s)")
    plt.ylabel("u_ff")
    plt.legend()
    plt.grid(True)

def plot_saturation_and_gates():
    plt.figure()
    plt.title("Learning gates and saturation (1=true)")
    plt.plot(sim0["t"], sim0["intent"].astype(float), label="intent |alpha_cmd|>min")
    plt.plot(sim0["t"], sim0["unsat"].astype(float), label="unsat (all motors away from rails)")
    plt.plot(sim0["t"], (sim0["intent"] & sim0["unsat"]).astype(float), label="learn mask")
    plt.xlabel("t (s)")
    plt.ylabel("mask")
    plt.legend()
    plt.ylim(-0.1, 1.1)
    plt.grid(True)

def plot_alpha_fit_quality():
    # Compare alpha_est vs model prediction using learned params, on learning mask
    mask = sim0["intent"] & sim0["unsat"]
    t = sim0["t"]
    alpha = sim0["alpha_est"]
    pred = E1 * sim0["u_a"] - b1 * sim0["w_meas_f"] + c1
    plt.figure()
    plt.title("Learning fit check (baseline run): α_est vs α_pred on learn mask")
    plt.plot(t[mask], alpha[mask], label="alpha_est")
    plt.plot(t[mask], pred[mask], label="alpha_pred (learned model)")
    plt.xlabel("t (s)")
    plt.ylabel("angular accel α")
    plt.legend()
    plt.grid(True)

def plot_convergence():
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Iterative Learning Convergence", fontsize=14)
    
    iters = iter_history["iteration"]
    
    # Plot 1: Parameter convergence
    ax1 = axes[0, 0]
    ax1.axhline(cfg["E_true"], color='k', linestyle='--', alpha=0.5, label=f'E_true={cfg["E_true"]}')
    ax1.plot(iters, iter_history["E_hat"], 'o-', label='E_hat (learned)')
    ax1.set_xlabel("Iteration")
    ax1.set_ylabel("Effectiveness E")
    ax1.set_title("E convergence")
    ax1.legend()
    ax1.grid(True)
    
    ax1b = axes[0, 1]
    ax1b.axhline(cfg["b_true"], color='k', linestyle='--', alpha=0.5, label=f'b_true={cfg["b_true"]}')
    ax1b.plot(iters, iter_history["b_hat"], 's-', label='b_hat (learned)')
    ax1b.set_xlabel("Iteration")
    ax1b.set_ylabel("Damping b")
    ax1b.set_title("b convergence")
    ax1b.legend()
    ax1b.grid(True)
    
    # Plot 2: Error metrics convergence
    ax2 = axes[1, 0]
    ax2.plot(iters, iter_history["rms_err"], 'o-', label='RMS error (deg/s)')
    ax2.plot(iters, iter_history["iae"], 's-', label='IAE')
    ax2.axhline(m0_stats["intent_all"]["rms_err"], color='r', linestyle=':', alpha=0.7, label='baseline RMS')
    ax2.set_xlabel("Iteration")
    ax2.set_ylabel("Error metric")
    ax2.set_title("Tracking error reduction")
    ax2.legend()
    ax2.grid(True)
    
    # Plot 3: PID effort reduction
    ax3 = axes[1, 1]
    ax3.plot(iters, iter_history["rms_pid"], 'o-', label='RMS PID effort')
    ax3.axhline(m0_stats["intent_all"]["rms_pid"], color='r', linestyle=':', alpha=0.7, label='baseline PID')
    ax3.set_xlabel("Iteration")
    ax3.set_ylabel("PID effort")
    ax3.set_title("PID effort reduction (FF taking over)")
    ax3.legend()
    ax3.grid(True)
    
    plt.tight_layout()

def plot_d_term_comparison():
    """Plot comparison of D-on-error vs D-on-gyro for different FF scenarios."""
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle("D-on-error vs D-on-gyro Comparison", fontsize=14)
    
    t = baseline_cmp["D_on_error"]["sim"]["t"]
    w_sp = baseline_cmp["D_on_error"]["sim"]["w_sp"]
    
    # Row 1: Rate tracking
    scenarios = [
        ("Baseline (no FF)", baseline_cmp),
        ("Fixed FF (wrong)", fixed_ff_cmp),
        ("Learned FF", learned_ff_cmp),
    ]
    
    for col, (title, cmp_data) in enumerate(scenarios):
        ax = axes[0, col]
        ax.plot(t, w_sp, 'k--', alpha=0.5, label='setpoint')
        ax.plot(t, cmp_data["D_on_error"]["sim"]["w_true"], label='D_on_error')
        ax.plot(t, cmp_data["D_on_gyro"]["sim"]["w_true"], label='D_on_gyro')
        ax.set_xlabel("t (s)")
        ax.set_ylabel("rate ω (deg/s)")
        ax.set_title(f"{title}")
        ax.legend(fontsize=8)
        ax.grid(True)
    
    # Row 2: Tracking error
    for col, (title, cmp_data) in enumerate(scenarios):
        ax = axes[1, col]
        err_d_err = cmp_data["D_on_error"]["sim"]["w_sp"] - cmp_data["D_on_error"]["sim"]["w_true"]
        err_d_gyro = cmp_data["D_on_gyro"]["sim"]["w_sp"] - cmp_data["D_on_gyro"]["sim"]["w_true"]
        ax.plot(t, err_d_err, label='D_on_error')
        ax.plot(t, err_d_gyro, label='D_on_gyro')
        ax.axhline(0, color='k', linestyle='--', alpha=0.3)
        ax.set_xlabel("t (s)")
        ax.set_ylabel("error (deg/s)")
        ax.set_title(f"Error: {title}")
        ax.legend(fontsize=8)
        ax.grid(True)
    
    plt.tight_layout()

def plot_d_term_summary_bar():
    """Bar chart summary of D-term comparison."""
    fig, axes = plt.subplots(1, 3, figsize=(14, 5))
    fig.suptitle("D-term Mode Impact Summary", fontsize=14)
    
    scenarios = ["Baseline\n(no FF)", "Fixed FF\n(wrong)", "Learned FF"]
    cmp_list = [baseline_cmp, fixed_ff_cmp, learned_ff_cmp]
    
    x = np.arange(len(scenarios))
    width = 0.35
    
    # RMS Error comparison
    ax1 = axes[0]
    rms_err = [c["D_on_error"]["metrics"]["intent_all"]["rms_err"] for c in cmp_list]
    rms_gyro = [c["D_on_gyro"]["metrics"]["intent_all"]["rms_err"] for c in cmp_list]
    ax1.bar(x - width/2, rms_err, width, label='D_on_error')
    ax1.bar(x + width/2, rms_gyro, width, label='D_on_gyro')
    ax1.set_ylabel('RMS Error (deg/s)')
    ax1.set_title('Tracking Error')
    ax1.set_xticks(x)
    ax1.set_xticklabels(scenarios)
    ax1.legend()
    ax1.grid(True, axis='y')
    
    # PID Effort comparison
    ax2 = axes[1]
    pid_err = [c["D_on_error"]["metrics"]["intent_all"]["rms_pid"] for c in cmp_list]
    pid_gyro = [c["D_on_gyro"]["metrics"]["intent_all"]["rms_pid"] for c in cmp_list]
    ax2.bar(x - width/2, pid_err, width, label='D_on_error')
    ax2.bar(x + width/2, pid_gyro, width, label='D_on_gyro')
    ax2.set_ylabel('RMS PID Effort')
    ax2.set_title('Controller Effort')
    ax2.set_xticks(x)
    ax2.set_xticklabels(scenarios)
    ax2.legend()
    ax2.grid(True, axis='y')
    
    # Improvement from D_on_gyro to D_on_error (negative = D_on_error is worse)
    ax3 = axes[2]
    improvement = [(g - e) / g * 100 for e, g in zip(rms_err, rms_gyro)]
    colors = ['green' if v > 0 else 'red' for v in improvement]
    ax3.bar(x, improvement, width*1.5, color=colors)
    ax3.axhline(0, color='k', linestyle='-', linewidth=0.5)
    ax3.set_ylabel('% Improvement')
    ax3.set_title('D_on_error vs D_on_gyro\n(positive = D_on_error better)')
    ax3.set_xticks(x)
    ax3.set_xticklabels(scenarios)
    ax3.grid(True, axis='y')
    
    plt.tight_layout()

def plot_rpm_sq_comparison():
    """Plot comparison of Linear vs RPM² motor models."""
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle("Linear vs RPM² Motor Model Comparison", fontsize=14)
    
    t = linear_results["baseline"]["sim"]["t"]
    w_sp = linear_results["baseline"]["sim"]["w_sp"]
    
    # Row 1: Rate tracking comparison
    for col, scenario in enumerate(["baseline", "fixed", "learned"]):
        ax = axes[0, col]
        ax.plot(t, w_sp, 'k--', alpha=0.5, label='setpoint')
        ax.plot(t, linear_results[scenario]["sim"]["w_true"], label='Linear')
        ax.plot(t, rpm_sq_results[scenario]["sim"]["w_true"], label='RPM²')
        ax.set_xlabel("t (s)")
        ax.set_ylabel("rate ω (deg/s)")
        title = {"baseline": "Baseline (no FF)", "fixed": "Fixed FF", "learned": "Learned FF"}[scenario]
        ax.set_title(title)
        ax.legend(fontsize=8)
        ax.grid(True)
    
    # Row 2: Error comparison
    for col, scenario in enumerate(["baseline", "fixed", "learned"]):
        ax = axes[1, col]
        err_linear = linear_results[scenario]["sim"]["w_sp"] - linear_results[scenario]["sim"]["w_true"]
        err_rpm = rpm_sq_results[scenario]["sim"]["w_sp"] - rpm_sq_results[scenario]["sim"]["w_true"]
        ax.plot(t, err_linear, label='Linear')
        ax.plot(t, err_rpm, label='RPM²')
        ax.axhline(0, color='k', linestyle='--', alpha=0.3)
        ax.set_xlabel("t (s)")
        ax.set_ylabel("error (deg/s)")
        title = {"baseline": "Baseline (no FF)", "fixed": "Fixed FF", "learned": "Learned FF"}[scenario]
        ax.set_title(f"Error: {title}")
        ax.legend(fontsize=8)
        ax.grid(True)
    
    plt.tight_layout()

def plot_rpm_sq_summary_bar():
    """Bar chart comparing Linear vs RPM² models."""
    fig, axes = plt.subplots(1, 3, figsize=(14, 5))
    fig.suptitle("Linear vs RPM² Motor Model: Learning Effectiveness", fontsize=14)
    
    scenarios = ["Baseline\n(no FF)", "Fixed FF\n(wrong)", "Learned FF"]
    scenario_keys = ["baseline", "fixed", "learned"]
    
    x = np.arange(len(scenarios))
    width = 0.35
    
    # RMS Error comparison
    ax1 = axes[0]
    rms_linear = [linear_results[s]["metrics"]["intent_all"]["rms_err"] for s in scenario_keys]
    rms_rpm = [rpm_sq_results[s]["metrics"]["intent_all"]["rms_err"] for s in scenario_keys]
    ax1.bar(x - width/2, rms_linear, width, label='Linear')
    ax1.bar(x + width/2, rms_rpm, width, label='RPM²')
    ax1.set_ylabel('RMS Error (deg/s)')
    ax1.set_title('Tracking Error')
    ax1.set_xticks(x)
    ax1.set_xticklabels(scenarios)
    ax1.legend()
    ax1.grid(True, axis='y')
    
    # PID Effort comparison
    ax2 = axes[1]
    pid_linear = [linear_results[s]["metrics"]["intent_all"]["rms_pid"] for s in scenario_keys]
    pid_rpm = [rpm_sq_results[s]["metrics"]["intent_all"]["rms_pid"] for s in scenario_keys]
    ax2.bar(x - width/2, pid_linear, width, label='Linear')
    ax2.bar(x + width/2, pid_rpm, width, label='RPM²')
    ax2.set_ylabel('RMS PID Effort')
    ax2.set_title('Controller Effort')
    ax2.set_xticks(x)
    ax2.set_xticklabels(scenarios)
    ax2.legend()
    ax2.grid(True, axis='y')
    
    # Improvement from baseline to learned
    ax3 = axes[2]
    improv_linear = (rms_linear[0] - rms_linear[2]) / rms_linear[0] * 100
    improv_rpm = (rms_rpm[0] - rms_rpm[2]) / rms_rpm[0] * 100
    bars = ax3.bar(['Linear', 'RPM²'], [improv_linear, improv_rpm], color=['tab:blue', 'tab:orange'])
    ax3.set_ylabel('% Improvement')
    ax3.set_title('Learning Benefit\n(baseline -> learned)')
    ax3.grid(True, axis='y')
    for bar, val in zip(bars, [improv_linear, improv_rpm]):
        ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1, f'{val:.1f}%', 
                ha='center', va='bottom', fontsize=12)
    
    plt.tight_layout()

def plot_rpm_profile():
    """Show RPM profile during maneuver for RPM² model."""
    sim = rpm_sq_results["learned"]["sim"]
    t = sim["t"]
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("RPM² Model: Motor Behavior During Maneuver", fontsize=14)
    
    # Motor commands
    ax1 = axes[0, 0]
    ax1.plot(t, sim["motors"][:, 0], label='M1 (up)')
    ax1.plot(t, sim["motors"][:, 2], label='M3 (down)')
    ax1.set_xlabel("t (s)")
    ax1.set_ylabel("Motor command (%)")
    ax1.set_title("Motor Commands")
    ax1.legend()
    ax1.grid(True)
    
    # Average RPM
    ax2 = axes[0, 1]
    ax2.plot(t, sim["rpm_avg"])
    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("RPM")
    ax2.set_title("Average Motor RPM")
    ax2.grid(True)
    
    # RPM² (proportional to thrust)
    ax3 = axes[1, 0]
    ax3.plot(t, sim["rpm_sq_avg"] / 1e6)
    ax3.set_xlabel("t (s)")
    ax3.set_ylabel("RPM² / 1e6")
    ax3.set_title("Average RPM² (∝ thrust)")
    ax3.grid(True)
    
    # Rate tracking
    ax4 = axes[1, 1]
    ax4.plot(t, sim["w_sp"], 'k--', label='setpoint')
    ax4.plot(t, sim["w_true"], label='actual')
    ax4.set_xlabel("t (s)")
    ax4.set_ylabel("rate (deg/s)")
    ax4.set_title("Rate Tracking (Learned FF)")
    ax4.legend()
    ax4.grid(True)
    
    plt.tight_layout()

def plot_pid_quality_comparison():
    """Plot comparing learning effectiveness: Good PIDs vs Bad PIDs (RPM² + D_on_error only)."""
    
    # First plot: Bar chart comparing the 2 cases
    fig1, axes1 = plt.subplots(1, 2, figsize=(12, 5))
    fig1.suptitle("PID Quality Stress Test: Good vs Bad PIDs (RPM² + D_on_error)", fontsize=14)
    
    # Organize results
    labels = []
    baseline_rms = []
    learned_rms = []
    improvements = []
    
    for r in all_results:
        labels.append(r['pid'])
        baseline_rms.append(r["baseline_rms"])
        learned_rms.append(r["learned_rms"])
        improvements.append(r["improvement"])
    
    x = np.arange(len(labels))
    width = 0.35
    
    # Plot 1: Baseline vs Learned RMS Error
    ax1 = axes1[0]
    bars1 = ax1.bar(x - width/2, baseline_rms, width, label='Baseline (no FF)', color='gray', alpha=0.8)
    bars2 = ax1.bar(x + width/2, learned_rms, width, label='Learned FF', color='steelblue', alpha=0.8)
    ax1.set_ylabel('RMS Error (deg/s)')
    ax1.set_title('RMS Tracking Error')
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, fontsize=10)
    ax1.legend()
    ax1.grid(True, axis='y')
    
    # Add value labels on bars
    for bar in bars1:
        ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5, 
                f'{bar.get_height():.1f}', ha='center', va='bottom', fontsize=9)
    for bar in bars2:
        ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5, 
                f'{bar.get_height():.1f}', ha='center', va='bottom', fontsize=9)
    
    # Plot 2: Improvement percentage
    ax2 = axes1[1]
    colors = ['green' if v > 0 else 'red' for v in improvements]
    bars = ax2.bar(x, improvements, color=colors, edgecolor='black', alpha=0.8)
    ax2.axhline(0, color='k', linestyle='-', linewidth=1)
    ax2.set_ylabel('% Improvement')
    ax2.set_title('Learning Improvement (green=better, red=worse)')
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels, fontsize=10)
    ax2.grid(True, axis='y')
    
    # Add value labels
    for bar, val in zip(bars, improvements):
        ypos = bar.get_height() + (2 if val >= 0 else -4)
        ax2.text(bar.get_x() + bar.get_width()/2, ypos, 
                f'{val:+.1f}%', ha='center', va='bottom' if val >= 0 else 'top', 
                fontsize=12, fontweight='bold')
    
    plt.tight_layout()
    
    # Second plot: Time-series comparison (2x2: Good/Bad × Baseline/Learned)
    fig2, axes2 = plt.subplots(2, 2, figsize=(14, 8))
    fig2.suptitle("Rate Tracking: Good vs Bad PIDs (Baseline vs Learned FF)", fontsize=14)
    
    plot_configs = [
        ("Good PIDs", 0),
        ("Bad PIDs", 1),
    ]
    
    for pid_name, row in plot_configs:
        if pid_name in pid_stress_results:
            result = pid_stress_results[pid_name]
            t = result["baseline"]["sim"]["t"]
            w_sp = result["baseline"]["sim"]["w_sp"]
            w_base = result["baseline"]["sim"]["w_true"]
            w_learn = result["learned"]["sim"]["w_true"]
            
            base_rms = result["baseline"]["metrics"]["intent_all"]["rms_err"]
            learn_rms = result["learned"]["metrics"]["intent_all"]["rms_err"]
            improve = (base_rms - learn_rms) / base_rms * 100
            
            # Baseline plot
            ax_base = axes2[row, 0]
            ax_base.plot(t, w_sp, 'k--', alpha=0.7, linewidth=1.5, label='setpoint')
            ax_base.plot(t, w_base, 'r-', alpha=0.8, linewidth=1.2, label=f'actual (RMS={base_rms:.1f})')
            ax_base.set_title(f"{pid_name} - Baseline (no FF)", fontsize=11)
            ax_base.set_ylabel("rate (deg/s)")
            ax_base.legend(fontsize=9, loc='upper right')
            ax_base.grid(True, alpha=0.3)
            
            # Learned plot
            ax_learn = axes2[row, 1]
            ax_learn.plot(t, w_sp, 'k--', alpha=0.7, linewidth=1.5, label='setpoint')
            ax_learn.plot(t, w_learn, 'g-', alpha=0.8, linewidth=1.2, label=f'actual (RMS={learn_rms:.1f})')
            title_color = 'darkgreen' if improve > 0 else 'red'
            ax_learn.set_title(f"{pid_name} - Learned FF ({improve:+.1f}%)", fontsize=11, color=title_color, fontweight='bold')
            ax_learn.set_ylabel("rate (deg/s)")
            ax_learn.legend(fontsize=9, loc='upper right')
            ax_learn.grid(True, alpha=0.3)
    
    axes2[1, 0].set_xlabel("time (s)")
    axes2[1, 1].set_xlabel("time (s)")
    
    plt.tight_layout()
    
    # Third plot: Error comparison
    fig3, axes3 = plt.subplots(1, 2, figsize=(14, 5))
    fig3.suptitle("Tracking Error: Baseline vs Learned FF", fontsize=14)
    
    for idx, (pid_name, _) in enumerate(plot_configs):
        ax = axes3[idx]
        if pid_name in pid_stress_results:
            result = pid_stress_results[pid_name]
            t = result["baseline"]["sim"]["t"]
            w_sp = result["baseline"]["sim"]["w_sp"]
            err_base = w_sp - result["baseline"]["sim"]["w_true"]
            err_learn = w_sp - result["learned"]["sim"]["w_true"]
            
            base_rms = result["baseline"]["metrics"]["intent_all"]["rms_err"]
            learn_rms = result["learned"]["metrics"]["intent_all"]["rms_err"]
            improve = (base_rms - learn_rms) / base_rms * 100
            
            ax.plot(t, err_base, 'r-', alpha=0.8, linewidth=1.2, label=f'Baseline err (RMS={base_rms:.1f})')
            ax.plot(t, err_learn, 'g-', alpha=0.8, linewidth=1.2, label=f'Learned err (RMS={learn_rms:.1f})')
            ax.axhline(0, color='k', linestyle='--', alpha=0.5)
            
            title_color = 'darkgreen' if improve > 0 else 'red'
            ax.set_title(f"{pid_name} ({improve:+.1f}%)", fontsize=11, color=title_color, fontweight='bold')
        
        ax.set_xlabel("time (s)")
        ax.set_ylabel("error (deg/s)")
        ax.legend(fontsize=9, loc='upper right')
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()

def plot_8combo_pidf():
    """Plot P, I, D, F time-series for Good PIDs vs Bad PIDs (RPM² + D_on_error only).
    All PIDF terms on same axis, color-coded."""
    
    cases = ["Good PIDs", "Bad PIDs"]
    
    # Figure 1: Rate tracking comparison
    fig1, axes1 = plt.subplots(2, 2, figsize=(14, 10))
    fig1.suptitle("Rate Tracking: Good PIDs vs Bad PIDs (RPM² + D_on_error)", fontsize=14)
    
    for row, pid_name in enumerate(cases):
        if pid_name not in pid_stress_results:
            continue
        result = pid_stress_results[pid_name]
        
        # Calculate improvement
        base_rms = result["baseline"]["metrics"]["intent_all"]["rms_err"]
        learn_rms = result["learned"]["metrics"]["intent_all"]["rms_err"]
        improve = (base_rms - learn_rms) / base_rms * 100
        
        # Baseline
        sim_base = result["baseline"]["sim"]
        t = sim_base["t"]
        w_sp = sim_base["w_sp"]
        
        ax = axes1[row, 0]
        ax.plot(t, w_sp, 'k--', linewidth=2, label='Setpoint', alpha=0.7)
        ax.plot(t, sim_base["w_true"], 'r-', linewidth=1.5, label=f'Actual (RMS={base_rms:.1f})')
        ax.set_title(f"{pid_name}: BASELINE (No FF)", fontsize=12, fontweight='bold')
        ax.set_xlabel("t (s)")
        ax.set_ylabel("Rate (deg/s)")
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        
        # Learned FF
        sim_learn = result["learned"]["sim"]
        ax = axes1[row, 1]
        ax.plot(t, w_sp, 'k--', linewidth=2, label='Setpoint', alpha=0.7)
        ax.plot(t, sim_learn["w_true"], 'g-', linewidth=1.5, label=f'Actual (RMS={learn_rms:.1f})')
        title_color = 'darkgreen' if improve > 0 else 'red'
        ax.set_title(f"{pid_name}: LEARNED FF ({improve:+.1f}%)", fontsize=12, 
                    fontweight='bold', color=title_color)
        ax.set_xlabel("t (s)")
        ax.set_ylabel("Rate (deg/s)")
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Figure 2: PIDF breakdown - all on same axis
    fig2, axes2 = plt.subplots(2, 2, figsize=(16, 10))
    fig2.suptitle("PIDF Controller Terms: All on One Axis (color-coded)", fontsize=14)
    
    colors = {'P': 'blue', 'I': 'green', 'D': 'red', 'F': 'purple'}
    
    for row, pid_name in enumerate(cases):
        if pid_name not in pid_stress_results:
            continue
        result = pid_stress_results[pid_name]
        
        base_rms = result["baseline"]["metrics"]["intent_all"]["rms_err"]
        learn_rms = result["learned"]["metrics"]["intent_all"]["rms_err"]
        improve = (base_rms - learn_rms) / base_rms * 100
        
        # Baseline PIDF
        sim_base = result["baseline"]["sim"]
        t = sim_base["t"]
        
        ax = axes2[row, 0]
        ax.plot(t, sim_base["P"], color=colors['P'], linewidth=1.5, label='P', alpha=0.9)
        ax.plot(t, sim_base["I"], color=colors['I'], linewidth=1.5, label='I', alpha=0.9)
        ax.plot(t, sim_base["D"], color=colors['D'], linewidth=1.5, label='D', alpha=0.9)
        ax.plot(t, sim_base["u_ff"], color=colors['F'], linewidth=1.5, label='F', alpha=0.9)
        ax.axhline(0, color='k', linestyle='--', alpha=0.3)
        ax.set_title(f"{pid_name}: BASELINE PIDF (No FF)", fontsize=12, fontweight='bold')
        ax.set_xlabel("t (s)")
        ax.set_ylabel("Controller output")
        ax.legend(loc='upper right', ncol=4)
        ax.grid(True, alpha=0.3)
        
        # Learned PIDF
        sim_learn = result["learned"]["sim"]
        
        ax = axes2[row, 1]
        ax.plot(t, sim_learn["P"], color=colors['P'], linewidth=1.5, label='P', alpha=0.9)
        ax.plot(t, sim_learn["I"], color=colors['I'], linewidth=1.5, label='I', alpha=0.9)
        ax.plot(t, sim_learn["D"], color=colors['D'], linewidth=1.5, label='D', alpha=0.9)
        ax.plot(t, sim_learn["u_ff"], color=colors['F'], linewidth=2, label='F (FF)', alpha=0.9)
        ax.axhline(0, color='k', linestyle='--', alpha=0.3)
        title_color = 'darkgreen' if improve > 0 else 'red'
        ax.set_title(f"{pid_name}: LEARNED PIDF ({improve:+.1f}%)", fontsize=12, 
                    fontweight='bold', color=title_color)
        ax.set_xlabel("t (s)")
        ax.set_ylabel("Controller output")
        ax.legend(loc='upper right', ncol=4)
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Figure 3: Overlay comparison - Baseline vs Learned for each PID case
    fig3, axes3 = plt.subplots(2, 4, figsize=(20, 8))
    fig3.suptitle("PIDF Comparison: Baseline (dashed) vs Learned (solid)", fontsize=14)
    
    term_names = ['P', 'I', 'D', 'F']
    term_keys = ['P', 'I', 'D', 'u_ff']
    
    for row, pid_name in enumerate(cases):
        if pid_name not in pid_stress_results:
            continue
        result = pid_stress_results[pid_name]
        
        sim_base = result["baseline"]["sim"]
        sim_learn = result["learned"]["sim"]
        t = sim_base["t"]
        
        base_rms = result["baseline"]["metrics"]["intent_all"]["rms_err"]
        learn_rms = result["learned"]["metrics"]["intent_all"]["rms_err"]
        improve = (base_rms - learn_rms) / base_rms * 100
        
        for col, (name, key) in enumerate(zip(term_names, term_keys)):
            ax = axes3[row, col]
            color = colors[name]
            
            ax.plot(t, sim_base[key], color=color, linewidth=1.5, alpha=0.5, 
                   linestyle='--', label='Baseline')
            ax.plot(t, sim_learn[key], color=color, linewidth=1.5, alpha=0.9, 
                   linestyle='-', label='Learned')
            ax.axhline(0, color='k', linestyle='--', alpha=0.3)
            
            title = f"{pid_name}: {name}-term"
            if col == 0:
                title += f" ({improve:+.1f}%)"
            ax.set_title(title, fontsize=10, fontweight='bold')
            ax.set_xlabel("t (s)")
            if col == 0:
                ax.set_ylabel("Value")
            ax.legend(fontsize=8, loc='upper right')
            ax.grid(True, alpha=0.3)
    
    plt.tight_layout()

def plot_rpm_binned_learning():
    """Plot the RPM-binned learning results and comparison."""
    fig, axes = plt.subplots(2, 3, figsize=(15, 9))
    fig.suptitle("RPM-Binned Learning: The Key to RPM² Thrust Compensation", fontsize=14)
    
    # --- Row 1: Bin visualization and fit ---
    
    # Plot 1: Bin counts histogram
    ax1 = axes[0, 0]
    bin_info = rpm_binned_results["bin_info"]
    counts = bin_info["counts"]
    x_min = bin_info["x_min"]
    bin_width = bin_info["bin_width"]
    bin_edges = [x_min + i * bin_width for i in range(len(counts) + 1)]
    ax1.bar(range(len(counts)), counts, color='steelblue', edgecolor='black')
    ax1.axhline(5, color='r', linestyle='--', label='min samples (5)')
    ax1.set_xlabel("Bin index")
    ax1.set_ylabel("Sample count")
    ax1.set_title("Samples per RPM² bin")
    ax1.legend()
    ax1.grid(True, axis='y')
    
    # Plot 2: Fitted E(x) line through bin averages
    ax2 = axes[0, 1]
    centers = bin_info["centers"]
    averages = bin_info["averages"]
    E0 = rpm_binned_results["E0"]
    E1 = rpm_binned_results["E1"]
    
    if len(centers) > 0:
        ax2.scatter(centers, averages, s=80, c='blue', label='Bin averages', zorder=3)
        x_fit = np.linspace(min(centers) - 0.1, max(centers) + 0.1, 50)
        y_fit = E0 + E1 * x_fit
        ax2.plot(x_fit, y_fit, 'r-', linewidth=2, label=f'E(x) = {E0:.2f} + {E1:.3f}·x')
    ax2.set_xlabel("x = RPM² / 1e6")
    ax2.set_ylabel("Effectiveness g = α/u")
    ax2.set_title("Fitted effectiveness model")
    ax2.legend()
    ax2.grid(True)
    
    # Plot 3: E(x) vs throttle position interpretation
    ax3 = axes[0, 2]
    # Show how E varies across throttle range
    throttle_pct = np.linspace(20, 80, 50)
    rpm = (throttle_pct / 100) * 25000  # Assuming rpm_max = 25000
    x_throttle = (rpm ** 2) / 1e6
    E_throttle = E0 + E1 * x_throttle
    ax3.plot(throttle_pct, E_throttle, 'b-', linewidth=2)
    ax3.set_xlabel("Throttle (%)")
    ax3.set_ylabel("Effectiveness E(x)")
    ax3.set_title("Effectiveness vs throttle position")
    ax3.grid(True)
    # Add hover marker
    hover_x = (0.5 * 25000) ** 2 / 1e6
    hover_E = E0 + E1 * hover_x
    ax3.axvline(50, color='g', linestyle='--', alpha=0.7, label='Hover (50%)')
    ax3.scatter([50], [hover_E], s=100, c='green', zorder=3)
    ax3.legend()
    
    # --- Row 2: Performance comparison ---
    
    t = rpm_binned_results["baseline"]["sim"]["t"]
    w_sp = rpm_binned_results["baseline"]["sim"]["w_sp"]
    
    # Plot 4: Rate tracking comparison
    ax4 = axes[1, 0]
    ax4.plot(t, w_sp, 'k--', alpha=0.6, label='setpoint')
    ax4.plot(t, rpm_binned_results["baseline"]["sim"]["w_true"], 
             label='Baseline (no FF)', alpha=0.8)
    ax4.plot(t, rpm_sq_results["learned"]["sim"]["w_true"], 
             label='Constant E (wrong)', alpha=0.8)
    ax4.plot(t, rpm_binned_results["learned"]["sim"]["w_true"], 
             label='Binned E (correct)', alpha=0.8)
    ax4.set_xlabel("t (s)")
    ax4.set_ylabel("rate (deg/s)")
    ax4.set_title("Rate tracking comparison")
    ax4.legend(fontsize=8)
    ax4.grid(True)
    
    # Plot 5: Error comparison
    ax5 = axes[1, 1]
    err_base = w_sp - rpm_binned_results["baseline"]["sim"]["w_true"]
    err_const = w_sp - rpm_sq_results["learned"]["sim"]["w_true"]
    err_binned = w_sp - rpm_binned_results["learned"]["sim"]["w_true"]
    ax5.plot(t, err_base, label='Baseline', alpha=0.8)
    ax5.plot(t, err_const, label='Constant E', alpha=0.8)
    ax5.plot(t, err_binned, label='Binned E', alpha=0.8)
    ax5.axhline(0, color='k', linestyle='--', alpha=0.3)
    ax5.set_xlabel("t (s)")
    ax5.set_ylabel("error (deg/s)")
    ax5.set_title("Tracking error comparison")
    ax5.legend(fontsize=8)
    ax5.grid(True)
    
    # Plot 6: Summary bar chart
    ax6 = axes[1, 2]
    methods = ['Baseline\n(no FF)', 'Constant E\n(wrong)', 'Binned E\n(correct)']
    rms_values = [
        rpm_binned_results["baseline"]["metrics"]["intent_all"]["rms_err"],
        rpm_sq_results["learned"]["metrics"]["intent_all"]["rms_err"],
        rpm_binned_results["learned"]["metrics"]["intent_all"]["rms_err"],
    ]
    colors = ['gray', 'red', 'green']
    bars = ax6.bar(methods, rms_values, color=colors, edgecolor='black')
    ax6.set_ylabel("RMS Error (deg/s)")
    ax6.set_title("Learning method comparison\n(RPM² motor model)")
    ax6.grid(True, axis='y')
    for bar, val in zip(bars, rms_values):
        ax6.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5, 
                f'{val:.1f}', ha='center', va='bottom', fontsize=11)
    
    plt.tight_layout()

# Disabled old plots - only show 8-combination analysis
# plot_rates()
# plot_error()
# plot_pid_ff()
# plot_saturation_and_gates()
# plot_alpha_fit_quality()
# plot_convergence()
plot_pid_quality_comparison()
plot_8combo_pidf()  # New PIDF breakdown plot
# plot_d_term_comparison()
# plot_d_term_summary_bar()
# plot_rpm_sq_comparison()
# plot_rpm_sq_summary_bar()
# plot_rpm_profile()
# plot_rpm_binned_learning()
plt.show()
