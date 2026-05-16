function [rec, bms] = simulate_fw(p, t, xd, yd, zd, psid)
%SIMULATE_FW  Fixed-wing UAV + BMS coupled simulation.
%  Cascaded GNC architecture (no rudder, Bank-to-Turn):
%
%  Longitudinal:
%    Outer:  alt error   → theta_cmd   (pid_alt)
%            speed error → delta_t     (pid_speed, PI)
%    Inner1: theta err   → q_ref       (pid_pitch_att, P/PI)
%    Inner2: q err       → delta_e     (pid_pitch_rate, fast PID)
%
%  Lateral:
%    Outer:  heading err → phi_cmd     (pid_hdg)
%    Inner1: phi err     → p_ref       (pid_roll_att, P/PI)
%    Inner2: p err       → delta_a     (pid_roll_rate, fast PID)

N  = length(t);
dt = p.dt;
g  = p.g;
u0 = p.u0;

%% Rate-limit parameters
rl_theta = get_or_default(p, 'rate_theta_cmd', deg2rad(20));
rl_phi   = get_or_default(p, 'rate_phi_cmd',   deg2rad(30));
rl_de    = get_or_default(p, 'rate_de', 5.0);
rl_da    = get_or_default(p, 'rate_da', 5.0);
rl_dt    = get_or_default(p, 'rate_dt', 3.0);

%% Plant (discretized state-space)
plant = create_plant_fw(p);

%% State vectors
% Longitudinal: [du; dw; dq; dtheta]
x_lon = zeros(4,1);
% Lateral:      [dv; dp; dr; dphi]
x_lat = zeros(4,1);

% Inertial position
x_pos = xd(1);  y_pos = yd(1);  z_pos = zd(1);
psi   = psid(1);

%% PID states [integrator; filter] — outer loops
p_alt   = [0;0];  p_spd   = [0;0];  p_hdg   = [0;0];
% Inner loop 1: attitude → rate reference
p_pitch_att  = [0;0];  p_roll_att  = [0;0];
% Inner loop 2: rate → actuator
p_pitch_rate = [0;0];  p_roll_rate = [0;0];

%% Previous commands for rate limiting
theta_cmd_prev = 0;
phi_cmd_prev   = 0;
d_e_prev = 0;  d_a_prev = 0;  d_t_prev = 0.5;

%% Saturation counters
sat_de = 0;  sat_da = 0;  sat_dt = 0;
sat_thresh = 0.95;

%% Battery state
soc   = p.bat.SOC0;
I_act = 0;

%% Pre-allocate recording
fn = {'x','y','z','phi','theta','psi','x_d','y_d','z_d',...
      'u_T','u_phi','u_theta','u_psi',...
      'airspeed','alpha','beta','d_t','d_e','d_a','d_r'};
rec = struct();
for i=1:numel(fn); rec.(fn{i})=zeros(1,N); end
bms = struct('soc',zeros(1,N),'voltage',zeros(1,N),...
    'current',zeros(1,N),'power',zeros(1,N),...
    'I_req',zeros(1,N),'k_avail',ones(1,N));

%% Simulation loop
fprintf('Simulating fixed-wing (%d steps)...\n', N);
for k = 1:N
    % Current perturbation states
    du     = x_lon(1);
    dw     = x_lon(2);
    dq     = x_lon(3);   % pitch rate
    dtheta = x_lon(4);

    dv     = x_lat(1);
    dp     = x_lat(2);   % roll rate
    dr     = x_lat(3);   % yaw rate
    dphi   = x_lat(4);

    % Total states
    u_total = u0 + du;
    theta   = p.theta0 + dtheta;
    phi     = dphi;
    q       = dq;         % pitch rate (trim q0=0)
    p_body  = dp;         % roll rate

    % Airspeed, AoA, sideslip
    V_air = sqrt(u_total^2 + dv^2 + dw^2);
    alpha = atan2(dw, u_total);
    beta  = asin(max(-1, min(1, dv / max(V_air, 0.1))));

    % Record
    rec.x(k) = x_pos;   rec.y(k) = y_pos;   rec.z(k) = z_pos;
    rec.phi(k)   = phi;  rec.theta(k) = theta; rec.psi(k) = psi;
    rec.x_d(k) = xd(k); rec.y_d(k) = yd(k);  rec.z_d(k) = zd(k);
    rec.airspeed(k) = V_air;
    rec.alpha(k)    = alpha;
    rec.beta(k)     = beta;

    %% ========= OUTER CONTROL LOOPS =========

    % --- Altitude → pitch command (PID) ---
    alt_err = zd(k) - z_pos;
    [theta_cmd, p_alt(1), p_alt(2)] = pid_ct(alt_err, p_alt(1), p_alt(2), p.pid_alt, dt);
    theta_cmd = max(-p.theta_max, min(p.theta_max, theta_cmd));
    theta_cmd = rate_limit(theta_cmd, theta_cmd_prev, rl_theta, dt);
    theta_cmd_prev = theta_cmd;

    % --- Speed → throttle (PI) ---
    spd_err = u0 - u_total;
    [d_t, p_spd(1), p_spd(2)] = pid_ct(spd_err, p_spd(1), p_spd(2), p.pid_speed, dt);
    d_t = max(0, min(1, 0.5 + d_t));
    d_t = rate_limit(d_t, d_t_prev, rl_dt, dt);
    d_t_prev = d_t;

    % --- Heading → bank command ---
    psi_des = psid(k);
    hdg_err = atan2(sin(psi_des - psi), cos(psi_des - psi));
    [phi_cmd, p_hdg(1), p_hdg(2)] = pid_ct(hdg_err, p_hdg(1), p_hdg(2), p.pid_hdg, dt);
    phi_cmd = max(-p.phi_max, min(p.phi_max, phi_cmd));
    phi_cmd = rate_limit(phi_cmd, phi_cmd_prev, rl_phi, dt);
    phi_cmd_prev = phi_cmd;

    %% ========= INNER LOOP 1: ATTITUDE → RATE REFERENCE =========

    % --- Pitch attitude → q_ref (P/PI) ---
    pitch_att_err = theta_cmd - theta;
    [q_ref, p_pitch_att(1), p_pitch_att(2)] = pid_ct(pitch_att_err, p_pitch_att(1), p_pitch_att(2), p.pid_pitch_att, dt);

    % --- Roll attitude → p_ref (P/PI) ---
    roll_att_err = phi_cmd - phi;
    [p_ref, p_roll_att(1), p_roll_att(2)] = pid_ct(roll_att_err, p_roll_att(1), p_roll_att(2), p.pid_roll_att, dt);

    %% ========= INNER LOOP 2: RATE → ACTUATOR =========

    % --- Pitch rate → elevator (fast PID) ---
    q_err = q_ref - q;
    [d_e, p_pitch_rate(1), p_pitch_rate(2)] = pid_ct(q_err, p_pitch_rate(1), p_pitch_rate(2), p.pid_pitch_rate, dt);
    d_e = -d_e;    % +q needs -delta_e (aero: +delta_e → nose down)
    d_e = rate_limit(d_e, d_e_prev, rl_de, dt);
    d_e_prev = d_e;

    % --- Roll rate → aileron (fast PID) ---
    p_err = p_ref - p_body;
    [d_a, p_roll_rate(1), p_roll_rate(2)] = pid_ct(p_err, p_roll_rate(1), p_roll_rate(2), p.pid_roll_rate, dt);
    d_a = rate_limit(d_a, d_a_prev, rl_da, dt);
    d_a_prev = d_a;

    %% Saturation tracking
    if abs(d_e) > sat_thresh * abs(p.pid_pitch_rate.hi); sat_de = sat_de + 1; end
    if abs(d_a) > sat_thresh * abs(p.pid_roll_rate.hi);  sat_da = sat_da + 1; end
    if d_t > 0.95 || d_t < 0.05;                        sat_dt = sat_dt + 1; end

    % Record control inputs (d_r = 0, no rudder)
    rec.d_t(k) = d_t;  rec.d_e(k) = d_e;
    rec.d_a(k) = d_a;  rec.d_r(k) = 0;
    rec.u_T(k) = d_t;      rec.u_phi(k) = d_a;
    rec.u_theta(k) = d_e;  rec.u_psi(k) = 0;

    %% ========= BMS STEP =========
    [I_act, soc, V, P_act, k_avail, I_req] = bms_step_fw(...
        d_t, d_e, d_a, I_act, soc, p.bat, dt);

    bms.soc(k) = soc;       bms.voltage(k) = V;
    bms.current(k) = I_act; bms.power(k) = P_act;
    bms.I_req(k) = I_req;   bms.k_avail(k) = k_avail;

    %% ========= PLANT UPDATE =========
    if k < N
        % Scale control inputs by battery availability
        d_t_eff = d_t * k_avail;
        d_e_eff = d_e * k_avail;
        d_a_eff = d_a * k_avail;

        % Longitudinal dynamics: u_lon = [d_t; d_e]
        u_lon = [d_t_eff; d_e_eff];
        x_lon = plant.Ad_lon * x_lon + plant.Bd_lon * u_lon;
        x_lon(4) = max(-p.theta_max, min(p.theta_max, x_lon(4)));

        % Lateral dynamics: u_lat = [d_a] (single input, no rudder)
        u_lat = d_a_eff;
        x_lat = plant.Ad_lat * x_lat + plant.Bd_lat * u_lat;
        x_lat(4) = max(-p.phi_max, min(p.phi_max, x_lat(4)));

        % Update Euler angles
        theta_new = p.theta0 + x_lon(4);
        phi_new   = x_lat(4);

        % Yaw rate from lateral state
        q_body = x_lon(3);
        r_body = x_lat(3);
        psi_dot = r_body*cos(phi_new)/cos(theta_new) + q_body*sin(phi_new)/cos(theta_new);
        psi = psi + psi_dot * dt;
        psi = atan2(sin(psi), cos(psi));

        % Position kinematics (body → inertial)
        u_body = u0 + x_lon(1);
        v_body = x_lat(1);
        w_body = x_lon(2);

        cp = cos(phi_new);  sp = sin(phi_new);
        ct = cos(theta_new);st = sin(theta_new);
        cs = cos(psi);      ss_ = sin(psi);

        x_dot = u_body*(ct*cs) + v_body*(sp*st*cs - cp*ss_) + w_body*(cp*st*cs + sp*ss_);
        y_dot = u_body*(ct*ss_) + v_body*(sp*st*ss_ + cp*cs) + w_body*(cp*st*ss_ - sp*cs);
        z_dot = -u_body*st + v_body*sp*ct + w_body*cp*ct;

        x_pos = x_pos + x_dot * dt;
        y_pos = y_pos + y_dot * dt;
        z_pos = z_pos - z_dot * dt;   % NED z_dot positive-down; altitude positive-up
        z_pos = max(0, z_pos);
    end
end

%% Saturation diagnostics
fprintf('Simulation complete.\n');
fprintf('--- Actuator saturation report (>%.0f%% of limit) ---\n', sat_thresh*100);
fprintf('  Elevator (d_e):  %5.1f%% of time\n', 100*sat_de/N);
fprintf('  Aileron  (d_a):  %5.1f%% of time\n', 100*sat_da/N);
fprintf('  Throttle (d_t):  %5.1f%% of time\n', 100*sat_dt/N);
end

%% ======================================================================
function val = get_or_default(s, field, default)
    if isfield(s, field); val = s.(field); else; val = default; end
end

function y = rate_limit(cmd, prev, max_rate, dt)
    delta = cmd - prev;
    max_delta = max_rate * dt;
    y = prev + max(-max_delta, min(max_delta, delta));
end
