function cost = simulate_fw_fast(p, t, xd, yd, zd, psid, plant)
%SIMULATE_FW_FAST  Stripped-down simulation for PSO/GWO fitness evaluation.
%  Cascaded architecture: no rudder, Bank-to-Turn, rate inner loops.
%  No BMS, no recording — just propagate states and accumulate cost.
%  NOW INCLUDES RATE-LIMITING to match simulate_fw behavior.

N  = length(t);
dt = p.dt;
u0 = p.u0;

%% Plant (use cached if provided)
if nargin < 7 || isempty(plant)
    plant = create_plant_fw(p);
end

%% Rate-limit parameters (must match simulate_fw)
rl_theta = get_or_default(p, 'rate_theta_cmd', deg2rad(20));
rl_phi   = get_or_default(p, 'rate_phi_cmd',   deg2rad(30));
rl_de    = get_or_default(p, 'rate_de', 5.0);
rl_da    = get_or_default(p, 'rate_da', 5.0);
rl_dt    = get_or_default(p, 'rate_dt', 3.0);

%% State vectors
x_lon = zeros(4,1);   % [du; dw; dq; dtheta]
x_lat = zeros(4,1);   % [dv; dp; dr; dphi]

x_pos = xd(1);  y_pos = yd(1);  z_pos = zd(1);
psi   = psid(1);

%% PID states [integrator; filter]
p_alt = [0;0]; p_spd = [0;0]; p_hdg = [0;0];
p_pitch_att = [0;0]; p_pitch_rate = [0;0];
p_roll_att  = [0;0]; p_roll_rate  = [0;0];

%% Previous commands for rate limiting
theta_cmd_prev = 0;
phi_cmd_prev   = 0;
d_e_prev = 0;  d_a_prev = 0;  d_t_prev = 0.5;

%% Cost accumulators
cost_pos   = 0;
cost_ctrl  = 0;
cost_osc   = 0;
cost_sat   = 0;
prev_theta = p.theta0;
prev_phi   = 0;

sat_thresh = 0.90;

%% Simulation loop
for k = 1:N
    du     = x_lon(1);   dw     = x_lon(2);
    dq     = x_lon(3);   dtheta = x_lon(4);
    dv     = x_lat(1);   dp     = x_lat(2);
    dphi   = x_lat(4);

    u_total = u0 + du;
    theta   = p.theta0 + dtheta;
    phi     = dphi;
    q       = dq;
    p_body  = dp;

    V_air = sqrt(u_total^2 + dv^2 + dw^2);

    %% Position tracking cost
    ex = xd(k) - x_pos;
    ey = yd(k) - y_pos;
    ez = zd(k) - z_pos;
    cost_pos = cost_pos + (ex^2 + ey^2 + 4*ez^2) * dt;

    %% Oscillation cost
    cost_osc = cost_osc + ((theta - prev_theta)^2 + (phi - prev_phi)^2) / dt;
    prev_theta = theta;
    prev_phi   = phi;

    %% OUTER CONTROL LOOPS
    alt_err = zd(k) - z_pos;
    [theta_cmd, p_alt(1), p_alt(2)] = pid_ct(alt_err, p_alt(1), p_alt(2), p.pid_alt, dt);
    theta_cmd = max(-p.theta_max, min(p.theta_max, theta_cmd));
    theta_cmd = rate_limit(theta_cmd, theta_cmd_prev, rl_theta, dt);
    theta_cmd_prev = theta_cmd;

    spd_err = u0 - u_total;
    [d_t, p_spd(1), p_spd(2)] = pid_ct(spd_err, p_spd(1), p_spd(2), p.pid_speed, dt);
    d_t = max(0, min(1, 0.5 + d_t));
    d_t = rate_limit(d_t, d_t_prev, rl_dt, dt);
    d_t_prev = d_t;

    psi_des = psid(k);
    hdg_err = atan2(sin(psi_des - psi), cos(psi_des - psi));
    [phi_cmd, p_hdg(1), p_hdg(2)] = pid_ct(hdg_err, p_hdg(1), p_hdg(2), p.pid_hdg, dt);
    phi_cmd = max(-p.phi_max, min(p.phi_max, phi_cmd));
    phi_cmd = rate_limit(phi_cmd, phi_cmd_prev, rl_phi, dt);
    phi_cmd_prev = phi_cmd;

    %% INNER LOOP 1: ATTITUDE → RATE REFERENCE
    pitch_att_err = theta_cmd - theta;
    [q_ref, p_pitch_att(1), p_pitch_att(2)] = pid_ct(pitch_att_err, p_pitch_att(1), p_pitch_att(2), p.pid_pitch_att, dt);

    roll_att_err = phi_cmd - phi;
    [p_ref, p_roll_att(1), p_roll_att(2)] = pid_ct(roll_att_err, p_roll_att(1), p_roll_att(2), p.pid_roll_att, dt);

    %% INNER LOOP 2: RATE → ACTUATOR
    q_err = q_ref - q;
    [d_e, p_pitch_rate(1), p_pitch_rate(2)] = pid_ct(q_err, p_pitch_rate(1), p_pitch_rate(2), p.pid_pitch_rate, dt);
    d_e = -d_e;
    d_e = rate_limit(d_e, d_e_prev, rl_de, dt);
    d_e_prev = d_e;

    p_err = p_ref - p_body;
    [d_a, p_roll_rate(1), p_roll_rate(2)] = pid_ct(p_err, p_roll_rate(1), p_roll_rate(2), p.pid_roll_rate, dt);
    d_a = rate_limit(d_a, d_a_prev, rl_da, dt);
    d_a_prev = d_a;

    %% Control effort cost
    cost_ctrl = cost_ctrl + (d_e^2 + d_a^2) * dt;

    %% Saturation penalty
    de_frac = abs(d_e) / max(abs(p.pid_pitch_rate.hi), 0.01);
    da_frac = abs(d_a) / max(abs(p.pid_roll_rate.hi),  0.01);
    dt_frac = abs(d_t - 0.5) / 0.5;
    if de_frac > sat_thresh; cost_sat = cost_sat + (de_frac - sat_thresh)^2 * dt; end
    if da_frac > sat_thresh; cost_sat = cost_sat + (da_frac - sat_thresh)^2 * dt; end
    if dt_frac > 0.9;       cost_sat = cost_sat + (dt_frac - 0.9)^2 * dt; end

    %% PLANT UPDATE
    if k < N
        u_lon = [d_t; d_e];
        x_lon = plant.Ad_lon * x_lon + plant.Bd_lon * u_lon;
        x_lon(4) = max(-p.theta_max, min(p.theta_max, x_lon(4)));

        u_lat = d_a;   % single input (aileron only)
        x_lat = plant.Ad_lat * x_lat + plant.Bd_lat * u_lat;
        x_lat(4) = max(-p.phi_max, min(p.phi_max, x_lat(4)));

        theta_new = p.theta0 + x_lon(4);
        phi_new   = x_lat(4);

        q_body_k = x_lon(3);  r_body = x_lat(3);
        psi_dot = r_body*cos(phi_new)/cos(theta_new) + q_body_k*sin(phi_new)/cos(theta_new);
        psi = psi + psi_dot * dt;
        psi = atan2(sin(psi), cos(psi));

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
        z_pos = z_pos - z_dot * dt;
        z_pos = max(0, z_pos);
    end

    %% Early termination if diverged
    if abs(ez) > 100 || abs(ex) > 300 || abs(ey) > 300
        cost = 1e6;
        return;
    end
end

%% Weighted total cost
w_pos  = 1.0;
w_ctrl = 0.50;   % increased from 0.20 to strongly penalize throttle bang-bang and chatter
w_osc  = 0.5;
w_sat  = 0.5;

cost = w_pos * cost_pos + w_ctrl * cost_ctrl + w_osc * cost_osc + w_sat * cost_sat;
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
