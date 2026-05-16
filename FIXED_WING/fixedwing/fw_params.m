%% fw_params.m — Fixed-wing UAV physical, aero, controller, and trajectory parameters
%  Adds fixed-wing-related fields to struct 'p'.
%  Usage: called from main_fw.m

%% Physical
p.m  = 0.8;       % mass [kg]
p.g  = 9.81;      % gravity [m/s^2]
p.Jx = 0.0824;    % roll moment of inertia  [kg·m^2]
p.Jy = 0.1135;    % pitch moment of inertia [kg·m^2]
p.Jz = 0.1759;    % yaw moment of inertia   [kg·m^2]

%% Aerodynamic geometry
p.S    = 0.55;     % wing reference area [m^2]
p.b    = 2.8;      % wingspan [m]
p.cbar = 0.20;     % mean aerodynamic chord [m]
p.rho  = 1.225;    % air density [kg/m^3]

%% Trim condition (steady level flight)
p.u0     = 15.0;   % trim airspeed [m/s]
p.theta0 = 0.0;    % trim pitch angle [rad] (level flight)

%% Longitudinal stability derivatives (dimensional, around trim)
%  dx_lon = A_lon * x_lon + B_lon * u_lon
%  x_lon = [du; dw; dq; dtheta]
%  u_lon = [d_delta_t; d_delta_e]
p.Xu = -0.45;      p.Xw =  0.36;      p.Xq = 0;
p.Zu = -2.54;      p.Zw = -5.33;      p.Zq = 0;
p.Mu =  0.0;       p.Mw = -2.15;      p.Mq = -3.40;

%% Longitudinal control derivatives
p.X_dt = 8.0;      p.X_de = 0.0;    % throttle → X, elevator → X
p.Z_dt = 0.0;      p.Z_de = -5.5;   % throttle → Z, elevator → Z
p.M_dt = 0.0;      p.M_de = -15.0;  % throttle → M, elevator → M

%% Lateral-directional stability derivatives (dimensional, around trim)
%  dx_lat = A_lat * x_lat + B_lat * u_lat
%  x_lat = [dv; dp; dr; dphi]
%  u_lat = [d_delta_a; d_delta_r]
p.Yv = -0.56;      p.Yp = 0;         p.Yr = 0;
p.Lv = -0.075;     p.Lp = -4.50;     p.Lr =  0.51;
p.Nv =  0.042;     p.Np = -0.069;    p.Nr = -0.52;

%% Lateral-directional control derivatives (aileron only — no rudder per architecture doc)
p.Y_da = 0.0;
p.L_da = 12.0;
p.N_da = -0.07;

%% Simulation
p.dt      = 0.005;    % timestep [s] (fixed-wing can use coarser dt)
p.T_final = 120;      % default simulation duration [s]

%% Angle limits [rad]
p.phi_max   = deg2rad(35);    % max bank angle
p.theta_max = deg2rad(25);    % max pitch perturbation
p.psi_max   = deg2rad(30);    % yaw can go full circle

%% Rate limits for command smoothing (rad/s or unit/s)
p.rate_theta_cmd = deg2rad(15);   % pitch command rate limit [rad/s]
p.rate_phi_cmd   = deg2rad(25);   % bank command rate limit [rad/s]
p.rate_de        = 4.0;           % elevator actuator rate [unit/s]
p.rate_da        = 4.0;           % aileron actuator rate [unit/s]
p.rate_dt        = 2.0;           % throttle actuator rate [unit/s]

%% PID Gains — struct: Kp, Ki, Kd, Tf, lo, hi
%  Cascaded architecture per flight architecture document:
%    Longitudinal: Altitude PID → theta_ref → Pitch Att P/PI → q_ref → Pitch Rate PID → delta_e
%    Lateral:      Heading PID → phi_ref → Roll Att P/PI → p_ref → Roll Rate PID → delta_a
%    Speed:        Airspeed PI → delta_t
%    No rudder (Bank-to-Turn)

% Outer loops
p.pid_alt   = struct('Kp',0.55, 'Ki',0.10, 'Kd',0.40, 'Tf',0.25, 'lo',-deg2rad(12), 'hi',deg2rad(12));  % alt err → theta_cmd (Tf increased for derivative filtering)
p.pid_speed = struct('Kp',0.40, 'Ki',0.35, 'Kd',0.00, 'Tf',0.10, 'lo',-0.5,         'hi',0.7);         % speed err → delta_t (PI) (Ki increased for descent speed control)
p.pid_hdg   = struct('Kp',2.50, 'Ki',0.12, 'Kd',0.50, 'Tf',0.12, 'lo',-deg2rad(30),  'hi',deg2rad(30)); % heading err → phi_cmd

% Inner loop 1: Attitude → rate reference (P/PI, moderate bandwidth)
p.pid_pitch_att = struct('Kp',4.0, 'Ki',1.0, 'Kd',0.0, 'Tf',0.10, 'lo',-1.5, 'hi',1.5);   % theta err → q_ref [rad/s] (Kp reduced to prevent pitch oscillation)
p.pid_roll_att  = struct('Kp',6.0, 'Ki',0.6, 'Kd',0.0, 'Tf',0.05, 'lo',-2.5, 'hi',2.5);   % phi err → p_ref [rad/s]

% Inner loop 2: Rate → actuator (fast PID)
p.pid_pitch_rate = struct('Kp',1.5, 'Ki',0.5, 'Kd',0.01, 'Tf',0.02, 'lo',-0.7, 'hi',0.7);  % q err → delta_e
p.pid_roll_rate  = struct('Kp',1.0, 'Ki',0.2, 'Kd',0.01, 'Tf',0.02, 'lo',-0.7, 'hi',0.7);  % p err → delta_a

%% Trajectory — pick one mode:

% --- Mission profile: takeoff → level → loiter → descent → land ---
p.traj_type = 'mission';
p.mission.climb_angle_deg    = 15;       % climb flight-path angle [deg]
p.mission.descend_angle_deg  = 15;       % descent flight-path angle [deg]
p.mission.takeoff_alt        = 25;       % target cruise altitude [m]
p.mission.level_length       = 10;       % level segment after climb [m]
p.mission.loiter_radius      = 96;       % loiter circle radius [m] (+20%)
p.mission.loiter_turns       = 2;        % number of full loiter circles
p.mission.approach_length    = 10;       % level segment before descent [m]

% --- Sharp Square: takeoff → square patrol (sharp 90° turns) → RTL land ---
% p.traj_type = 'sharp_square';
% p.square.side_length   = 100;      % square side length [m]
% p.square.corner_radius = 10;       % corner turn radius [m] (small = sharp)
% p.square.cruise_alt    = 30;       % patrol altitude [m]
% p.square.num_laps      = 2;        % number of square laps

% --- Figure-8 ---
% p.traj_type = 'figure8';
% p.traj.radius   = 80;       % lobe radius [m]
% p.traj.alt      = 80;       % altitude [m]
% p.traj.center   = [0, 0];
% p.traj.yaw_mode = 'heading';
% p.traj.yaw_const = 0;

% --- Loiter circle ---
% p.traj_type = 'circle';
% p.traj.radius   = 60;       % circle radius [m]
% p.traj.alt      = 80;       % altitude [m]
% p.traj.omega    = p.u0 / p.traj.radius;
% p.traj.center   = [0, 0];
% p.traj.yaw_mode = 'heading';
% p.traj.yaw_const = 0;

% --- Racetrack mode ---
% p.traj_type = 'racetrack';
% p.traj.length  = 200;   p.traj.width = 80;
% p.traj.alt     = 80;
% p.traj.yaw_mode = 'heading';

% --- Waypoint mode ---
% p.traj_type = 'waypoint';
% p.traj.waypoints = [0 0 80; 200 0 80; 200 200 80; 0 200 80; 0 0 80];
% p.traj.wp_speed  = 15.0;
% p.traj.yaw_mode  = 'heading';
