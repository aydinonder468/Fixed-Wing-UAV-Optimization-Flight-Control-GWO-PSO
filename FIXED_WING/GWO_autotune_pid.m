%% GWO_autotune_pid.m — Grey Wolf Optimizer (GWO) PID Auto-Tuning
%  Cascaded architecture (no rudder, Bank-to-Turn, rate inner loops).
%  Optimizes Kp, Ki, Kd for all 7 active PID controllers simultaneously.
%
%  GWO: Mirjalili, Mirjalili & Lewis (2014) "Grey Wolf Optimizer"
%    Alpha (α) — best solution
%    Beta  (β) — second best
%    Delta (δ) — third best
%    Omega (ω) — remaining wolves
%
%  Tuned PIDs:  pid_alt, pid_speed, pid_hdg,
%               pid_pitch_att, pid_pitch_rate, pid_roll_att, pid_roll_rate
%  Parameters:  7 PIDs × 3 gains (Kp, Ki, Kd) = 21 dimensions
%
%  Usage:  Run this script. It saves optimized gains to gwo_best_gains.mat.
%          Then run main_fw.m to use the tuned response.

clear; clc; close all;

%% Add paths & load parameters
addpath('fixedwing', 'bms', 'ui');
fw_params;
bms_params;

%% Generate trajectory (use full mission duration for accurate evaluation)
p_tune = p;
p_tune.quiet   = true;
[t, xd, yd, zd, psid] = trajectory_fw(p_tune);
fprintf('GWO Tuning trajectory: %s, %.1f s, %d samples\n', p.traj_type, t(end), length(t));

%% ===== GWO CONFIGURATION =====
nPID   = 7;
nGain  = 3;
D      = nPID * nGain;   % 21

nWolves    = 60;     % pack size (increased from 40 for better convergence)
maxIter    = 80;     % maximum iterations (increased from 60)

%% Gain ordering:  [Kp Ki Kd] for each of 7 PIDs
%  Index:  1-3    pid_alt         (alt err -> theta_cmd)
%          4-6    pid_speed       (speed err -> d_t)
%          7-9    pid_hdg         (heading err -> phi_cmd)
%         10-12   pid_pitch_att   (theta err -> q_ref)
%         13-15   pid_pitch_rate  (q err -> d_e)
%         16-18   pid_roll_att    (phi err -> p_ref)
%         19-21   pid_roll_rate   (p err -> d_a)

lb = [0.10, 0.00, 0.00, ...   % alt
      0.15, 0.00, 0.00, ...   % speed (Kp min increased from 0.05)
      0.50, 0.00, 0.00, ...   % hdg
      2.00, 0.10, 0.00, ...   % pitch_att
      0.30, 0.05, 0.00, ...   % pitch_rate
      2.00, 0.10, 0.00, ...   % roll_att
      0.20, 0.05, 0.00];      % roll_rate

ub = [1.50, 0.30, 0.60, ...   % alt (Kd reduced)
      0.60, 0.25, 0.30, ...   % speed (Kp max reduced to 0.60, Ki max reduced to 0.25 to prevent windup)
      4.00, 0.50, 2.00, ...   % hdg (Kd increased)
      5.00, 1.50, 0.20, ...   % pitch_att (Kp max reduced to 5.00 to prevent oscillation, Kd reduced)
      3.00, 1.00, 0.15, ...   % pitch_rate (Kd reduced)
      8.00, 1.50, 0.50, ...   % roll_att
      2.00, 0.50, 0.15];      % roll_rate (Kd reduced)

%% ===== INITIALIZE WOLF PACK =====
pos = zeros(nWolves, D);       % wolf positions (gain vectors)
fitness = inf(nWolves, 1);     % fitness values (lower = better)

% Seed wolf #1 with current (manual) gains as warm start
pos(1,:) = [p.pid_alt.Kp,        p.pid_alt.Ki,        p.pid_alt.Kd, ...
            p.pid_speed.Kp,      p.pid_speed.Ki,      p.pid_speed.Kd, ...
            p.pid_hdg.Kp,        p.pid_hdg.Ki,        p.pid_hdg.Kd, ...
            p.pid_pitch_att.Kp,  p.pid_pitch_att.Ki,  p.pid_pitch_att.Kd, ...
            p.pid_pitch_rate.Kp, p.pid_pitch_rate.Ki, p.pid_pitch_rate.Kd, ...
            p.pid_roll_att.Kp,   p.pid_roll_att.Ki,   p.pid_roll_att.Kd, ...
            p.pid_roll_rate.Kp,  p.pid_roll_rate.Ki,  p.pid_roll_rate.Kd];

% Remaining wolves: random uniform between lb and ub
for i = 2:nWolves
    pos(i,:) = lb + rand(1,D) .* (ub - lb);
end

% Alpha, Beta, Delta — best three solutions
alpha_pos   = zeros(1,D);  alpha_score = inf;
beta_pos    = zeros(1,D);  beta_score  = inf;
delta_pos   = zeros(1,D);  delta_score = inf;

%% Pre-compute plant (only depends on aero derivatives, NOT PID gains)
plant = create_plant_fw(p_tune);

%% ===== GWO MAIN LOOP =====
cost_history = zeros(maxIter, 1);

fprintf('\n===== GWO Auto-Tuning Started =====\n');
fprintf('  Wolves: %d,  Iterations: %d,  Dimensions: %d\n', nWolves, maxIter, D);
fprintf('  Fitness = position_error + 0.05*control_effort + 0.3*oscillation + 0.5*saturation\n\n');

tic;
for iter = 1:maxIter
    % 'a' linearly decreases from 2 to 0 over iterations
    a = 2 - 2 * (iter / maxIter);

    %% --- Phase 1: Evaluate fitness of all wolves ---
    for i = 1:nWolves
        % Clamp position to bounds
        pos(i,:) = max(lb, min(ub, pos(i,:)));

        % Apply gains to parameter struct
        p_eval = apply_pid_gain_vector(p_tune, pos(i,:));

        % Evaluate fitness via fast simulation
        try
            fitness(i) = simulate_fw_fast(p_eval, t, xd, yd, zd, psid, plant);
        catch
            fitness(i) = 1e6;
        end

        % Update hierarchy: Alpha > Beta > Delta
        if fitness(i) < alpha_score
            % New alpha; cascade previous alpha → beta → delta
            delta_score = beta_score;  delta_pos = beta_pos;
            beta_score  = alpha_score; beta_pos  = alpha_pos;
            alpha_score = fitness(i);  alpha_pos = pos(i,:);
        elseif fitness(i) < beta_score
            delta_score = beta_score;  delta_pos = beta_pos;
            beta_score  = fitness(i);  beta_pos  = pos(i,:);
        elseif fitness(i) < delta_score
            delta_score = fitness(i);  delta_pos = pos(i,:);
        end
    end

    cost_history(iter) = alpha_score;

    % Print progress
    if mod(iter, 5) == 1 || iter == maxIter
        fprintf('  Iter %3d/%d  |  Alpha Cost = %10.2f  |  a = %.3f  |  Elapsed: %.1fs\n', ...
            iter, maxIter, alpha_score, a, toc);
    end

    %% --- Phase 2: Update wolf positions toward Alpha, Beta, Delta ---
    for i = 1:nWolves
        % Vectorized GWO position update
        r1 = rand(1,D);  r2 = rand(1,D);
        A1 = 2*a*r1 - a;
        C1 = 2*r2;
        D_alpha = abs(C1 .* alpha_pos - pos(i,:));
        X1 = alpha_pos - A1 .* D_alpha;

        r1 = rand(1,D);  r2 = rand(1,D);
        A2 = 2*a*r1 - a;
        C2 = 2*r2;
        D_beta = abs(C2 .* beta_pos - pos(i,:));
        X2 = beta_pos - A2 .* D_beta;

        r1 = rand(1,D);  r2 = rand(1,D);
        A3 = 2*a*r1 - a;
        C3 = 2*r2;
        D_delta = abs(C3 .* delta_pos - pos(i,:));
        X3 = delta_pos - A3 .* D_delta;

        % New position = average of three leader-guided positions
        pos(i,:) = (X1 + X2 + X3) / 3;

        % Enforce search space bounds
        pos(i,:) = max(lb, min(ub, pos(i,:)));
    end
end
elapsed = toc;

%% ===== RESULTS =====
fprintf('\n===== GWO Optimization Complete =====\n');
fprintf('  Total time:  %.1f s\n', elapsed);
fprintf('  Alpha cost:  %.4f\n', alpha_score);
fprintf('  Beta  cost:  %.4f\n', beta_score);
fprintf('  Delta cost:  %.4f\n\n', delta_score);

g = alpha_pos;
pid_names = {'pid_alt', 'pid_speed', 'pid_hdg', 'pid_pitch_att', ...
             'pid_pitch_rate', 'pid_roll_att', 'pid_roll_rate'};
fprintf('  %-16s    Kp        Ki        Kd\n', 'PID Loop');
fprintf('  %s\n', repmat('-', 1, 52));
for j = 1:nPID
    idx = (j-1)*3 + (1:3);
    fprintf('  %-16s  %7.4f   %7.4f   %7.4f\n', pid_names{j}, g(idx(1)), g(idx(2)), g(idx(3)));
end

%% ===== SAVE BEST GAINS TO FILE =====
gwo_gain_vector = alpha_pos;   %#ok<NASGU>
gwo_best_cost   = alpha_score; %#ok<NASGU>
save('gwo_best_gains.mat', 'gwo_gain_vector', 'gwo_best_cost');
fprintf('\n  >> Gains saved to gwo_best_gains.mat\n');

%% ===== PRINT READY-TO-PASTE GAINS =====
fprintf('\n--- Optimized PID gain structs (paste into fw_params.m) ---\n\n');

Tf_vals = [p.pid_alt.Tf, p.pid_speed.Tf, p.pid_hdg.Tf, ...
           p.pid_pitch_att.Tf, p.pid_pitch_rate.Tf, ...
           p.pid_roll_att.Tf, p.pid_roll_rate.Tf];
lo_vals = {p.pid_alt.lo, p.pid_speed.lo, p.pid_hdg.lo, ...
           p.pid_pitch_att.lo, p.pid_pitch_rate.lo, ...
           p.pid_roll_att.lo, p.pid_roll_rate.lo};
hi_vals = {p.pid_alt.hi, p.pid_speed.hi, p.pid_hdg.hi, ...
           p.pid_pitch_att.hi, p.pid_pitch_rate.hi, ...
           p.pid_roll_att.hi, p.pid_roll_rate.hi};

for j = 1:nPID
    idx = (j-1)*3 + (1:3);
    fprintf("p.%s = struct('Kp',%.4f, 'Ki',%.4f, 'Kd',%.4f, 'Tf',%.2f, 'lo',%.4f, 'hi',%.4f);\n", ...
        pid_names{j}, g(idx(1)), g(idx(2)), g(idx(3)), ...
        Tf_vals(j), lo_vals{j}, hi_vals{j});
end

%% ===== APPLY & RUN FULL SIMULATION =====
fprintf('\n--- Running full simulation with GWO-optimized gains ---\n');
p_opt = apply_pid_gain_vector(p, alpha_pos);
p_opt.gain_sets.manual = [p.pid_alt.Kp, p.pid_alt.Ki, p.pid_alt.Kd, ...
    p.pid_speed.Kp, p.pid_speed.Ki, p.pid_speed.Kd, ...
    p.pid_hdg.Kp, p.pid_hdg.Ki, p.pid_hdg.Kd, ...
    p.pid_pitch_att.Kp, p.pid_pitch_att.Ki, p.pid_pitch_att.Kd, ...
    p.pid_pitch_rate.Kp, p.pid_pitch_rate.Ki, p.pid_pitch_rate.Kd, ...
    p.pid_roll_att.Kp, p.pid_roll_att.Ki, p.pid_roll_att.Kd, ...
    p.pid_roll_rate.Kp, p.pid_roll_rate.Ki, p.pid_roll_rate.Kd];
p_opt.gain_sets.gwo = alpha_pos;
p_opt.gain_sets.pso = [];
p_opt.active_control = 'gwo';
[t_full, xd_f, yd_f, zd_f, psid_f] = trajectory_fw(p_opt);
[rec, bms_rec] = simulate_fw(p_opt, t_full, xd_f, yd_f, zd_f, psid_f);
mfeas = mission_feasibility(p_opt, rec, bms_rec, t_full);
fw_monitor(t_full, rec, bms_rec, p_opt, mfeas, xd_f, yd_f, zd_f, psid_f);

%% ===== CONVERGENCE PLOT =====
figure('Color','w','Name','GWO Convergence','NumberTitle','off');
semilogy(1:maxIter, cost_history, 'r-', 'LineWidth', 1.5);
xlabel('Iteration'); ylabel('Alpha Cost (log)');
title('GWO Auto-Tune Convergence'); grid on;

fprintf('\nGWO auto-tuning complete. Review plots and copy gains to fw_params.m.\n');
