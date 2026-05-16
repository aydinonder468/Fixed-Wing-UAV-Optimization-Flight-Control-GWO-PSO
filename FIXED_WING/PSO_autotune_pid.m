%% PSO_autotune_fw.m — PSO-based PID Auto-Tuning for Fixed-Wing UAV
%  Cascaded architecture (no rudder, Bank-to-Turn, rate inner loops).
%  Optimizes Kp, Ki, Kd for all 7 active PID controllers simultaneously.
%
%  Tuned PIDs:  pid_alt, pid_speed, pid_hdg,
%               pid_pitch_att, pid_pitch_rate, pid_roll_att, pid_roll_rate
%  Parameters:  7 PIDs x 3 gains (Kp, Ki, Kd) = 21 dimensions

clear; clc; close all;

%% Add paths & load parameters
addpath('fixedwing', 'bms', 'ui');
fw_params;
bms_params;

%% Generate trajectory (use full mission duration for accurate evaluation)
p_tune = p;
p_tune.quiet   = true;
[t, xd, yd, zd, psid] = trajectory_fw(p_tune);
fprintf('PSO Tuning trajectory: %s, %.1f s, %d samples\n', p.traj_type, t(end), length(t));

%% ===== PSO CONFIGURATION =====
nPID   = 7;
nGain  = 3;
D      = nPID * nGain;   % 21

nPart      = 60;     % swarm size (increased from 40)
maxIter    = 80;     % maximum iterations (increased from 60)
W_start    = 0.9;
W_end      = 0.4;
C1         = 2.0;
C2         = 2.0;

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

ub = [1.50, 0.30, 0.60, ...   % alt (Kd reduced from 1.50 to 0.60)
      0.60, 0.25, 0.30, ...   % speed (Kp max reduced to 0.60, Ki max reduced to 0.25)
      4.00, 0.50, 2.00, ...   % hdg (Kd increased to 2.00)
      5.00, 1.50, 0.20, ...   % pitch_att (Kp max reduced to 5.00, Kd reduced to 0.20)
      3.00, 1.00, 0.15, ...   % pitch_rate (Kd reduced to 0.15)
      8.00, 1.50, 0.50, ...   % roll_att
      2.00, 0.50, 0.15];      % roll_rate (Kd reduced to 0.15)

%% ===== INITIALIZE SWARM =====
pos = zeros(nPart, D);
vel = zeros(nPart, D);
pbest_pos = zeros(nPart, D);
pbest_val = inf(nPart, 1);
gbest_pos = zeros(1, D);
gbest_val = inf;

% Seed particle #1 with current gains
pos(1,:) = [p.pid_alt.Kp,        p.pid_alt.Ki,        p.pid_alt.Kd, ...
            p.pid_speed.Kp,      p.pid_speed.Ki,      p.pid_speed.Kd, ...
            p.pid_hdg.Kp,        p.pid_hdg.Ki,        p.pid_hdg.Kd, ...
            p.pid_pitch_att.Kp,  p.pid_pitch_att.Ki,  p.pid_pitch_att.Kd, ...
            p.pid_pitch_rate.Kp, p.pid_pitch_rate.Ki, p.pid_pitch_rate.Kd, ...
            p.pid_roll_att.Kp,   p.pid_roll_att.Ki,   p.pid_roll_att.Kd, ...
            p.pid_roll_rate.Kp,  p.pid_roll_rate.Ki,  p.pid_roll_rate.Kd];

for i = 2:nPart
    pos(i,:) = lb + rand(1,D) .* (ub - lb);
end

vel = 0.1 * (rand(nPart, D) - 0.5) .* (ub - lb);

%% Pre-compute plant
plant = create_plant_fw(p_tune);

%% ===== PSO MAIN LOOP =====
cost_history = zeros(maxIter, 1);

fprintf('\n===== PSO Auto-Tuning Started =====\n');
fprintf('  Particles: %d,  Iterations: %d,  Dimensions: %d\n', nPart, maxIter, D);
fprintf('  Fitness = position_error + 0.50*control_effort + 0.5*oscillation + 0.5*saturation\n\n');

tic;
for iter = 1:maxIter
    W = W_start - (W_start - W_end) * (iter - 1) / (maxIter - 1);

    for i = 1:nPart
        pos(i,:) = max(lb, min(ub, pos(i,:)));
        p_eval = apply_pid_gain_vector(p_tune, pos(i,:));

        try
            cost = simulate_fw_fast(p_eval, t, xd, yd, zd, psid, plant);
        catch
            cost = 1e6;
        end

        if cost < pbest_val(i)
            pbest_val(i)    = cost;
            pbest_pos(i,:)  = pos(i,:);
        end
        if cost < gbest_val
            gbest_val = cost;
            gbest_pos = pos(i,:);
        end
    end

    cost_history(iter) = gbest_val;

    if mod(iter, 5) == 1 || iter == maxIter
        fprintf('  Iter %3d/%d  |  Best Cost = %10.2f  |  W = %.3f  |  Elapsed: %.1fs\n', ...
            iter, maxIter, gbest_val, W, toc);
    end

    for i = 1:nPart
        r1 = rand(1, D);
        r2 = rand(1, D);
        vel(i,:) = W * vel(i,:) ...
                 + C1 * r1 .* (pbest_pos(i,:) - pos(i,:)) ...
                 + C2 * r2 .* (gbest_pos       - pos(i,:));
        v_max = 0.3 * (ub - lb);
        vel(i,:) = max(-v_max, min(v_max, vel(i,:)));
        pos(i,:) = pos(i,:) + vel(i,:);
    end
end
elapsed = toc;

%% ===== RESULTS =====
fprintf('\n===== PSO Optimization Complete =====\n');
fprintf('  Total time:  %.1f s\n', elapsed);
fprintf('  Best cost:   %.4f\n\n', gbest_val);

g = gbest_pos;
pid_names = {'pid_alt', 'pid_speed', 'pid_hdg', 'pid_pitch_att', 'pid_pitch_rate', 'pid_roll_att', 'pid_roll_rate'};
fprintf('  %-16s    Kp        Ki        Kd\n', 'PID Loop');
fprintf('  %s\n', repmat('-', 1, 52));
for j = 1:nPID
    idx = (j-1)*3 + (1:3);
    fprintf('  %-16s  %7.4f   %7.4f   %7.4f\n', pid_names{j}, g(idx(1)), g(idx(2)), g(idx(3)));
end

%% ===== SAVE =====
pso_gain_vector = gbest_pos;  %#ok<NASGU>
pso_best_cost   = gbest_val;  %#ok<NASGU>
save('pso_best_gains.mat', 'pso_gain_vector', 'pso_best_cost');
fprintf('\n  >> Gains saved to pso_best_gains.mat\n');

%% ===== PRINT GAINS =====
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
fprintf('\n--- Running full simulation with optimized gains ---\n');
p_opt = apply_pid_gain_vector(p, gbest_pos);
p_opt.gain_sets.manual = [p.pid_alt.Kp, p.pid_alt.Ki, p.pid_alt.Kd, ...
    p.pid_speed.Kp, p.pid_speed.Ki, p.pid_speed.Kd, ...
    p.pid_hdg.Kp, p.pid_hdg.Ki, p.pid_hdg.Kd, ...
    p.pid_pitch_att.Kp, p.pid_pitch_att.Ki, p.pid_pitch_att.Kd, ...
    p.pid_pitch_rate.Kp, p.pid_pitch_rate.Ki, p.pid_pitch_rate.Kd, ...
    p.pid_roll_att.Kp, p.pid_roll_att.Ki, p.pid_roll_att.Kd, ...
    p.pid_roll_rate.Kp, p.pid_roll_rate.Ki, p.pid_roll_rate.Kd];
p_opt.gain_sets.gwo = [];
p_opt.gain_sets.pso = gbest_pos;
p_opt.active_control = 'pso';
[t_full, xd_f, yd_f, zd_f, psid_f] = trajectory_fw(p_opt);
[rec, bms_rec] = simulate_fw(p_opt, t_full, xd_f, yd_f, zd_f, psid_f);
mfeas = mission_feasibility(p_opt, rec, bms_rec, t_full);
fw_monitor(t_full, rec, bms_rec, p_opt, mfeas, xd_f, yd_f, zd_f, psid_f);

%% ===== CONVERGENCE PLOT =====
figure('Color','w','Name','PSO Convergence','NumberTitle','off');
semilogy(1:maxIter, cost_history, 'b-', 'LineWidth', 1.5);
xlabel('Iteration'); ylabel('Best Cost (log)');
title('PSO Auto-Tune Convergence'); grid on;

fprintf('\nPSO auto-tuning complete. Review plots and copy gains to fw_params.m.\n');
