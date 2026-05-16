function [t, xd, yd, zd, psid] = trajectory_fw(p)
%TRAJECTORY_FW  Generate fixed-wing reference trajectories.
%  [t, xd, yd, zd, psid] = trajectory_fw(p)
%
%  Modes: 'circle'         — loiter circle at constant altitude
%         'figure8'        — figure-8 (lemniscate) pattern
%         'racetrack'      — oval racetrack pattern
%         'waypoint'       — fly through waypoints
%         'mission'        — takeoff → level → loiter → approach → descent → land
%         'figure8_mission'— takeoff → figure-8 (x2) → descent → land
%         'spiral_climb'   — helisel tırmanış (geniş daire, sabit açı, 200m)
%         'recon_square'   — takeoff → kare keşif rotası → RTL iniş

dt = p.dt;

switch p.traj_type
    case 'circle'
        N = round(p.T_final / dt);
        t = (0:N-1) * dt;
        xd = zeros(1,N); yd = zeros(1,N); zd = zeros(1,N); psid = zeros(1,N);
        
        R   = p.traj.radius;
        alt = p.traj.alt;
        cx  = p.traj.center(1);
        cy  = p.traj.center(2);
        omega = p.u0 / R;    % angular rate to maintain trim speed
        
        for k = 1:N
            tk = t(k);
            ang = omega * tk;
            xd(k) = cx + R * cos(ang);
            yd(k) = cy + R * sin(ang);
            zd(k) = alt;
            psid(k) = ang + pi/2;   % tangent direction
            % Wrap to [-pi, pi]
            psid(k) = atan2(sin(psid(k)), cos(psid(k)));
        end
        
        fprintf('Trajectory: circle, R=%.0fm, alt=%.0fm, T=%.0fs\n', R, alt, t(end));
        
    case 'figure8'
        N = round(p.T_final / dt);
        t = (0:N-1) * dt;
        xd = zeros(1,N); yd = zeros(1,N); zd = zeros(1,N); psid = zeros(1,N);
        
        R   = p.traj.radius;
        alt = p.traj.alt;
        cx  = p.traj.center(1);
        cy  = p.traj.center(2);
        omega = p.u0 / (R * sqrt(2));  % scale omega for figure-8 path length
        
        for k = 1:N
            tk = t(k);
            ang = omega * tk;
            % Lemniscate of Gerono: x = R*cos(ang), y = R*sin(ang)*cos(ang)
            xd(k) = cx + R * cos(ang);
            yd(k) = cy + R * sin(ang) * cos(ang);
            zd(k) = alt;
            
            % Heading from tangent: dx/dt, dy/dt
            dx = -R * sin(ang) * omega;
            dy = R * (cos(2*ang)) * omega;  % d/dt[sin*cos] = cos(2t)
            psid(k) = atan2(dy, dx);
        end
        
        fprintf('Trajectory: figure-8, R=%.0fm, alt=%.0fm, T=%.0fs\n', R, alt, t(end));
        
    case 'racetrack'
        N = round(p.T_final / dt);
        t = (0:N-1) * dt;
        xd = zeros(1,N); yd = zeros(1,N); zd = zeros(1,N); psid = zeros(1,N);
        
        L = p.traj.length;   % straight segment length
        R = p.traj.width/2;  % semicircle radius
        alt = p.traj.alt;
        
        % Total perimeter
        perim = 2*L + 2*pi*R;
        
        for k = 1:N
            tk = t(k);
            dist = mod(p.u0 * tk, perim);  % distance along track
            
            if dist < L
                % Bottom straight (left to right)
                xd(k) = dist;
                yd(k) = 0;
                psid(k) = 0;
            elseif dist < L + pi*R
                % Right semicircle
                ang = (dist - L) / R;
                xd(k) = L + R*sin(ang);
                yd(k) = R*(1 - cos(ang));
                psid(k) = ang;
            elseif dist < 2*L + pi*R
                % Top straight (right to left)
                frac = dist - L - pi*R;
                xd(k) = L - frac;
                yd(k) = 2*R;
                psid(k) = pi;
            else
                % Left semicircle
                ang = (dist - 2*L - pi*R) / R;
                xd(k) = -R*sin(ang);
                yd(k) = 2*R - R*(1 - cos(ang));
                psid(k) = pi + ang;
            end
            zd(k) = alt;
            psid(k) = atan2(sin(psid(k)), cos(psid(k)));
        end
        
        fprintf('Trajectory: racetrack, L=%.0fm, W=%.0fm, T=%.0fs\n', L, 2*R, t(end));
        
    case 'waypoint'
        wp  = p.traj.waypoints;
        spd = p.traj.wp_speed;
        
        % Total path length
        nw = size(wp,1);
        seg_len = zeros(nw-1,1);
        for j = 1:nw-1
            seg_len(j) = norm(wp(j+1,:) - wp(j,:));
        end
        cum_len = [0; cumsum(seg_len)];
        total_dist = cum_len(end);
        
        T_total = total_dist / spd + 5.0;  % + 5s settle
        N = round(T_total / dt);
        t = (0:N-1) * dt;
        xd = zeros(1,N); yd = zeros(1,N); zd = zeros(1,N); psid = zeros(1,N);
        
        for k = 1:N
            tk = t(k);
            dist = spd * tk;
            
            if dist >= cum_len(end)
                pt = wp(end,:);
            else
                seg = find(cum_len(2:end) >= dist, 1);
                if isempty(seg); seg = nw-1; end
                if seg_len(seg) > 0
                    frac = (dist - cum_len(seg)) / seg_len(seg);
                else
                    frac = 0;
                end
                pt = wp(seg,:) + frac*(wp(seg+1,:) - wp(seg,:));
            end
            xd(k) = pt(1); yd(k) = pt(2); zd(k) = pt(3);
            
            % Yaw = heading toward next point
            if k > 1
                psid(k) = atan2(yd(k)-yd(k-1), xd(k)-xd(k-1));
            end
        end
        
        fprintf('Trajectory: waypoint, dist=%.0fm, T=%.1fs\n', total_dist, t(end));

    case 'mission'
        %% ===== FULL MISSION: takeoff → level → loiter → approach → land =====
        %  All segments are built as dense waypoint arrays at simulation dt,
        %  heading is derived from the path tangent for smoothness.

        V = p.u0;  % constant ground-speed assumption

        % --- Read mission parameters (with safe defaults) ---
        if isfield(p, 'mission')
            m = p.mission;
        else
            m = struct();
        end
        gamma_climb   = deg2rad(get_or(m, 'climb_angle_deg',   15));
        gamma_desc    = deg2rad(get_or(m, 'descend_angle_deg', 15));
        alt_cruise    = get_or(m, 'takeoff_alt',      25);
        L_level       = get_or(m, 'level_length',     10);
        R_loiter      = get_or(m, 'loiter_radius',    80);
        n_turns       = get_or(m, 'loiter_turns',      2);
        L_approach    = get_or(m, 'approach_length',  10);

        % --- Segment geometry ---
        % 1. Climb:  horizontal distance to reach alt_cruise at gamma_climb
        L_climb_h = alt_cruise / tan(gamma_climb);       % horizontal [m]
        L_climb_s = alt_cruise / sin(gamma_climb);       % slant (arc length) [m]

        % 2. Level:  flat segment at alt_cruise
        %    (L_level already defined)

        % 3. Loiter entry: straight tangent into circle
        %    Circle center offset from end of level segment
        loiter_circ  = 2 * pi * R_loiter * n_turns;      % total arc length

        % 4. Approach: flat segment at alt_cruise after loiter exit

        % 5. Descent: mirror of climb
        L_desc_h = alt_cruise / tan(gamma_desc);
        L_desc_s = alt_cruise / sin(gamma_desc);

        % --- Time for each segment ---
        T_climb    = L_climb_s / V;
        T_level    = L_level   / V;
        T_loiter   = loiter_circ / V;
        T_approach = L_approach / V;
        T_desc     = L_desc_s  / V;
        T_settle   = 3.0;  % hover at end

        T_total = T_climb + T_level + T_loiter + T_approach + T_desc + T_settle;
        N = round(T_total / dt);
        t = (0:N-1) * dt;
        xd = zeros(1,N); yd = zeros(1,N); zd = zeros(1,N); psid = zeros(1,N);

        % Cumulative time boundaries
        t1 = T_climb;
        t2 = t1 + T_level;
        t3 = t2 + T_loiter;
        t4 = t3 + T_approach;
        t5 = t4 + T_desc;

        % --- X-axis offsets at segment boundaries ---
        x_end_climb    = L_climb_h;
        x_end_level    = x_end_climb + L_level;

        % Loiter circle center: offset in +Y from end-of-level by R_loiter
        % so the aircraft enters tangentially flying in +X direction.
        cx_loiter = x_end_level;
        cy_loiter = R_loiter;

        % After full loiter, exit point is same as entry: (x_end_level, 0)
        x_end_approach = x_end_level + L_approach;
        % Descent ends at ground
        x_end_desc     = x_end_approach + L_desc_h;

        omega_loiter = V / R_loiter;   % angular rate [rad/s]

        for k = 1:N
            tk = t(k);

            if tk < t1
                % --- SEGMENT 1: Climb at gamma_climb ---
                frac = tk / T_climb;
                xd(k) = frac * L_climb_h;
                yd(k) = 0;
                zd(k) = frac * alt_cruise;
                psid(k) = 0;   % flying along +X

            elseif tk < t2
                % --- SEGMENT 2: Level flight at alt_cruise ---
                dt_seg = tk - t1;
                xd(k) = x_end_climb + V * dt_seg;
                yd(k) = 0;
                zd(k) = alt_cruise;
                psid(k) = 0;

            elseif tk < t3
                % --- SEGMENT 3: Loiter circle (CCW in XY) ---
                dt_seg = tk - t2;
                ang = -pi/2 + omega_loiter * dt_seg;  % start at bottom (y=0)
                xd(k) = cx_loiter + R_loiter * cos(ang);
                yd(k) = cy_loiter + R_loiter * sin(ang);
                zd(k) = alt_cruise;
                % Tangent heading for CCW circle
                psid(k) = ang + pi/2;

            elseif tk < t4
                % --- SEGMENT 4: Approach (level, +X direction) ---
                dt_seg = tk - t3;
                xd(k) = x_end_level + V * dt_seg;
                yd(k) = 0;
                zd(k) = alt_cruise;
                psid(k) = 0;

            elseif tk < t5
                % --- SEGMENT 5: Descent at gamma_desc ---
                frac = (tk - t4) / T_desc;
                xd(k) = x_end_approach + frac * L_desc_h;
                yd(k) = 0;
                zd(k) = alt_cruise * (1 - frac);
                psid(k) = 0;

            else
                % --- SEGMENT 6: Settle on ground ---
                xd(k) = x_end_desc;
                yd(k) = 0;
                zd(k) = 0;
                psid(k) = 0;
            end

            % Clamp altitude non-negative
            zd(k) = max(0, zd(k));
            % Wrap heading to [-pi, pi]
            psid(k) = atan2(sin(psid(k)), cos(psid(k)));
        end

        %% Post-process: smooth heading across segment transitions
        % Low-pass filter psid using a short moving-average with angle wrapping
        smooth_win = max(1, round(0.5 / dt));   % 0.5s smoothing window
        psid_smooth = psid;
        for k = 2:N
            % Unwrap difference, blend, re-wrap
            diff = atan2(sin(psid(k) - psid_smooth(k-1)), cos(psid(k) - psid_smooth(k-1)));
            alpha_s = min(1.0, dt * smooth_win * 2 / 0.5);  % blending factor
            psid_smooth(k) = psid_smooth(k-1) + alpha_s * diff;
            psid_smooth(k) = atan2(sin(psid_smooth(k)), cos(psid_smooth(k)));
        end
        psid = psid_smooth;

        fprintf('Trajectory: mission profile\n');
        fprintf('  Climb:    %.0fm horiz, gamma=%.0f°, alt=%.0fm\n', L_climb_h, rad2deg(gamma_climb), alt_cruise);
        fprintf('  Level:    %.0fm\n', L_level);
        fprintf('  Loiter:   R=%.0fm, %d turns (%.0fm arc)\n', R_loiter, n_turns, loiter_circ);
        fprintf('  Approach: %.0fm\n', L_approach);
        fprintf('  Descent:  %.0fm horiz, gamma=%.0f°\n', L_desc_h, rad2deg(gamma_desc));
        fprintf('  Total:    %.1fs, %d samples\n', t(end), N);

    case 'figure8_mission'
        %% ===== TAKEOFF → FIGURE-8 (x2) → DESCENT → LAND =====
        V = p.u0;

        if isfield(p, 'fig8m')
            m = p.fig8m;
        else
            m = struct();
        end
        gamma_climb = deg2rad(get_or(m, 'climb_angle_deg', 15));
        gamma_desc  = deg2rad(get_or(m, 'descend_angle_deg', 8));
        alt_cruise  = get_or(m, 'cruise_alt', 50);
        R_lobe      = get_or(m, 'lobe_radius', 100);
        n_fig8      = get_or(m, 'num_fig8', 2);

        % Climb segment
        L_climb_h = alt_cruise / tan(gamma_climb);
        L_climb_s = alt_cruise / sin(gamma_climb);
        T_climb   = L_climb_s / V;

        % Level transition before figure-8
        L_level = 20;
        T_level = L_level / V;

        % Figure-8 arc length per loop: ~4*pi*R/sqrt(2)
        fig8_loop_len = 4 * pi * R_lobe / sqrt(2);
        T_fig8 = (fig8_loop_len * n_fig8) / V;
        omega_fig8 = V / (R_lobe * sqrt(2));

        % Level transition after figure-8
        T_approach = L_level / V;

        % Descent
        L_desc_h = alt_cruise / tan(gamma_desc);
        L_desc_s = alt_cruise / sin(gamma_desc);
        T_desc   = L_desc_s / V;

        T_settle = 3.0;
        T_total = T_climb + T_level + T_fig8 + T_approach + T_desc + T_settle;
        N = round(T_total / dt);
        t = (0:N-1) * dt;
        xd = zeros(1,N); yd = zeros(1,N); zd = zeros(1,N); psid = zeros(1,N);

        t1 = T_climb;
        t2 = t1 + T_level;
        t3 = t2 + T_fig8;
        t4 = t3 + T_approach;
        t5 = t4 + T_desc;

        x_end_climb = L_climb_h;
        x_end_level = x_end_climb + L_level;
        % Figure-8 center = end of level segment
        cx_fig8 = x_end_level + R_lobe;
        cy_fig8 = 0;

        for k = 1:N
            tk = t(k);

            if tk < t1
                % Climb
                frac = tk / T_climb;
                xd(k) = frac * L_climb_h;
                yd(k) = 0;
                zd(k) = frac * alt_cruise;
                psid(k) = 0;

            elseif tk < t2
                % Level transition
                dt_seg = tk - t1;
                xd(k) = x_end_climb + V * dt_seg;
                yd(k) = 0;
                zd(k) = alt_cruise;
                psid(k) = 0;

            elseif tk < t3
                % Figure-8 (lemniscate of Gerono)
                dt_seg = tk - t2;
                ang = omega_fig8 * dt_seg;
                xd(k) = cx_fig8 + R_lobe * cos(ang);
                yd(k) = cy_fig8 + R_lobe * sin(ang) * cos(ang);
                zd(k) = alt_cruise;
                % Heading from tangent
                dx_dt = -R_lobe * sin(ang) * omega_fig8;
                dy_dt = R_lobe * cos(2*ang) * omega_fig8;
                psid(k) = atan2(dy_dt, dx_dt);

            elseif tk < t4
                % Level approach back on +X axis
                dt_seg = tk - t3;
                % Exit figure-8 at (cx_fig8+R_lobe, 0) heading +X
                xd(k) = cx_fig8 + R_lobe + V * dt_seg;
                yd(k) = 0;
                zd(k) = alt_cruise;
                psid(k) = 0;

            elseif tk < t5
                % Gentle descent
                frac = (tk - t4) / T_desc;
                x_start_desc = cx_fig8 + R_lobe + L_level;
                xd(k) = x_start_desc + frac * L_desc_h;
                yd(k) = 0;
                zd(k) = alt_cruise * (1 - frac);
                psid(k) = 0;

            else
                % Settle
                x_end_all = cx_fig8 + R_lobe + L_level + L_desc_h;
                xd(k) = x_end_all;
                yd(k) = 0;
                zd(k) = 0;
                psid(k) = 0;
            end

            zd(k) = max(0, zd(k));
            psid(k) = atan2(sin(psid(k)), cos(psid(k)));
        end

        % Smooth heading
        psid = smooth_heading(psid, dt, N);

        fprintf('Trajectory: figure-8 mission\n');
        fprintf('  Climb:  gamma=%.0f°, alt=%.0fm\n', rad2deg(gamma_climb), alt_cruise);
        fprintf('  Fig-8:  R=%.0fm, %d loops\n', R_lobe, n_fig8);
        fprintf('  Desc:   gamma=%.0f°\n', rad2deg(gamma_desc));
        fprintf('  Total:  %.1fs, %d samples\n', t(end), N);

    case 'spiral_climb'
        %% ===== HELISEL TIRMANIS — genis daire, sabit aci, 200m =====
        V = p.u0;

        if isfield(p, 'spiral')
            m = p.spiral;
        else
            m = struct();
        end
        R_spiral    = get_or(m, 'radius', 120);
        alt_target  = get_or(m, 'alt_target', 200);
        gamma_climb = deg2rad(get_or(m, 'climb_angle_deg', 15));
        T_settle    = 5.0;

        % Vertical climb rate
        Vz = V * sin(gamma_climb);
        % Horizontal speed component
        Vh = V * cos(gamma_climb);
        % Angular rate on circle
        omega_sp = Vh / R_spiral;

        % Time to reach target altitude
        T_climb = alt_target / Vz;

        T_total = T_climb + T_settle;
        N = round(T_total / dt);
        t = (0:N-1) * dt;
        xd = zeros(1,N); yd = zeros(1,N); zd = zeros(1,N); psid = zeros(1,N);

        % Circle center at (0, R_spiral) — aircraft starts at (0,0)
        cx_sp = 0;
        cy_sp = R_spiral;

        for k = 1:N
            tk = t(k);

            if tk <= T_climb
                ang = -pi/2 + omega_sp * tk;
                xd(k) = cx_sp + R_spiral * cos(ang);
                yd(k) = cy_sp + R_spiral * sin(ang);
                zd(k) = Vz * tk;
                % CCW tangent heading
                psid(k) = ang + pi/2;
            else
                % Hold at top
                ang_final = -pi/2 + omega_sp * T_climb;
                xd(k) = cx_sp + R_spiral * cos(ang_final);
                yd(k) = cy_sp + R_spiral * sin(ang_final);
                zd(k) = alt_target;
                psid(k) = ang_final + pi/2;
            end

            zd(k) = max(0, zd(k));
            psid(k) = atan2(sin(psid(k)), cos(psid(k)));
        end

        n_spiral_turns = (omega_sp * T_climb) / (2*pi);
        fprintf('Trajectory: spiral climb\n');
        fprintf('  Radius: %.0fm, Target alt: %.0fm\n', R_spiral, alt_target);
        fprintf('  Climb angle: %.0f°, Vz=%.1f m/s\n', rad2deg(gamma_climb), Vz);
        fprintf('  Turns: %.1f, Total: %.1fs, %d samples\n', n_spiral_turns, t(end), N);

    case 'recon_square'
        %% ===== TAKEOFF → KARE KESIF ROTASI → RTL INIS =====
        V = p.u0;

        if isfield(p, 'recon')
            m = p.recon;
        else
            m = struct();
        end
        gamma_climb = deg2rad(get_or(m, 'climb_angle_deg', 15));
        gamma_desc  = deg2rad(get_or(m, 'descend_angle_deg', 10));
        alt_cruise  = get_or(m, 'cruise_alt', 60);
        side_len    = get_or(m, 'side_length', 200);
        R_corner    = get_or(m, 'corner_radius', 40);

        % Climb
        L_climb_h = alt_cruise / tan(gamma_climb);
        L_climb_s = alt_cruise / sin(gamma_climb);
        T_climb   = L_climb_s / V;

        % Level transition
        L_level = 15;
        T_level = L_level / V;

        % Square with rounded corners: 4 straight + 4 quarter-circle turns
        L_straight = side_len - 2*R_corner;
        L_corner_arc = pi/2 * R_corner;
        L_square = 4 * L_straight + 4 * L_corner_arc;
        T_square = L_square / V;

        % Return leg (fly back to start on +X axis)
        % After completing square, aircraft is back near start
        T_approach = L_level / V;

        % Descent
        L_desc_h = alt_cruise / tan(gamma_desc);
        L_desc_s = alt_cruise / sin(gamma_desc);
        T_desc   = L_desc_s / V;

        T_settle = 3.0;
        T_total = T_climb + T_level + T_square + T_approach + T_desc + T_settle;
        N = round(T_total / dt);
        t = (0:N-1) * dt;
        xd = zeros(1,N); yd = zeros(1,N); zd = zeros(1,N); psid = zeros(1,N);

        t1 = T_climb;
        t2 = t1 + T_level;
        t3 = t2 + T_square;
        t4 = t3 + T_approach;
        t5 = t4 + T_desc;

        x_start_sq = L_climb_h + L_level;

        % Build square path as parametric by distance along the perimeter
        % Segments: straight(+X) → corner(R) → straight(+Y) → corner(R)
        %         → straight(-X) → corner(R) → straight(-Y) → corner(R)
        seg_lens = [L_straight, L_corner_arc, L_straight, L_corner_arc, ...
                    L_straight, L_corner_arc, L_straight, L_corner_arc];
        cum_seg = [0, cumsum(seg_lens)];

        for k = 1:N
            tk = t(k);

            if tk < t1
                % Climb
                frac = tk / T_climb;
                xd(k) = frac * L_climb_h;
                yd(k) = 0;
                zd(k) = frac * alt_cruise;
                psid(k) = 0;

            elseif tk < t2
                % Level transition
                dt_seg = tk - t1;
                xd(k) = L_climb_h + V * dt_seg;
                yd(k) = 0;
                zd(k) = alt_cruise;
                psid(k) = 0;

            elseif tk < t3
                % Square patrol
                dt_seg = tk - t2;
                dist_sq = V * dt_seg;
                dist_sq = mod(dist_sq, L_square);

                [sx, sy, sh] = square_point(dist_sq, cum_seg, L_straight, R_corner);
                xd(k) = x_start_sq + sx;
                yd(k) = sy;
                zd(k) = alt_cruise;
                psid(k) = sh;

            elseif tk < t4
                % Approach back
                dt_seg = tk - t3;
                xd(k) = x_start_sq + V * dt_seg;
                yd(k) = 0;
                zd(k) = alt_cruise;
                psid(k) = 0;

            elseif tk < t5
                % Descent
                frac = (tk - t4) / T_desc;
                x_start_desc = x_start_sq + L_level;
                xd(k) = x_start_desc + frac * L_desc_h;
                yd(k) = 0;
                zd(k) = alt_cruise * (1 - frac);
                psid(k) = 0;

            else
                % Settle
                x_end_all = x_start_sq + L_level + L_desc_h;
                xd(k) = x_end_all;
                yd(k) = 0;
                zd(k) = 0;
                psid(k) = 0;
            end

            zd(k) = max(0, zd(k));
            psid(k) = atan2(sin(psid(k)), cos(psid(k)));
        end

        % Smooth heading
        psid = smooth_heading(psid, dt, N);

        fprintf('Trajectory: recon square patrol\n');
        fprintf('  Climb: gamma=%.0f°, alt=%.0fm\n', rad2deg(gamma_climb), alt_cruise);
        fprintf('  Square: %.0fm side, R_corner=%.0fm\n', side_len, R_corner);
        fprintf('  Descent: gamma=%.0f°\n', rad2deg(gamma_desc));
        fprintf('  Total: %.1fs, %d samples\n', t(end), N);

    case 'figure6_mission'
        %% ===== TAKEOFF → FIGÜR-6 (6 çizimi) → DESCENT → LAND =====
        % The "6" shape from above: 
        %   1. Climb to cruise altitude
        %   2. Full CCW circle (the loop of the "6")
        %   3. Straight forward flight (the tail of the "6")
        %   4. Descent and landing
        %
        % The circle is on the LEFT side, the tail extends to the RIGHT.
        % This naturally looks like the digit "6" when viewed from above.

        V = p.u0;

        if isfield(p, 'fig6')
            m = p.fig6;
        else
            m = struct();
        end
        gamma_climb = deg2rad(get_or(m, 'climb_angle_deg', 15));
        gamma_desc  = deg2rad(get_or(m, 'descend_angle_deg', 8));
        alt_cruise  = get_or(m, 'cruise_alt', 50);
        R_loop      = get_or(m, 'loop_radius', 80);

        % Climb segment
        L_climb_h = alt_cruise / tan(gamma_climb);
        L_climb_s = alt_cruise / sin(gamma_climb);
        T_climb   = L_climb_s / V;

        % Level transition before loop
        L_level = 15;
        T_level = L_level / V;

        % Full CCW circle (the loop of the "6")
        loop_arc = 2 * pi * R_loop;
        T_loop = loop_arc / V;
        omega_loop = V / R_loop;

        % Tail of the "6": straight forward in +X direction
        L_tail = 2.5 * R_loop;
        T_tail = L_tail / V;

        % Level transition after tail
        L_approach = 15;
        T_approach = L_approach / V;

        % Descent
        L_desc_h = alt_cruise / tan(gamma_desc);
        L_desc_s = alt_cruise / sin(gamma_desc);
        T_desc   = L_desc_s / V;

        T_settle = 3.0;
        T_total = T_climb + T_level + T_loop + T_tail + T_approach + T_desc + T_settle;
        N = round(T_total / dt);
        t = (0:N-1) * dt;
        xd = zeros(1,N); yd = zeros(1,N); zd = zeros(1,N); psid = zeros(1,N);

        % Cumulative time boundaries
        t1 = T_climb;
        t2 = t1 + T_level;
        t3 = t2 + T_loop;
        t4 = t3 + T_tail;
        t5 = t4 + T_approach;
        t6 = t5 + T_desc;

        % Position landmarks
        x_end_climb = L_climb_h;
        x_end_level = x_end_climb + L_level;

        % Loop center: aircraft starts at (x_end_level, 0) heading +X
        % Enter at bottom of circle (angle -pi/2), go CCW
        cx_loop = x_end_level;
        cy_loop = R_loop;

        for k = 1:N
            tk = t(k);

            if tk < t1
                % --- Climb ---
                frac = tk / T_climb;
                xd(k) = frac * L_climb_h;
                yd(k) = 0;
                zd(k) = frac * alt_cruise;
                psid(k) = 0;

            elseif tk < t2
                % --- Level transition ---
                dt_seg = tk - t1;
                xd(k) = x_end_climb + V * dt_seg;
                yd(k) = 0;
                zd(k) = alt_cruise;
                psid(k) = 0;

            elseif tk < t3
                % --- Full circle loop (CCW) — the loop of "6" ---
                dt_seg = tk - t2;
                ang = -pi/2 + omega_loop * dt_seg;
                xd(k) = cx_loop + R_loop * cos(ang);
                yd(k) = cy_loop + R_loop * sin(ang);
                zd(k) = alt_cruise;
                psid(k) = ang + pi/2;  % CCW tangent

            elseif tk < t4
                % --- Tail of "6" (straight forward +X) ---
                dt_seg = tk - t3;
                % After loop, aircraft is back at bottom (cx_loop, 0)
                xd(k) = cx_loop + V * dt_seg;
                yd(k) = 0;
                zd(k) = alt_cruise;
                psid(k) = 0;

            elseif tk < t5
                % --- Level approach ---
                dt_seg = tk - t4;
                xd(k) = cx_loop + L_tail + V * dt_seg;
                yd(k) = 0;
                zd(k) = alt_cruise;
                psid(k) = 0;

            elseif tk < t6
                % --- Descent ---
                frac = (tk - t5) / T_desc;
                x_start_desc = cx_loop + L_tail + L_approach;
                xd(k) = x_start_desc + frac * L_desc_h;
                yd(k) = 0;
                zd(k) = alt_cruise * (1 - frac);
                psid(k) = 0;

            else
                % --- Settle ---
                x_end_all = cx_loop + L_tail + L_approach + L_desc_h;
                xd(k) = x_end_all;
                yd(k) = 0;
                zd(k) = 0;
                psid(k) = 0;
            end

            zd(k) = max(0, zd(k));
            psid(k) = atan2(sin(psid(k)), cos(psid(k)));
        end

        % Smooth heading
        psid = smooth_heading(psid, dt, N);

        fprintf('Trajectory: figure-6 mission\n');
        fprintf('  Climb:  gamma=%.0f deg, alt=%.0fm\n', rad2deg(gamma_climb), alt_cruise);
        fprintf('  Loop:   R=%.0fm (full CCW circle)\n', R_loop);
        fprintf('  Tail:   %.0fm forward\n', L_tail);
        fprintf('  Desc:   gamma=%.0f deg\n', rad2deg(gamma_desc));
        fprintf('  Total:  %.1fs, %d samples\n', t(end), N);

    case 'sharp_square'
        %% ===== TAKEOFF → SHARP SQUARE PATROL (90° TURNS) → RTL LAND =====
        %  Tests controller response to sharp heading changes.
        %  Small corner_radius = sharper turns, more demanding on yaw control.

        V = p.u0;

        if isfield(p, 'square')
            m = p.square;
        else
            m = struct();
        end
        gamma_climb = deg2rad(get_or(m, 'climb_angle_deg', 15));
        gamma_desc  = deg2rad(get_or(m, 'descend_angle_deg', 10));
        alt_cruise  = get_or(m, 'cruise_alt', 30);
        side_len    = get_or(m, 'side_length', 100);
        R_corner    = get_or(m, 'corner_radius', 10);  % small = sharp
        n_laps      = get_or(m, 'num_laps', 2);

        % Climb
        L_climb_h = alt_cruise / tan(gamma_climb);
        L_climb_s = alt_cruise / sin(gamma_climb);
        T_climb   = L_climb_s / V;

        % Level transition
        L_level = 15;
        T_level = L_level / V;

        % Square perimeter: 4 straights + 4 quarter-circle corners
        L_straight = side_len - 2*R_corner;
        L_corner_arc = pi/2 * R_corner;
        L_square = 4 * L_straight + 4 * L_corner_arc;
        L_total_square = L_square * n_laps;
        T_square = L_total_square / V;

        % Return leg
        T_approach = L_level / V;

        % Descent
        L_desc_h = alt_cruise / tan(gamma_desc);
        L_desc_s = alt_cruise / sin(gamma_desc);
        T_desc   = L_desc_s / V;

        T_settle = 3.0;
        T_total = T_climb + T_level + T_square + T_approach + T_desc + T_settle;
        N = round(T_total / dt);
        t = (0:N-1) * dt;
        xd = zeros(1,N); yd = zeros(1,N); zd = zeros(1,N); psid = zeros(1,N);

        t1 = T_climb;
        t2 = t1 + T_level;
        t3 = t2 + T_square;
        t4 = t3 + T_approach;
        t5 = t4 + T_desc;

        x_start_sq = L_climb_h + L_level;

        % Build square path segments
        seg_lens = [L_straight, L_corner_arc, L_straight, L_corner_arc, ...
                    L_straight, L_corner_arc, L_straight, L_corner_arc];
        cum_seg = [0, cumsum(seg_lens)];

        for k = 1:N
            tk = t(k);

            if tk < t1
                % Climb
                frac = tk / T_climb;
                xd(k) = frac * L_climb_h;
                yd(k) = 0;
                zd(k) = frac * alt_cruise;
                psid(k) = 0;

            elseif tk < t2
                % Level transition
                dt_seg = tk - t1;
                xd(k) = L_climb_h + V * dt_seg;
                yd(k) = 0;
                zd(k) = alt_cruise;
                psid(k) = 0;

            elseif tk < t3
                % Square patrol (may span multiple laps)
                dt_seg = tk - t2;
                dist_sq = V * dt_seg;
                lap_dist = mod(dist_sq, L_square);

                [sx, sy, sh] = square_point(lap_dist, cum_seg, L_straight, R_corner);
                xd(k) = x_start_sq + sx;
                yd(k) = sy;
                zd(k) = alt_cruise;
                psid(k) = sh;

            elseif tk < t4
                % Approach back
                dt_seg = tk - t3;
                xd(k) = x_start_sq + V * dt_seg;
                yd(k) = 0;
                zd(k) = alt_cruise;
                psid(k) = 0;

            elseif tk < t5
                % Descent
                frac = (tk - t4) / T_desc;
                x_start_desc = x_start_sq + L_level;
                xd(k) = x_start_desc + frac * L_desc_h;
                yd(k) = 0;
                zd(k) = alt_cruise * (1 - frac);
                psid(k) = 0;

            else
                % Settle
                x_end_all = x_start_sq + L_level + L_desc_h;
                xd(k) = x_end_all;
                yd(k) = 0;
                zd(k) = 0;
                psid(k) = 0;
            end

            zd(k) = max(0, zd(k));
            psid(k) = atan2(sin(psid(k)), cos(psid(k)));
        end

        % Smooth heading (less aggressive for sharp turns)
        psid = smooth_heading(psid, dt, N);

        fprintf('Trajectory: sharp square patrol\n');
        fprintf('  Climb: gamma=%.0f°, alt=%.0fm\n', rad2deg(gamma_climb), alt_cruise);
        fprintf('  Square: %.0fm side, corner R=%.0fm (%.0f° turns), %d laps\n', side_len, R_corner, 90, n_laps);
        fprintf('  Desc: gamma=%.0f°\n', rad2deg(gamma_desc));
        fprintf('  Total: %.1fs, %d samples\n', t(end), N);

    otherwise
        error('Unknown trajectory type: %s', p.traj_type);
end

end

%% ======================================================================
function val = get_or(s, field, default)
%GET_OR  Return struct field if it exists, else return default.
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end

function psid = smooth_heading(psid, dt, N)
%SMOOTH_HEADING  Low-pass filter heading array with angle wrapping.
    smooth_win = max(1, round(0.5 / dt));
    psid_s = psid;
    for k = 2:N
        d = atan2(sin(psid(k) - psid_s(k-1)), cos(psid(k) - psid_s(k-1)));
        a = min(1.0, dt * smooth_win * 2 / 0.5);
        psid_s(k) = psid_s(k-1) + a * d;
        psid_s(k) = atan2(sin(psid_s(k)), cos(psid_s(k)));
    end
    psid = psid_s;
end

function [sx, sy, sh] = square_point(dist, cum_seg, L_str, R_c)
%SQUARE_POINT  Compute (x,y,heading) on a rounded-corner square by arc distance.
%  Segments: str(+X) → corner → str(+Y) → corner → str(-X) → corner → str(-Y) → corner

    % Headings at each side: 0, pi/2, pi, -pi/2
    hdg_side = [0, pi/2, pi, -pi/2];
    % Corner center offsets from end of straight segment
    % Corner i turns from hdg_side(i) to hdg_side(i+1)
    % Starting corner centers (relative to square origin)
    cx = [L_str,          L_str,          0,      0];
    cy = [0,              L_str,          L_str,  0];

    seg = find(cum_seg(2:end) >= dist - 1e-9, 1);
    if isempty(seg); seg = 8; end
    d_in_seg = dist - cum_seg(seg);

    side_idx = ceil(seg / 2);   % which side (1-4)
    is_corner = mod(seg, 2) == 0;

    if ~is_corner
        % Straight segment
        h = hdg_side(side_idx);
        frac = d_in_seg / max(L_str, 1e-6);
        switch side_idx
            case 1; sx = frac * L_str;      sy = 0;
            case 2; sx = L_str;              sy = frac * L_str;
            case 3; sx = L_str - frac*L_str; sy = L_str;
            case 4; sx = 0;                  sy = L_str - frac*L_str;
        end
        sh = h;
    else
        % Rounded corner (quarter-circle)
        ang_in = d_in_seg / R_c;   % angle swept so far [rad]
        h_start = hdg_side(side_idx);
        ccx = cx(side_idx);
        ccy = cy(side_idx);

        % Circle center is offset inward by R_c from the corner vertex
        switch side_idx
            case 1  % corner at (L_str, 0), turning from hdg=0 to hdg=pi/2
                ocx = ccx;           ocy = ccy + R_c;
                ang0 = -pi/2;
            case 2  % corner at (L_str, L_str), turning from pi/2 to pi
                ocx = ccx - R_c;    ocy = ccy;
                ang0 = 0;
            case 3  % corner at (0, L_str), turning from pi to -pi/2
                ocx = ccx;           ocy = ccy - R_c;
                ang0 = pi/2;
            case 4  % corner at (0, 0), turning from -pi/2 to 0
                ocx = ccx + R_c;    ocy = ccy;
                ang0 = pi;
        end

        sx = ocx + R_c * cos(ang0 + ang_in);
        sy = ocy + R_c * sin(ang0 + ang_in);
        sh = h_start + ang_in;
        sh = atan2(sin(sh), cos(sh));
    end
end

