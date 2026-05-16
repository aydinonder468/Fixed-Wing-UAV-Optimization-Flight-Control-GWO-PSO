%% main_fw.m — Fixed-Wing UAV + BMS Simulation
clear; clc; close all;

%% Add subfolders to MATLAB path
addpath('fixedwing', 'bms', 'ui');

%% ======================================================================
%  STEP 1: LOAD PARAMETERS
% ======================================================================
fw_params;
bms_params;

% Build manual gain vector from fw_params defaults (7 PID × 3 gains = 21)
p.gain_sets.manual = [p.pid_alt.Kp,        p.pid_alt.Ki,        p.pid_alt.Kd, ...
                       p.pid_speed.Kp,      p.pid_speed.Ki,      p.pid_speed.Kd, ...
                       p.pid_hdg.Kp,        p.pid_hdg.Ki,        p.pid_hdg.Kd, ...
                       p.pid_pitch_att.Kp,  p.pid_pitch_att.Ki,  p.pid_pitch_att.Kd, ...
                       p.pid_pitch_rate.Kp, p.pid_pitch_rate.Ki, p.pid_pitch_rate.Kd, ...
                       p.pid_roll_att.Kp,   p.pid_roll_att.Ki,   p.pid_roll_att.Kd, ...
                       p.pid_roll_rate.Kp,  p.pid_roll_rate.Ki,  p.pid_roll_rate.Kd];

% Auto-load previously optimized gains if available
p.gain_sets.gwo = load_cached_gains('gwo_best_gains.mat', 'gwo_gain_vector');
p.gain_sets.pso = load_cached_gains('pso_best_gains.mat', 'pso_gain_vector');

fprintf('\n');
fprintf('========================================\n');
fprintf('  FIXED-WING UAV SIMULATION CONTROLLER\n');
fprintf('========================================\n\n');

%% ======================================================================
%  STEP 2: CONTROL TYPE SELECTION
% ======================================================================
control_choice = select_control_type();

if isempty(control_choice)
    fprintf('No control type selected. Exiting.\n');
    return;
end

switch control_choice
    case 1
        % ===== MANUEL PID =====
        fprintf('\n>> Control Type: MANUEL PID\n');
        p.active_control = 'manual';
        p = apply_pid_gain_vector(p, p.gain_sets.manual);

        % Ask user if they want to adjust gains manually
        adjust = input('Manuel PID kazançlarını düzenlemek ister misiniz? (e/h): ', 's');
        if lower(adjust) == 'e'
            p = manual_gain_input(p);
            % Update manual gain set with user values
            p.gain_sets.manual = build_gain_vector(p);
            fprintf('[+] Updated manual gains applied.\n');
        end

        % Save current gains
        save('manual_gains.mat', 'p');
        fprintf('[+] Manual gains saved to manual_gains.mat\n');

    case 2
        % ===== GWO — LOAD CACHED GAINS =====
        fprintf('\n>> Control Type: GWO (Grey Wolf Optimizer)\n');
        if isempty(p.gain_sets.gwo)
            fprintf('[!] No cached GWO gains found (gwo_best_gains.mat missing).\n');
            fprintf('    Run GWO_autotune_pid once to generate optimized gains.\n');
            fprintf('\n  Optimizasyonu şimdi başlatmak ister misiniz? (e/h): ');
            run_opt = input('', 's');
            if lower(run_opt) == 'e'
                GWO_autotune_pid;
                return;
            else
                fprintf('Exiting.\n');
                return;
            end
        end
        p.active_control = 'gwo';
        p = apply_pid_gain_vector(p, p.gain_sets.gwo);
        fprintf('[+] Using cached GWO gains — no re-optimization needed.\n');

    case 3
        % ===== PSO — LOAD CACHED GAINS =====
        fprintf('\n>> Control Type: PSO (Particle Swarm Optimization)\n');
        if isempty(p.gain_sets.pso)
            fprintf('[!] No cached PSO gains found (pso_best_gains.mat missing).\n');
            fprintf('    Run PSO_autotune_pid once to generate optimized gains.\n');
            fprintf('\n  Optimizasyonu şimdi başlatmak ister misiniz? (e/h): ');
            run_opt = input('', 's');
            if lower(run_opt) == 'e'
                PSO_autotune_pid;
                return;
            else
                fprintf('Exiting.\n');
                return;
            end
        end
        p.active_control = 'pso';
        p = apply_pid_gain_vector(p, p.gain_sets.pso);
        fprintf('[+] Using cached PSO gains — no re-optimization needed.\n');
end

%% ======================================================================
%  STEP 3: ROUTE SELECTION
% ======================================================================
route_choice = select_flight_route();
if isempty(route_choice)
    fprintf('No route selected. Exiting.\n');
    return;
end

p = apply_route(p, route_choice);
fprintf('\n>> Route %d selected: %s\n\n', route_choice, route_name(route_choice));

%% ======================================================================
%  STEP 4: TRAJECTORY GENERATION
% ======================================================================
[t, xd, yd, zd, psid] = trajectory_fw(p);
fprintf('Trajectory: %s, %.1f s, %d samples\n', p.traj_type, t(end), length(t));

%% ======================================================================
%  STEP 5: SIMULATION (coupled fixed-wing + BMS)
% ======================================================================
tic;
[rec, bms] = simulate_fw(p, t, xd, yd, zd, psid);
fprintf('Simulation complete in %.2f s.\n', toc);

%% ======================================================================
%  STEP 6: MISSION FEASIBILITY ANALYSIS
% ======================================================================
mfeas = mission_feasibility(p, rec, bms, t);

%% ======================================================================
%  STEP 7: UNIFIED DASHBOARD + POST-ANALYSIS PLOTS
% ======================================================================
fw_monitor(t, rec, bms, p, mfeas, xd, yd, zd, psid);


%% ======================================================================
%  HELPER FUNCTIONS
% ======================================================================

function gains = load_cached_gains(filename, field_name)
%LOAD_CACHED_GAINS  Load optimized gains from .mat file if available.
    gains = [];
    if isfile(filename)
        try
            data = load(filename, field_name);
            if isfield(data, field_name) && numel(data.(field_name)) == 21
                gains = data.(field_name);
                fprintf('[+] %s gains loaded from %s\n', upper(field_name), filename);
            else
                fprintf('[!] %s has unexpected format.\n', filename);
            end
        catch ME
            fprintf('[!] Failed to load %s (%s)\n', filename, ME.message);
        end
    else
        fprintf('[i] No %s found (will use manual gains)\n', filename);
    end
end

function vec = build_gain_vector(p)
%BUILD_GAIN_VECTOR  Extract 21-element gain vector from parameter struct.
    vec = [p.pid_alt.Kp,        p.pid_alt.Ki,        p.pid_alt.Kd, ...
           p.pid_speed.Kp,      p.pid_speed.Ki,      p.pid_speed.Kd, ...
           p.pid_hdg.Kp,        p.pid_hdg.Ki,        p.pid_hdg.Kd, ...
           p.pid_pitch_att.Kp,  p.pid_pitch_att.Ki,  p.pid_pitch_att.Kd, ...
           p.pid_pitch_rate.Kp, p.pid_pitch_rate.Ki, p.pid_pitch_rate.Kd, ...
           p.pid_roll_att.Kp,   p.pid_roll_att.Ki,   p.pid_roll_att.Kd, ...
           p.pid_roll_rate.Kp,  p.pid_roll_rate.Ki,  p.pid_roll_rate.Kd];
end

function p = manual_gain_input(p)
%MANUAL_GAIN_INPUT  Interactive CLI for tuning individual PID gains.
    pid_names = {'pid_alt', 'pid_speed', 'pid_hdg', ...
                 'pid_pitch_att', 'pid_pitch_rate', ...
                 'pid_roll_att', 'pid_roll_rate'};
    pid_labels = {'Altitude  (alt err → theta_cmd)', ...
                  'Speed     (speed err → delta_t)', ...
                  'Heading   (heading err → phi_cmd)', ...
                  'Pitch Att (theta err → q_ref)', ...
                  'Pitch Rate (q err → delta_e)', ...
                  'Roll Att  (phi err → p_ref)', ...
                  'Roll Rate (p err → delta_a)'};

    fprintf('\n--- Manuel PID Kazanç Ayarlama ---\n');
    fprintf('Mevcut değerler parantez içinde gösterilmektedir.\n');
    fprintf('Değiştirmek istemiyorsanız Enter tuşuna basın.\n\n');

    for j = 1:length(pid_names)
        key = pid_names{j};
        fprintf('  [%s] %s\n', j, pid_labels{j});
        current = p.(key);

        kp_str = input(sprintf('    Kp [%.4f]: ', current.Kp), 's');
        if ~isempty(kp_str), p.(key).Kp = str2double(kp_str); end

        ki_str = input(sprintf('    Ki [%.4f]: ', current.Ki), 's');
        if ~isempty(ki_str), p.(key).Ki = str2double(ki_str); end

        kd_str = input(sprintf('    Kd [%.4f]: ', current.Kd), 's');
        if ~isempty(kd_str), p.(key).Kd = str2double(kd_str); end

        fprintf('\n');
    end
end

function choice = select_control_type()
%SELECT_CONTROL_TYPE  Modal dialog with 3 control type options.
%  Returns 1 (Manuel), 2 (GWO), or 3 (PSO), or [] if closed.

    choice = [];

    fig = figure('Name','Control Type Selection','NumberTitle','off',...
        'MenuBar','none','ToolBar','none','Resize','off',...
        'Units','pixels','Position',[400 300 580 380],...
        'Color',[0.15 0.17 0.22],'CloseRequestFcn',@onClose);
    movegui(fig,'center');

    % Title
    uicontrol(fig,'Style','text','String','SELECT CONTROL TYPE',...
        'Units','pixels','Position',[30 340 520 30],...
        'FontSize',18,'FontWeight','bold','ForegroundColor',[0.9 0.95 1],...
        'BackgroundColor',[0.15 0.17 0.22],'HorizontalAlignment','center');

    uicontrol(fig,'Style','text',...
        'String','Choose how PID gains are determined for this simulation.',...
        'Units','pixels','Position',[30 320 520 20],...
        'FontSize',9,'ForegroundColor',[0.6 0.65 0.7],...
        'BackgroundColor',[0.15 0.17 0.22],'HorizontalAlignment','center');

    % Option definitions
    options = {
        '1  Manuel PID', ...
        'Onceden tanimli manuel PID kazanc lari ile simülasyon. | ', ...
        'Kullanici isterse CLI üzerinden tek tek Kp/Ki/Kd girebilir.';

        '2  GWO Optimizasyonu', ...
        'Kaydedilmis GWO kazançlari ile hizli simülasyon. | ', ...
        'gwo_best_gains.mat yoksa GWO_autotune_pid.m bir kez çalistirilir.';

        '3  PSO Optimizasyonu', ...
        'Kaydedilmis PSO kazançlari ile hizli simülasyon. | ', ...
        'pso_best_gains.mat yoksa PSO_autotune_pid.m bir kez çalistirilir.';
    };

    colors = {[0.20 0.55 0.35], [0.55 0.20 0.35], [0.18 0.40 0.65]};
    hover  = {[0.25 0.70 0.45], [0.70 0.25 0.45], [0.22 0.52 0.82]};

    y0 = 280;
    card_h = 72;
    gap = 8;

    for i = 1:3
        yp = y0 - (i-1)*(card_h + gap);

        % Card background
        uicontrol(fig,'Style','text','String','',...
            'Units','pixels','Position',[18 yp 544 card_h],...
            'BackgroundColor',[0.20 0.22 0.28]);

        % Option number badge
        uicontrol(fig,'Style','text','String',sprintf('%d',i),...
            'Units','pixels','Position',[28 yp+20 30 30],...
            'FontSize',16,'FontWeight','bold',...
            'ForegroundColor','w','BackgroundColor',colors{i},...
            'HorizontalAlignment','center');

        % Option name
        uicontrol(fig,'Style','text','String',options{i,1},...
            'Units','pixels','Position',[68 yp+card_h-28 320 22],...
            'FontSize',11,'FontWeight','bold',...
            'ForegroundColor',[0.95 0.95 0.95],...
            'BackgroundColor',[0.20 0.22 0.28],...
            'HorizontalAlignment','left');

        % Description
        uicontrol(fig,'Style','text','String',[options{i,2}, options{i,3}],...
            'Units','pixels','Position',[68 yp+4 420 card_h-32],...
            'FontSize',8,'ForegroundColor',[0.65 0.68 0.72],...
            'BackgroundColor',[0.20 0.22 0.28],...
            'HorizontalAlignment','left');

        % Select button
        btn = uicontrol(fig,'Style','pushbutton',...
            'String','SELECT',...
            'Units','pixels','Position',[460 yp+18 85 36],...
            'FontSize',10,'FontWeight','bold',...
            'ForegroundColor','w','BackgroundColor',colors{i},...
            'UserData',i,'Callback',@onSelect);

        % Hover effect
        set(btn,'ButtonDownFcn',@(s,e) set(s,'BackgroundColor',hover{i}));
    end

    % Footer
    uicontrol(fig,'Style','text',...
        'String','GWO/PSO seçenekleri kaydedilmis kazançlari yükler. Dosya yoksa optimizasyon sorulur.',...
        'Units','pixels','Position',[30 8 520 18],...
        'FontSize',8,'ForegroundColor',[0.45 0.48 0.52],...
        'BackgroundColor',[0.15 0.17 0.22],'HorizontalAlignment','center');

    % Wait for selection
    uiwait(fig);

    function onSelect(src, ~)
        choice = src.UserData;
        uiresume(fig);
        delete(fig);
    end

    function onClose(src, ~)
        choice = [];
        uiresume(src);
        delete(src);
    end
end

function choice = select_flight_route()
%SELECT_FLIGHT_ROUTE  Modal dialog with route options.
%  Returns route ID or [] if closed.

    choice = [];

    fig = figure('Name','Flight Route Selection','NumberTitle','off',...
        'MenuBar','none','ToolBar','none','Resize','off',...
        'Units','pixels','Position',[400 200 620 520],...
        'Color',[0.15 0.17 0.22],'CloseRequestFcn',@onClose);
    movegui(fig,'center');

    % Title
    uicontrol(fig,'Style','text','String','SELECT FLIGHT ROUTE',...
        'Units','pixels','Position',[30 470 560 35],...
        'FontSize',18,'FontWeight','bold','ForegroundColor',[0.9 0.95 1],...
        'BackgroundColor',[0.15 0.17 0.22],'HorizontalAlignment','center');

    uicontrol(fig,'Style','text',...
        'String','Choose a mission profile and press the button to launch simulation.',...
        'Units','pixels','Position',[30 448 560 22],...
        'FontSize',9,'ForegroundColor',[0.6 0.65 0.7],...
        'BackgroundColor',[0.15 0.17 0.22],'HorizontalAlignment','center');

    % Route definitions
    routes = {
        'Mission (Standart)',...
        ['Takeoff (15°) -> Seviye -> Loiter (R=96m, 2 tur) -> Approach -> Landing | ',...
         'Klasik gorev profili, arttirilmis donus yaricsapi.'];

        'Figur-6 Mission',...
        ['Takeoff (15°) -> 50m irtifa -> Genis acili 6 cizimi (R=120m) -> Yumusak inis | ',...
         'Havada buyuk 6 cizen kesif rotasi.'];

        'Keskin Kare (Sharp Square)',...
        ['Takeoff -> 100m kare patrol (90° keskin donusler, R=10m) -> RTL inis | ',...
         'Kontrolcunun keskin manevralara tepkisini test eder.'];
    };

    colors = {[0.20 0.55 0.35], [0.18 0.40 0.65], [0.55 0.30 0.20]};
    hover  = {[0.25 0.70 0.45], [0.22 0.52 0.82], [0.70 0.40 0.25]};

    y0 = 400;
    card_h = 72;
    gap = 8;

    for i = 1:3
        yp = y0 - (i-1)*(card_h + gap);

        % Card background
        uicontrol(fig,'Style','text','String','',...
            'Units','pixels','Position',[18 yp 584 card_h],...
            'BackgroundColor',[0.20 0.22 0.28]);

        % Route number badge
        uicontrol(fig,'Style','text','String',sprintf('%d',i),...
            'Units','pixels','Position',[28 yp+20 30 30],...
            'FontSize',16,'FontWeight','bold',...
            'ForegroundColor','w','BackgroundColor',colors{i},...
            'HorizontalAlignment','center');

        % Route name
        uicontrol(fig,'Style','text','String',routes{i,1},...
            'Units','pixels','Position',[68 yp+card_h-28 320 22],...
            'FontSize',11,'FontWeight','bold',...
            'ForegroundColor',[0.95 0.95 0.95],...
            'BackgroundColor',[0.20 0.22 0.28],...
            'HorizontalAlignment','left');

        % Description
        uicontrol(fig,'Style','text','String',routes{i,2},...
            'Units','pixels','Position',[68 yp+4 420 card_h-32],...
            'FontSize',8,'ForegroundColor',[0.65 0.68 0.72],...
            'BackgroundColor',[0.20 0.22 0.28],...
            'HorizontalAlignment','left');

        % Launch button
        btn = uicontrol(fig,'Style','pushbutton',...
            'String','LAUNCH',...
            'Units','pixels','Position',[505 yp+18 85 36],...
            'FontSize',10,'FontWeight','bold',...
            'ForegroundColor','w','BackgroundColor',colors{i},...
            'UserData',i,'Callback',@onSelect);

        % Hover effect
        set(btn,'ButtonDownFcn',@(s,e) set(s,'BackgroundColor',hover{i}));
    end

    % Footer
    uicontrol(fig,'Style','text',...
        'String','Press LAUNCH or close window to cancel.',...
        'Units','pixels','Position',[30 8 560 18],...
        'FontSize',8,'ForegroundColor',[0.45 0.48 0.52],...
        'BackgroundColor',[0.15 0.17 0.22],'HorizontalAlignment','center');

    % Wait for selection
    uiwait(fig);

    function onSelect(src, ~)
        choice = src.UserData;
        uiresume(fig);
        delete(fig);
    end

    function onClose(src, ~)
        choice = [];
        uiresume(src);
        delete(src);
    end
end

function p = apply_route(p, route_id)
%APPLY_ROUTE  Configure trajectory parameters for the selected route.
    switch route_id
        case 1
            p.traj_type = 'mission';

        case 2
            p.traj_type = 'figure6_mission';
            p.fig6.climb_angle_deg   = 15;
            p.fig6.descend_angle_deg = 8;
            p.fig6.cruise_alt        = 50;
            p.fig6.loop_radius       = 80;

        case 3
            p.traj_type = 'sharp_square';
            p.square.climb_angle_deg = 15;
            p.square.descend_angle_deg = 10;
            p.square.cruise_alt    = 30;
            p.square.side_length   = 100;
            p.square.corner_radius = 10;   % sharp 90° turns
            p.square.num_laps      = 2;
    end
end

function name = route_name(route_id)
%ROUTE_NAME  Return display name for route ID.
    names = {'Mission (Standart)', 'Figur-6 Mission', 'Keskin Kare (Sharp Square)'};
    name = names{route_id};
end
