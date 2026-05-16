function mesh = load_aircraft_mesh(stl_path, scale, rot_offset, pos_offset)
%LOAD_AIRCRAFT_MESH  Load an STL mesh or generate a built-in fixed-wing shape.
%  Returns a struct with fields: vertices (Nx3), faces (Mx3), loaded (bool).
%
%  INPUTS:
%    stl_path   — path to .stl file (string)
%    scale      — scalar or [sx sy sz]  (default: 1)
%    rot_offset — [roll pitch yaw] in DEGREES to correct mesh orientation
%    pos_offset — [dx dy dz] centroid shift in body frame
%
%  If the STL file is missing or fails to load, a built-in procedural
%  fixed-wing mesh is generated so the dashboard never crashes.
%
%  Body frame convention: x-forward, y-right, z-down.

    if nargin < 2 || isempty(scale);      scale = 1;          end
    if nargin < 3 || isempty(rot_offset);  rot_offset = [0 0 0]; end
    if nargin < 4 || isempty(pos_offset);  pos_offset = [0 0 0]; end

    mesh = struct('vertices',[],'faces',[],'loaded',false,'source','');

    % --- Try loading STL file ---
    if nargin >= 1 && ~isempty(stl_path) && isfile(stl_path)
        try
            tr = stlread(stl_path);
            mesh.vertices = tr.Points;
            mesh.faces    = tr.ConnectivityList;
            mesh.loaded   = true;
            mesh.source   = stl_path;
            fprintf('[mesh] Loaded STL: %s  (%d verts, %d faces)\n', ...
                stl_path, size(mesh.vertices,1), size(mesh.faces,1));
        catch ME
            fprintf('[mesh] Failed to read %s: %s\n', stl_path, ME.message);
            fprintf('[mesh] Falling back to built-in mesh.\n');
        end
    else
        if nargin >= 1 && ~isempty(stl_path)
            fprintf('[mesh] STL not found: %s\n', stl_path);
        end
        fprintf('[mesh] Using built-in procedural fixed-wing mesh.\n');
    end

    % --- Generate built-in mesh if STL failed ---
    if ~mesh.loaded
        [mesh.vertices, mesh.faces] = generate_builtin_mesh();
        mesh.source = 'built-in';
    end

    % --- Apply scale ---
    if isscalar(scale)
        mesh.vertices = mesh.vertices * scale;
    else
        mesh.vertices = mesh.vertices .* scale(:)';
    end

    % --- Apply rotation offset (deg → rad) ---
    if any(rot_offset ~= 0)
        R = euler_rot(deg2rad(rot_offset(1)), ...
                       deg2rad(rot_offset(2)), ...
                       deg2rad(rot_offset(3)));
        mesh.vertices = (R * mesh.vertices')';
    end

    % --- Center mesh at origin + apply position offset ---
    c = mean(mesh.vertices, 1);
    mesh.vertices = mesh.vertices - c + pos_offset(:)';
end

%% =====================================================================
function R = euler_rot(phi, theta, psi)
%EULER_ROT  ZYX rotation matrix (same convention as simulation).
    cp=cos(phi); sp=sin(phi);
    ct=cos(theta); st=sin(theta);
    cs=cos(psi); ss=sin(psi);
    R = [ct*cs, sp*st*cs-cp*ss, cp*st*cs+sp*ss;
         ct*ss, sp*st*ss+cp*cs, cp*st*ss-sp*cs;
         -st,   sp*ct,          cp*ct];
end

%% =====================================================================
function [V, F] = generate_builtin_mesh()
%GENERATE_BUILTIN_MESH  Procedural low-poly fixed-wing aircraft.
%  Body frame: x-forward, y-right, z-down.  Unit scale (~1 m span).

    % --- Fuselage (tapered cylinder approximation) ---
    %   8-sided cross-section, extruded from nose to tail
    n_sides = 8;
    ang = linspace(0, 2*pi, n_sides+1); ang(end) = [];

    % Cross-section stations along x-axis
    stations_x = [0.50, 0.35, 0.15, -0.05, -0.20, -0.35, -0.50];
    radii      = [0.00, 0.04, 0.055, 0.055, 0.045, 0.030, 0.01];

    V_fus = [];
    for s = 1:length(stations_x)
        for a = 1:n_sides
            V_fus(end+1,:) = [stations_x(s), ...
                              radii(s)*cos(ang(a)), ...
                              radii(s)*sin(ang(a))]; %#ok<AGROW>
        end
    end

    F_fus = [];
    for s = 1:length(stations_x)-1
        for a = 1:n_sides
            a2 = mod(a, n_sides) + 1;
            i1 = (s-1)*n_sides + a;
            i2 = (s-1)*n_sides + a2;
            i3 = s*n_sides + a2;
            i4 = s*n_sides + a;
            F_fus(end+1,:) = [i1 i2 i3]; %#ok<AGROW>
            F_fus(end+1,:) = [i1 i3 i4]; %#ok<AGROW>
        end
    end

    nv = size(V_fus, 1);

    % --- Main wings ---
    %   Tapered wing planform, slight dihedral
    wing_root_x  =  0.05;   % leading edge at root
    wing_tip_x   = -0.02;   % leading edge at tip
    chord_root   =  0.12;
    chord_tip    =  0.06;
    wing_span    =  0.55;   % half-span
    dihedral_z   = -0.02;   % tip rises slightly
    thickness    =  0.012;

    % Right wing (y > 0)
    wr = [wing_root_x,  0,          -thickness/2;  % 1 root LE top
          wing_root_x,  0,           thickness/2;  % 2 root LE bot
          wing_root_x - chord_root, 0, thickness/2; % 3 root TE bot
          wing_root_x - chord_root, 0,-thickness/2; % 4 root TE top
          wing_tip_x, wing_span, dihedral_z-thickness/2; % 5 tip LE top
          wing_tip_x, wing_span, dihedral_z+thickness/2; % 6 tip LE bot
          wing_tip_x - chord_tip, wing_span, dihedral_z+thickness/2; % 7 tip TE bot
          wing_tip_x - chord_tip, wing_span, dihedral_z-thickness/2]; % 8 tip TE top

    % Left wing: mirror y
    wl = wr;  wl(:,2) = -wl(:,2);

    % Faces for one wing panel (8 verts → 6 quads = 12 tris)
    wf = [1 2 6;  1 6 5;   % LE
          4 3 7;  4 7 8;   % TE
          1 4 8;  1 8 5;   % top
          2 3 7;  2 7 6;   % bottom
          1 2 3;  1 3 4;   % root cap
          5 6 7;  5 7 8];  % tip cap

    wr_off = nv;  V_wing = [wr; wl];
    F_wing = [wf + wr_off; wf + wr_off + 8];
    nv = nv + 16;

    % --- Horizontal tail ---
    ht_x = -0.42;   ht_chord = 0.08;  ht_span = 0.18;  ht_thick = 0.008;
    ht_r = [ht_x, 0, -ht_thick/2;
            ht_x, 0,  ht_thick/2;
            ht_x-ht_chord, 0, ht_thick/2;
            ht_x-ht_chord, 0,-ht_thick/2;
            ht_x, ht_span, -ht_thick/2;
            ht_x, ht_span,  ht_thick/2;
            ht_x-ht_chord, ht_span, ht_thick/2;
            ht_x-ht_chord, ht_span,-ht_thick/2];
    ht_l = ht_r;  ht_l(:,2) = -ht_l(:,2);

    F_ht = [wf + nv; wf + nv + 8];
    V_ht = [ht_r; ht_l];
    nv = nv + 16;

    % --- Vertical tail ---
    vt_x = -0.42;  vt_chord = 0.09;  vt_height = 0.12;  vt_thick = 0.008;
    V_vt = [vt_x, -vt_thick/2, 0;
            vt_x,  vt_thick/2, 0;
            vt_x-vt_chord,  vt_thick/2, 0;
            vt_x-vt_chord, -vt_thick/2, 0;
            vt_x, -vt_thick/2, -vt_height;
            vt_x,  vt_thick/2, -vt_height;
            vt_x-vt_chord*0.6, vt_thick/2, -vt_height;
            vt_x-vt_chord*0.6,-vt_thick/2, -vt_height];
    F_vt = wf + nv;
    nv = nv + 8; %#ok<NASGU>

    % --- Assemble ---
    V = [V_fus; V_wing; V_ht; V_vt];
    F = [F_fus; F_wing; F_ht; F_vt];
end
