function plant = create_plant_fw(p)
%CREATE_PLANT_FW  Build longitudinal + lateral state-space models & discretize.
%  plant = create_plant_fw(p)
%
%  Longitudinal: x_lon = [du; dw; dq; dtheta]
%                u_lon = [d_delta_t; d_delta_e]
%
%  Lateral:      x_lat = [dv; dp; dr; dphi]
%                u_lat = [d_delta_a; d_delta_r]

dt = p.dt;
g  = p.g;
u0 = p.u0;
ct0 = cos(p.theta0);
st0 = sin(p.theta0);
tt0 = tan(p.theta0);

%% Longitudinal A,B (continuous)
A_lon = [p.Xu,    p.Xw,    p.Xq,       -g*ct0;
         p.Zu,    p.Zw,    p.Zq + u0,  -g*st0;
         p.Mu,    p.Mw,    p.Mq,        0;
         0,       0,       1,           0     ];

B_lon = [p.X_dt,  p.X_de;
         p.Z_dt,  p.Z_de;
         p.M_dt,  p.M_de;
         0,       0      ];

%% Lateral A,B (continuous) — aileron only, no rudder (Bank-to-Turn)
A_lat = [p.Yv,    p.Yp,    p.Yr - u0,   g*ct0;
         p.Lv,    p.Lp,    p.Lr,        0;
         p.Nv,    p.Np,    p.Nr,        0;
         0,       1,       tt0,         0    ];

B_lat = [p.Y_da;
         p.L_da;
         p.N_da;
         0     ];

%% Store continuous
plant.A_lon = A_lon;  plant.B_lon = B_lon;
plant.A_lat = A_lat;  plant.B_lat = B_lat;
plant.C_lon = eye(4); plant.D_lon = zeros(4,2);
plant.C_lat = eye(4); plant.D_lat = zeros(4,1);

%% Display (suppress in quiet/PSO mode)
quiet = isfield(p, 'quiet') && p.quiet;
if ~quiet
    fprintf('\n===== Fixed-Wing Plant (Longitudinal) =====\n');
    fprintf('  A_lon = \n');  disp(A_lon);
    fprintf('  B_lon = \n');  disp(B_lon);
    fprintf('\n===== Fixed-Wing Plant (Lateral) =====\n');
    fprintf('  A_lat = \n');  disp(A_lat);
    fprintf('  B_lat = \n');  disp(B_lat);
    fprintf('Discretizing via c2d (ZOH, dt=%.4f s)...\n', dt);
end

%% Exact discretization (c2d, zero-order hold)

sys_lon = ss(A_lon, B_lon, eye(4), zeros(4,2));
sys_lat = ss(A_lat, B_lat, eye(4), zeros(4,1));

sys_lon_d = c2d(sys_lon, dt);
sys_lat_d = c2d(sys_lat, dt);

plant.Ad_lon = sys_lon_d.A;   plant.Bd_lon = sys_lon_d.B;
plant.Ad_lat = sys_lat_d.A;   plant.Bd_lat = sys_lat_d.B;

if ~quiet; fprintf('Discretization complete.\n'); end
end
