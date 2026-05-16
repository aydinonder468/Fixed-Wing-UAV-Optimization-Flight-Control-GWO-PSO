function fw_monitor(t, rec, bms, p, mfeas, xd, yd, zd, psid)
%FW_MONITOR  Unified real-time fixed-wing UAV dashboard — single window.
%  fw_monitor(t, rec, bms, p, mfeas, xd, yd, zd, psid)
%
%  Features:
%    - 3D animation with STL mesh / line fallback, chase camera
%    - Real-time signal plots (position, errors, attitude, controls)
%    - Battery / BMS monitoring
%    - Gain display table at top showing current Kp/Ki/Kd
%    - Control type selection (PID-GWO, PID-PSO, PID-Manual) at bottom
%    - Manual tuning dialog with editable gain table
%    - Re-simulation on control type change or manual tuning

N = length(t);
skip = max(1, round(0.02 / (t(2)-t(1))));
idx = 1:skip:N;  tf = t(idx);

%% ===== GAIN DATA PREPARATION =====
pid_names = {'Altitude','Speed','Heading','Pitch Att','Pitch Rate','Roll Att','Roll Rate'};
pid_keys  = {'pid_alt','pid_speed','pid_hdg','pid_pitch_att','pid_pitch_rate','pid_roll_att','pid_roll_rate'};
nPID = 7;

%% ===== AIRCRAFT SCALING =====
traj_span    = max([max(rec.x)-min(rec.x), max(rec.y)-min(rec.y), 10]);
vis_size     = traj_span * 0.08;
builtin_span = 1.1;
mesh_scale   = vis_size / builtin_span;
L_fus_phys  = vis_size * 0.45;
L_wing_phys = vis_size / 2;
L_tail_phys = vis_size * (p.cbar / p.b) * 2.5;

%% ===== MESH CONFIGURATION =====
mesh_rot_offset = [0, 0, 0];
mesh_pos_offset = [0, 0, 0];
stl_path = fullfile(fileparts(mfilename('fullpath')), 'assets', 'fixedwing.stl');
aircraft_mesh = load_aircraft_mesh(stl_path, mesh_scale, mesh_rot_offset, mesh_pos_offset);
use_mesh = ~isempty(aircraft_mesh.vertices);
if use_mesh
    aircraft_mesh.vertices(:,3) = -aircraft_mesh.vertices(:,3);
end

%% ===== CAMERA CONFIGURATION =====
cam_follow   = true;
cam_dist     = 4;
cam_height   = 2;
cam_smooth   = 0.15;
cam_zoom     = 1.0;
cam_pos_cur  = [0 0 0];
cam_init     = false;
if cam_follow
    fprintf('[cam] Chase camera: dist=%.0fm, height=%.0fm, smooth=%.2f\n', cam_dist, cam_height, cam_smooth);
else
    fprintf('[cam] Static offset camera active.\n');
end
fprintf('[viz] Display frame: x-forward, y-right, z-UP (NED rotation corrected).\n');

%% ===== FIGURE CREATION =====
fig = figure('Color','w','Name','Fixed-Wing Monitor','NumberTitle','off',...
    'Units','normalized','OuterPosition',[0 0 1 1],'CloseRequestFcn',@closeCB);
pause(0.1);
fig.UserData = struct('go',false,'speed',1.0,'stop',false,'resim_needed',false,'active_control',p.active_control);

%% ===== TOP GAIN DISPLAY TABLE =====
gain_data = cell(nPID, 4);
for j = 1:nPID
    pid_s = p.(pid_keys{j});
    gain_data{j,1} = pid_names{j};
    gain_data{j,2} = sprintf('%.4f', pid_s.Kp);
    gain_data{j,3} = sprintf('%.4f', pid_s.Ki);
    gain_data{j,4} = sprintf('%.4f', pid_s.Kd);
end
utt = uitable(fig,'Units','normalized','Position',[0.005 0.935 0.99 0.058],...
    'ColumnName',{'PID Loop','Kp','Ki','Kd'},...
    'ColumnWidth',{90,70,70,70},...
    'ColumnEditable',false,...
    'Data',gain_data,...
    'FontName','FixedWidth','FontSize',8);

% Control type label next to table
ctrl_type_str = upper(p.active_control);
if strcmp(p.active_control,'gwo'); ctrl_type_str = 'GWO'; end
if strcmp(p.active_control,'pso'); ctrl_type_str = 'PSO'; end
if strcmp(p.active_control,'manual'); ctrl_type_str = 'MANUAL'; end
gain_title = annotation(fig,'textbox',[0.42 0.993 0.20 0.018],...
    'String',sprintf('Active Gains  [ %s ]',ctrl_type_str),...
    'FontSize',9,'FontWeight','bold','EdgeColor','none',...
    'HorizontalAlignment','center','Color',[0.2 0.4 0.7]);

%% ===== LAYOUT — ADJUSTED POSITIONS =====
ry = [0.76, 0.585, 0.41, 0.155];  ph = 0.155;
ax3d  = axes('Parent',fig,'Position',[0.03 0.12 0.43 0.80]);
ax_p  = axes('Parent',fig,'Position',[0.51 ry(1) 0.20 ph]);
ax_e  = axes('Parent',fig,'Position',[0.51 ry(2) 0.20 ph]);
ax_a  = axes('Parent',fig,'Position',[0.51 ry(3) 0.20 ph]);
ax_c  = axes('Parent',fig,'Position',[0.51 ry(4) 0.20 ph]);
ax_s  = axes('Parent',fig,'Position',[0.76 ry(1) 0.20 ph]);
ax_vi = axes('Parent',fig,'Position',[0.76 ry(2) 0.20 ph]);
ax_pw = axes('Parent',fig,'Position',[0.76 ry(3) 0.20 ph]);

%% ===== 3D AXES =====
hold(ax3d,'on'); grid(ax3d,'on'); box(ax3d,'on'); axis(ax3d,'equal');
xlabel(ax3d,'X [m]'); ylabel(ax3d,'Y [m]'); zlabel(ax3d,'Z [m]');
title(ax3d,'3D Animation — Fixed-Wing','FontSize',13,'FontWeight','bold');
set(ax3d,'FontSize',8); view(ax3d,35,25);
mg = max(10, max(max(abs(rec.x)),max(abs(rec.y)))*0.25);
ref_x=rec.x_d; ref_y=rec.y_d; ref_z=rec.z_d;
xlim(ax3d,[min([rec.x ref_x])-mg, max([rec.x ref_x])+mg]);
ylim(ax3d,[min([rec.y ref_y])-mg, max([rec.y ref_y])+mg]);
zlim(ax3d,[max(0,min([rec.z ref_z])-mg), max([rec.z ref_z])+mg]);
plot3(ax3d,ref_x,ref_y,ref_z,'--','Color',[.75 .75 .75],'LineWidth',1.2);
trail = animatedline(ax3d,'Color',[0 .45 .74],'LineWidth',1.5);

%% ===== AIRCRAFT 3D OBJECT =====
if use_mesh
    hg_aircraft = hgtransform('Parent', ax3d);
    patch('Parent', hg_aircraft, ...
          'Vertices', aircraft_mesh.vertices, ...
          'Faces',    aircraft_mesh.faces, ...
          'FaceColor', [0.45 0.55 0.72], ...
          'EdgeColor', [0.25 0.30 0.40], ...
          'EdgeAlpha', 0.15, ...
          'FaceAlpha', 0.95, ...
          'FaceLighting', 'gouraud', ...
          'AmbientStrength', 0.5, ...
          'DiffuseStrength', 0.7, ...
          'SpecularStrength', 0.3);
    camlight(ax3d, 'headlight');
    camlight(ax3d, 'left');
    lighting(ax3d, 'gouraud');
    fprintf('[viz] 3D mesh aircraft active (%s).\n', aircraft_mesh.source);
else
    L_fus = L_fus_phys;  L_wing = L_wing_phys;  L_tail = L_tail_phys;
    hg_aircraft = [];
    fus_line   = plot3(ax3d,nan,nan,nan,'k-','LineWidth',2.5);
    wing_line  = plot3(ax3d,nan,nan,nan,'-','Color',[.2 .5 .8],'LineWidth',2.2);
    tail_line  = plot3(ax3d,nan,nan,nan,'-','Color',[.6 .3 .1],'LineWidth',1.8);
    vtail_line = plot3(ax3d,nan,nan,nan,'-','Color',[.6 .3 .1],'LineWidth',1.8);
    nose_dot   = plot3(ax3d,nan,nan,nan,'ko','MarkerSize',4,'MarkerFaceColor',[.8 .1 .1]);
    fprintf('[viz] Line-based aircraft fallback active.\n');
end

xl3=xlim(ax3d); yl3=ylim(ax3d); zl3=zlim(ax3d);
ttxt = text(ax3d,xl3(1)+.2,yl3(2)-.2,zl3(2),'t=0.0s','FontSize',10,'FontWeight','bold');

%% ===== SIGNAL PLOT HELPERS =====
Te = t(end);
    function h = aplot(ax, tit, yl, xlab)
        hold(ax,'on'); grid(ax,'on'); title(ax,tit,'FontSize',9,'FontWeight','bold');
        ylabel(ax,yl); xlim(ax,[0 Te]); set(ax,'FontSize',7);
        if nargin<4; set(ax,'XTickLabel',[]); else; xlabel(ax,xlab); end
        h = ax;
    end

%% ===== SIGNAL PLOTS =====
B=[0 .45 .74]; R=[.85 .33 .10]; G=[.47 .67 .19]; P=[.5 0 .5];
aplot(ax_p,'Position','[m]');
al={};
al{1}=animatedline(ax_p,'Color',[.8 .4 .4],'LineWidth',.8,'LineStyle','--');
al{2}=animatedline(ax_p,'Color',B,'LineWidth',1);
al{3}=animatedline(ax_p,'Color',[.4 .8 .4],'LineWidth',.8,'LineStyle','--');
al{4}=animatedline(ax_p,'Color',R,'LineWidth',1);
al{5}=animatedline(ax_p,'Color',[.4 .4 .8],'LineWidth',.8,'LineStyle','--');
al{6}=animatedline(ax_p,'Color',G,'LineWidth',1);
legend(ax_p,{'xr','x','yr','y','zr','z'},'FontSize',5,'Location','eastoutside');

aplot(ax_e,'Errors','[m]');
al{7}=animatedline(ax_e,'Color',B,'LineWidth',1);
al{8}=animatedline(ax_e,'Color',R,'LineWidth',1);
al{9}=animatedline(ax_e,'Color',G,'LineWidth',1);
legend(ax_e,{'ex','ey','ez'},'FontSize',6,'Location','eastoutside');

aplot(ax_a,'Attitude','[deg]');
al{10}=animatedline(ax_a,'Color',B,'LineWidth',1);
al{11}=animatedline(ax_a,'Color',R,'LineWidth',1);
al{12}=animatedline(ax_a,'Color',G,'LineWidth',1);
yline(ax_a, rad2deg(p.phi_max),'r--','LineWidth',.5);
yline(ax_a,-rad2deg(p.phi_max),'r--','LineWidth',.5);
legend(ax_a,{'\phi','\theta','\psi'},'FontSize',6,'Location','eastoutside');

aplot(ax_c,'Controls','u','Time [s]');
al{13}=animatedline(ax_c,'Color',B,'LineWidth',1);
al{14}=animatedline(ax_c,'Color',R,'LineWidth',1);
al{15}=animatedline(ax_c,'Color',G,'LineWidth',1);
al{16}=animatedline(ax_c,'Color',P,'LineWidth',1);
legend(ax_c,{'\delta_t','\delta_a','\delta_e','\delta_r'},'FontSize',5,'Location','eastoutside');

%% ===== BATTERY PLOTS =====
aplot(ax_s,'SOC','%'); ylim(ax_s,[0 105]);
al{17}=animatedline(ax_s,'Color',[.2 .65 .2],'LineWidth',1.2);
al{21}=animatedline(ax_s,'Color',[.9 .7 0],'LineWidth',.8,'LineStyle','--');
al{22}=animatedline(ax_s,'Color',[.9 .15 .15],'LineWidth',.8,'LineStyle','--');

yyaxis(ax_vi,'left'); hold(ax_vi,'on'); grid(ax_vi,'on');
al{18}=animatedline(ax_vi,'Color',B,'LineWidth',1);
ylabel(ax_vi,'V'); set(ax_vi,'YColor',B,'FontSize',7,'XTickLabel',[]);
xlim(ax_vi,[0 Te]); ylim(ax_vi,[min(bms.voltage)*.95, max(bms.voltage)*1.02]);
yyaxis(ax_vi,'right');
al{19}=animatedline(ax_vi,'Color',R,'LineWidth',1);
ylabel(ax_vi,'A'); set(ax_vi,'YColor',R);
ylim(ax_vi,[0 max(bms.current)*1.3+.5]);
title(ax_vi,'V & I','FontSize',9,'FontWeight','bold');

aplot(ax_pw,'Power','[W]','Time [s]');
ylim(ax_pw,[0 max(bms.power)*1.2+1]);
al{20}=animatedline(ax_pw,'Color',P,'LineWidth',1);

%% ===== BATTERY STATUS PANEL =====
annotation(fig,'rectangle',[.88 .22 .04 .17],'Color',[.4 .4 .4],'LineWidth',1.2,'FaceColor',[.92 .92 .92]);
bfill=annotation(fig,'rectangle',[.88 .22 .04 .17],'Color','none','FaceColor',[.2 .7 .2]);
bsoc=annotation(fig,'textbox',[.865 .395 .07 .03],'String','100%','FontSize',12,...
    'FontWeight','bold','EdgeColor','none','HorizontalAlignment','center','Color',[.2 .7 .2]);
tx=.74; tw=.13;
bv=annotation(fig,'textbox',[tx .345 tw .022],'String','V: --','FontSize',8,'FontWeight','bold','EdgeColor','none');
bi=annotation(fig,'textbox',[tx .323 tw .022],'String','I: --','FontSize',8,'FontWeight','bold','EdgeColor','none');
bp=annotation(fig,'textbox',[tx .301 tw .022],'String','P: --','FontSize',8,'FontWeight','bold','EdgeColor','none');
br=annotation(fig,'textbox',[tx .279 tw .022],'String','Rem: --','FontSize',8,'FontWeight','bold','EdgeColor','none');
annotation(fig,'textbox',[tx .250 tw .028],'String',sprintf('Mission: %s',mfeas.status),...
    'FontSize',8.5,'FontWeight','bold','EdgeColor','none','Color',mfeas.color,...
    'BackgroundColor',[.95 .95 .95],'HorizontalAlignment','center','VerticalAlignment','middle');
bw=annotation(fig,'textbox',[tx .215 tw .025],'String','Battery OK','FontSize',8,...
    'FontWeight','bold','EdgeColor','none','HorizontalAlignment','center','Color',[.2 .7 .2]);

%% ===== BOTTOM PANEL: CONTROL TYPE + ANIMATION CONTROLS =====
% --- Control Type Selection Row ---
ctrl_bg = uibuttongroup(fig,'Units','normalized','Position',[0.01 0.065 0.35 0.045],...
    'Title','Control Type','FontSize',9,'FontWeight','bold',...
    'BackgroundColor',[0.94 0.94 0.96],'SelectionChangeFcn',@ctrlTypeCB);

% Radio buttons
has_gwo = ~isempty(p.gain_sets.gwo);
has_pso = ~isempty(p.gain_sets.pso);
rb_x = 0.01; rb_w = 0.11;
if has_gwo
    rb_gwo = uicontrol(ctrl_bg,'Style','radiobutton','String','PID-GWO',...
        'Units','normalized','Position',[rb_x 0.25 rb_w 0.5],...
        'FontSize',9,'FontWeight','bold','Tag','rb_gwo');
end
if has_pso
    rb_pso = uicontrol(ctrl_bg,'Style','radiobutton','String','PID-PSO',...
        'Units','normalized','Position',[rb_x+0.12 0.25 rb_w 0.5],...
        'FontSize',9,'FontWeight','bold','Tag','rb_pso');
end
rb_man = uicontrol(ctrl_bg,'Style','radiobutton','String','PID-Manuel',...
    'Units','normalized','Position',[rb_x+0.24 0.25 rb_w+0.04 0.5],...
    'FontSize',9,'FontWeight','bold','Tag','rb_man');

% Set initial selection
switch p.active_control
    case 'gwo'
        if has_gwo; set(ctrl_bg,'SelectedObject',rb_gwo); end
    case 'pso'
        if has_pso; set(ctrl_bg,'SelectedObject',rb_pso); end
    otherwise
        set(ctrl_bg,'SelectedObject',rb_man);
end

% --- Tune button (visible only for Manual) ---
btn_tune = uicontrol(fig,'Style','pushbutton','String','Tune...',...
    'Units','normalized','Position',[0.37 0.07 0.07 0.035],...
    'FontSize',9,'FontWeight','bold',...
    'BackgroundColor',[0.85 0.75 0.55],'ForegroundColor','k',...
    'Callback',@tuneCB,'Visible','off');

% --- Re-Simulate button (always visible) ---
btn_resim = uicontrol(fig,'Style','pushbutton','String','Re-Simulate',...
    'Units','normalized','Position',[0.45 0.07 0.09 0.035],...
    'FontSize',9,'FontWeight','bold',...
    'BackgroundColor',[0.20 0.55 0.75],'ForegroundColor','w',...
    'Callback',@resimCB);

% --- Status label showing active control ---
lbl_ctrl = annotation(fig,'textbox',[0.555 0.068 0.10 0.035],...
    'String',sprintf('Active: %s',ctrl_type_str),...
    'FontSize',9,'FontWeight','bold','EdgeColor','none',...
    'Color',[0.2 0.4 0.7],'HorizontalAlignment','left');

% --- Show Tune button if Manual is active ---
if strcmp(p.active_control,'manual')
    set(btn_tune,'Visible','on');
end

% --- Animation Controls Row ---
btn=uicontrol(fig,'Style','pushbutton','String','Start Animation',...
    'FontSize',11,'FontWeight','bold','Units','normalized','Position',[0.22 0.015 0.14 0.04],...
    'BackgroundColor',[.3 .7 .3],'ForegroundColor','w','Callback',@startCB);
uicontrol(fig,'Style','text','String','Speed:','Units','normalized',...
    'Position',[0.02 0.018 0.04 0.025],'FontSize',9,'BackgroundColor','w','HorizontalAlignment','right');
uicontrol(fig,'Style','slider','Units','normalized',...
    'Min',0,'Max',1,'Value',1/3,'Position',[0.065 0.018 0.08 0.025],'Callback',@speedCB);
slbl=uicontrol(fig,'Style','text','String','1.0x','Units','normalized',...
    'Position',[0.15 0.018 0.04 0.025],'FontSize',9,'FontWeight','bold','BackgroundColor','w');
stxt=annotation(fig,'textbox',[0.38 0.012 0.10 0.035],'String','t = 0.0 s',...
    'FontSize',10,'FontWeight','bold','EdgeColor','none','HorizontalAlignment','center');

%% ===== STORE RE-SIMULATION DATA =====
sim_data = struct('t',t,'xd',xd,'yd',yd,'zd',zd,'psid',psid,'p',p);
fig.UserData = struct(...
    'go',false,'speed',1.0,'stop',false,'resim_needed',false,...
    'active_control',p.active_control,...
    'sim_data',sim_data,...
    'anim_idx',1);

%% ===== INITIAL STATE =====
draw3d(1); updplots(1); updbat(1); drawnow;

%% ===== WAIT FOR START =====
fprintf('Dashboard ready.\n');
while isvalid(fig) && ~fig.UserData.go; pause(0.05); end
if ~isvalid(fig); return; end
set(btn,'Enable','off','String','Running...','BackgroundColor',[.6 .6 .6]);

%% ===== ANIMATION LOOP WITH RE-SIM SUPPORT =====
while isvalid(fig) && ~fig.UserData.stop
    for k=2:length(idx)
        if ~isvalid(fig) || fig.UserData.stop; break; end
        if fig.UserData.resim_needed; break; end
        dtf = tf(k)-tf(k-1);
        spd = fig.UserData.speed;
        ts=tic; draw3d(k); updplots(k); updbat(k);
        r=dtf/spd - toc(ts); if r>0; pause(r); end
    end

    if ~isvalid(fig); return; end
    if fig.UserData.stop; break; end

    % Handle re-simulation
    if fig.UserData.resim_needed
        do_resimulate();
        fig.UserData.resim_needed = false;
        fig.UserData.go = true;
        continue;
    end
    break;
end

if isvalid(fig)
    set(btn,'String','Done','BackgroundColor',[.4 .4 .4]);
end
fprintf('Animation complete.\n');

%% ===== POST-ANALYSIS =====
fprintf('Generating analysis plots...\n');
post_analysis_plots_fw(t, rec, bms, p, mfeas);
fprintf('  8 analysis figures generated.\n');

%% ======================================================================
%  NESTED HELPER FUNCTIONS
%  ======================================================================

    function draw3d(kk)
        ii=idx(kk); px=rec.x(ii); py=rec.y(ii); pz=rec.z(ii);
        psi_k = rec.psi(ii);

        Rm_ned = rotm(rec.phi(ii), rec.theta(ii), psi_k);
        Rd = Rm_ned;
        Rd(3,:) = -Rd(3,:);
        Rd(:,3) = -Rd(:,3);

        if ~isempty(hg_aircraft)
            T = eye(4);
            T(1:3,1:3) = Rd;
            T(1:3,4)   = [px; py; pz];
            set(hg_aircraft, 'Matrix', T);
        else
            pos = [px;py;pz];
            fus_b   = [L_fus 0 0; -L_fus 0 0]';
            wing_b  = [0 -L_wing 0; 0 L_wing 0]';
            tail_b  = [-L_fus -L_tail*0.6 0; -L_fus L_tail*0.6 0]';
            vtail_b = [-L_fus 0 0; -L_fus 0 L_tail]';
            fus_w   = Rd*fus_b + pos;
            wing_w  = Rd*wing_b + pos;
            tail_w  = Rd*tail_b + pos;
            vtail_w = Rd*vtail_b + pos;
            nose_w  = Rd*[L_fus;0;0] + pos;
            set(fus_line,'XData',fus_w(1,:),'YData',fus_w(2,:),'ZData',fus_w(3,:));
            set(wing_line,'XData',wing_w(1,:),'YData',wing_w(2,:),'ZData',wing_w(3,:));
            set(tail_line,'XData',tail_w(1,:),'YData',tail_w(2,:),'ZData',tail_w(3,:));
            set(vtail_line,'XData',vtail_w(1,:),'YData',vtail_w(2,:),'ZData',vtail_w(3,:));
            set(nose_dot,'XData',nose_w(1),'YData',nose_w(2),'ZData',nose_w(3));
        end

        addpoints(trail,px,py,pz);
        set(ttxt,'String',sprintf('t=%.1fs',t(ii)));

        % Camera
        ac_pos = [px, py, pz];
        if cam_follow
            cs_k = cos(psi_k); ss_k = sin(psi_k);
            off_body = [-cam_dist * cam_zoom, 0, cam_height * cam_zoom];
            cam_target = ac_pos + [off_body(1)*cs_k - off_body(2)*ss_k, ...
                                   off_body(1)*ss_k + off_body(2)*cs_k, ...
                                   off_body(3)];
            if ~cam_init
                cam_pos_cur = cam_target;
                cam_init = true;
            else
                cam_pos_cur = cam_pos_cur + cam_smooth * (cam_target - cam_pos_cur);
            end
            set(ax3d, 'CameraTarget', ac_pos, 'CameraPosition', cam_pos_cur);
        else
            set(ax3d, 'CameraTarget', ac_pos, ...
                      'CameraPosition', ac_pos + [-40 -40 20]);
        end
    end

    function updplots(kk)
        ii=idx(kk); tv=t(ii);
        addpoints(al{1},tv,rec.x_d(ii)); addpoints(al{2},tv,rec.x(ii));
        addpoints(al{3},tv,rec.y_d(ii)); addpoints(al{4},tv,rec.y(ii));
        addpoints(al{5},tv,rec.z_d(ii)); addpoints(al{6},tv,rec.z(ii));
        addpoints(al{7},tv,rec.x_d(ii)-rec.x(ii));
        addpoints(al{8},tv,rec.y_d(ii)-rec.y(ii));
        addpoints(al{9},tv,rec.z_d(ii)-rec.z(ii));
        addpoints(al{10},tv,rad2deg(rec.phi(ii)));
        addpoints(al{11},tv,rad2deg(rec.theta(ii)));
        addpoints(al{12},tv,rad2deg(rec.psi(ii)));
        addpoints(al{13},tv,rec.d_t(ii));  addpoints(al{14},tv,rec.d_a(ii));
        addpoints(al{15},tv,rec.d_e(ii));  addpoints(al{16},tv,rec.d_r(ii));
        addpoints(al{17},tv,bms.soc(ii)*100);

        soc_req_rem = mfeas.SOC_req_total * ((N - ii) / N);
        sw_dyn = soc_req_rem + p.bat.soc_warn;
        sc_dyn = soc_req_rem + p.bat.soc_crit;
        addpoints(al{21},tv,sw_dyn*100);
        addpoints(al{22},tv,sc_dyn*100);

        addpoints(al{18},tv,bms.voltage(ii));
        addpoints(al{19},tv,bms.current(ii));
        addpoints(al{20},tv,bms.power(ii));
        set(stxt,'String',sprintf('t = %.2f s',tv));
        set(slbl,'String',sprintf('%.1fx',fig.UserData.speed));
        drawnow limitrate;
    end

    function updbat(kk)
        ii=idx(kk); s=bms.soc(ii); v=bms.voltage(ii);
        In=bms.current(ii); Pn=bms.power(ii);

        soc_req_rem = mfeas.SOC_req_total * ((N - ii) / N);
        sw_dyn = soc_req_rem + p.bat.soc_warn;
        sc_dyn = soc_req_rem + p.bat.soc_crit;

        if s>sw_dyn;     bc=[.2 .7 .2];  ws='Battery OK';
        elseif s>sc_dyn; bc=[.95 .75 0];  ws='Battery Low';
        else;            bc=[.9 .15 .15]; ws='BATTERY CRITICAL'; end

        set(bfill,'Position',[.88 .22 .04 max(.001,s*.17)],'FaceColor',bc);
        set(bsoc,'String',sprintf('%.0f%%',s*100),'Color',bc);
        set(bv,'String',sprintf('V: %.1f V',v));
        set(bi,'String',sprintf('I: %.1f A',In));
        set(bp,'String',sprintf('P: %.0f W',Pn));
        eff_Pn = max(p.bat.P_base, Pn);
        set(br,'String',sprintf('Rem: %.0fs',s*p.bat.capacity_As*p.bat.V_nom/eff_Pn));
        set(bw,'String',ws,'Color',bc);
    end

    function update_gain_display()
        % Update the gain table with current p PID values
        gain_data_new = cell(nPID, 4);
        for j = 1:nPID
            pid_s = p.(pid_keys{j});
            gain_data_new{j,1} = pid_names{j};
            gain_data_new{j,2} = sprintf('%.4f', pid_s.Kp);
            gain_data_new{j,3} = sprintf('%.4f', pid_s.Ki);
            gain_data_new{j,4} = sprintf('%.4f', pid_s.Kd);
        end
        set(utt,'Data',gain_data_new);

        % Update title
        ac = fig.UserData.active_control;
        if strcmp(ac,'gwo'); cs = 'GWO'; elseif strcmp(ac,'pso'); cs = 'PSO';
        else; cs = 'MANUAL'; end
        set(gain_title,'String',sprintf('Active Gains  [ %s ]',cs));
        set(lbl_ctrl,'String',sprintf('Active: %s',cs));
    end

    function ctrlTypeCB(src, ~)
        % Control type radio button changed
        sel = get(src,'SelectedObject');
        tag = get(sel,'Tag');

        switch tag
            case 'rb_gwo'
                fig.UserData.active_control = 'gwo';
                set(btn_tune,'Visible','off');
            case 'rb_pso'
                fig.UserData.active_control = 'pso';
                set(btn_tune,'Visible','off');
            case 'rb_man'
                fig.UserData.active_control = 'manual';
                set(btn_tune,'Visible','on');
        end
    end

    function resimCB(~, ~)
        % Re-Simulate button clicked — apply selected control type gains
        fig.UserData.resim_needed = true;
        fig.UserData.go = false;
        set(btn,'String','Re-Simulating...','BackgroundColor',[.8 .6 .2]);
        drawnow;
    end

    function do_resimulate()
        % Apply gains from selected control type and re-run simulation
        ac = fig.UserData.active_control;
        sd = fig.UserData.sim_data;

        % Get the gain vector for the selected type
        switch ac
            case 'gwo'
                if ~isempty(p.gain_sets.gwo)
                    gv = p.gain_sets.gwo;
                else
                    fprintf('[!] No GWO gains available. Using manual.\n');
                    gv = p.gain_sets.manual;
                    fig.UserData.active_control = 'manual';
                end
            case 'pso'
                if ~isempty(p.gain_sets.pso)
                    gv = p.gain_sets.pso;
                else
                    fprintf('[!] No PSO gains available. Using manual.\n');
                    gv = p.gain_sets.manual;
                    fig.UserData.active_control = 'manual';
                end
            otherwise  % manual
                gv = p.gain_sets.manual;
        end

        % Apply gains
        p = apply_pid_gain_vector(p, gv);
        p.quiet = true;  % suppress plant output during re-simulation

        % Re-run simulation
        fprintf('\n>> Re-simulating with %s gains...\n', ac);
        [rec, bms] = simulate_fw(p, sd.t, sd.xd, sd.yd, sd.zd, sd.psid);
        mfeas = mission_feasibility(p, rec, bms, sd.t);

        % Update parameters in sim_data
        sd.p = p;
        fig.UserData.sim_data = sd;

        % Update gain display
        update_gain_display();

        % Clear all animatedlines and re-initialize
        for i = 1:numel(al)
            if ~isempty(al{i})
                reset(al{i});
            end
        end
        reset(trail);

        % Re-plot reference trajectory in 3D (already static, no need to clear)
        % Update 3D axis limits
        mg2 = max(10, max(max(abs(rec.x)),max(abs(rec.y)))*0.25);
        xlim(ax3d,[min([rec.x rec.x_d])-mg2, max([rec.x rec.x_d])+mg2]);
        ylim(ax3d,[min([rec.y rec.y_d])-mg2, max([rec.y rec.y_d])+mg2]);
        zlim(ax3d,[max(0,min([rec.z rec.z_d])-mg2), max([rec.z rec.z_d])+mg2]);

        % Update plot y-limits for battery
        ylim(ax_vi,[min(bms.voltage)*.95, max(bms.voltage)*1.02]);
        ylim(ax_pw,[0 max(bms.power)*1.2+1]);

        % Draw first frame
        draw3d(1); updplots(1); updbat(1); drawnow;

        % Update button state
        set(btn,'String','Running...','BackgroundColor',[.6 .6 .6],'Enable','on');

        fprintf('>> Re-simulation complete. Animation restarted.\n\n');
    end

    function tuneCB(~, ~)
        % Open tuning dialog for manual PID adjustment
        open_tuning_dialog();
    end

    function open_tuning_dialog()
        % Create a modal tuning dialog
        tfig = figure('Name','PID Manual Tuning','NumberTitle','off',...
            'MenuBar','none','ToolBar','none','Resize','off',...
            'Units','pixels','Position',[350 150 520 430],...
            'Color',[0.15 0.17 0.22],'WindowStyle','modal',...
            'CloseRequestFcn',@tuneCloseCB);
        movegui(tfig,'center');

        % Title
        uicontrol(tfig,'Style','text','String','PID MANUAL TUNING',...
            'Units','pixels','Position',[20 395 480 25],...
            'FontSize',14,'FontWeight','bold','ForegroundColor',[0.9 0.95 1],...
            'BackgroundColor',[0.15 0.17 0.22],'HorizontalAlignment','center');

        uicontrol(tfig,'Style','text',...
            'String','Adjust Kp, Ki, Kd for each PID loop. Double-click cells to edit.',...
            'Units','pixels','Position',[20 370 480 20],...
            'FontSize',8,'ForegroundColor',[0.6 0.65 0.7],...
            'BackgroundColor',[0.15 0.17 0.22],'HorizontalAlignment','center');

        % Prepare editable table data
        tune_data = cell(nPID, 4);
        for j = 1:nPID
            pid_s = p.(pid_keys{j});
            tune_data{j,1} = pid_names{j};
            tune_data{j,2} = pid_s.Kp;
            tune_data{j,3} = pid_s.Ki;
            tune_data{j,4} = pid_s.Kd;
        end

        % Editable uitable
        utt_tune = uitable(tfig,'Units','pixels','Position',[20 80 480 280],...
            'ColumnName',{'PID Loop','Kp','Ki','Kd'},...
            'ColumnWidth',{100,110,110,110},...
            'ColumnEditable',[false, true, true, true],...
            'Data',tune_data,...
            'FontName','FixedWidth','FontSize',10,...
            'CellEditCallback',@tuneEditCB,...
            'BackgroundColor',[0.94 0.94 0.96],...
            'ForegroundColor',[0.1 0.1 0.1]);

        % Buttons
        uicontrol(tfig,'Style','pushbutton','String','Apply & Re-Simulate',...
            'Units','pixels','Position',[100 25 160 40],...
            'FontSize',11,'FontWeight','bold',...
            'BackgroundColor',[0.20 0.55 0.75],'ForegroundColor','w',...
            'Callback',@tuneApplyCB);

        uicontrol(tfig,'Style','pushbutton','String','Cancel',...
            'Units','pixels','Position',[280 25 120 40],...
            'FontSize',11,'FontWeight','bold',...
            'BackgroundColor',[0.6 0.3 0.3],'ForegroundColor','w',...
            'Callback',@tuneCloseCB);

        % Store reference to table in figure
        tfig.UserData = struct('table',utt_tune,'applied',false);

        % Wait for dialog to close
        uiwait(tfig);

        % If applied, update gains and re-simulate
        if tfig.UserData.applied
            % Read gains from table
            td = get(utt_tune,'Data');
            new_gains = zeros(1, 21);
            for j = 1:nPID
                new_gains((j-1)*3+1) = td{j,2};
                new_gains((j-1)*3+2) = td{j,3};
                new_gains((j-1)*3+3) = td{j,4};
            end
            p.gain_sets.manual = new_gains;
            fig.UserData.active_control = 'manual';
            update_gain_display();
            fig.UserData.resim_needed = true;
            fig.UserData.go = false;
            set(btn,'String','Re-Simulating...','BackgroundColor',[.8 .6 .2]);
            drawnow;
        end
    end

    function tuneEditCB(src, ev)
        % Validate edited cell - ensure non-negative numeric
        val = str2double(ev.EditData);
        if isnan(val) || val < 0
            % Revert to previous value
            old_data = get(src,'Data');
            old_data{ev.Indices(1),ev.Indices(2)} = ev.PreviousData;
            set(src,'Data',old_data);
        end
    end

    function tuneApplyCB(src, ~)
        % Mark as applied and close dialog
        tfig_h = ancestor(src,'figure');
        tfig_h.UserData.applied = true;
        uiresume(tfig_h);
        delete(tfig_h);
    end

    function tuneCloseCB(src, ~)
        src.UserData.applied = false;
        uiresume(src);
        delete(src);
    end

    function startCB(~,~)
        s=fig.UserData; s.go=true; fig.UserData=s;
    end

    function speedCB(src,~)
        sv=src.Value; sp=0.1*10^(sv*3); sp=round(sp*10)/10;
        s=fig.UserData; s.speed=max(.1,min(100,sp)); fig.UserData=s;
        set(slbl,'String',sprintf('%.1fx',s.speed));
    end

    function closeCB(src,~)
        s=src.UserData; s.stop=true; src.UserData=s; delete(src);
    end
end

function R=rotm(ph,th,ps)
%ROTM  ZYX body-to-NED rotation matrix (z-down inertial convention).
cp=cos(ph);sp=sin(ph);ct=cos(th);st=sin(th);cs=cos(ps);ss=sin(ps);
R=[ct*cs,sp*st*cs-cp*ss,cp*st*cs+sp*ss;ct*ss,sp*st*ss+cp*cs,cp*st*ss-sp*cs;-st,sp*ct,cp*ct];
end
