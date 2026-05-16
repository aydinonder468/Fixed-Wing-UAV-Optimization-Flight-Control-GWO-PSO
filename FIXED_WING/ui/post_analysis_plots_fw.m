function post_analysis_plots_fw(t, rec, bms, p, mfeas)
%POST_ANALYSIS_PLOTS_FW  Full-simulation analysis figures (8 windows).

lw = 1.2;

% 1. Position tracking
figure('Color','w','Name','Position Tracking','NumberTitle','off');
subplot(3,1,1); plot(t,rec.x_d,'r--',t,rec.x,'b','LineWidth',lw);
ylabel('X [m]'); legend('Ref','Act'); title('Position Tracking'); grid on;
subplot(3,1,2); plot(t,rec.y_d,'r--',t,rec.y,'b','LineWidth',lw);
ylabel('Y [m]'); legend('Ref','Act'); grid on;
subplot(3,1,3); plot(t,rec.z_d,'r--',t,rec.z,'b','LineWidth',lw);
ylabel('Z [m]'); xlabel('Time [s]'); legend('Ref','Act'); grid on;

% 2. Tracking errors
figure('Color','w','Name','Tracking Errors','NumberTitle','off');
subplot(3,1,1); plot(t,rec.x_d-rec.x,'b','LineWidth',lw); ylabel('e_x'); title('Tracking Errors'); grid on;
subplot(3,1,2); plot(t,rec.y_d-rec.y,'b','LineWidth',lw); ylabel('e_y'); grid on;
subplot(3,1,3); plot(t,rec.z_d-rec.z,'b','LineWidth',lw); ylabel('e_z'); xlabel('Time [s]'); grid on;

% 3. Attitude
figure('Color','w','Name','Attitude Angles','NumberTitle','off');
subplot(3,1,1); plot(t,rad2deg(rec.phi),'b','LineWidth',lw);
hold on; yline(rad2deg(p.phi_max),'r--'); yline(-rad2deg(p.phi_max),'r--');
ylabel('\phi [deg]'); title('Attitude Angles'); grid on;
subplot(3,1,2); plot(t,rad2deg(rec.theta),'b','LineWidth',lw);
hold on; yline(rad2deg(p.theta_max),'r--'); yline(-rad2deg(p.theta_max),'r--');
ylabel('\theta [deg]'); grid on;
subplot(3,1,3); plot(t,rad2deg(rec.psi),'b','LineWidth',lw);
ylabel('\psi [deg]'); xlabel('Time [s]'); grid on;

% 4. Control inputs
figure('Color','w','Name','Control Inputs','NumberTitle','off');
fd={'d_t','d_e','d_a','d_r'};
nm={'\delta_t (Throttle)','\delta_e (Elevator)','\delta_a (Aileron)','\delta_r (Rudder)'};
for i=1:4
    subplot(4,1,i); plot(t,rec.(fd{i}),'b','LineWidth',lw);
    ylabel(nm{i}); grid on;
    if i==1; title('Control Inputs'); end
    if i==4; xlabel('Time [s]'); end
end

% 5. Airspeed, AoA, sideslip
figure('Color','w','Name','Aero Variables','NumberTitle','off');
subplot(3,1,1); plot(t,rec.airspeed,'b','LineWidth',lw);
hold on; yline(p.u0,'r--','LineWidth',.8);
ylabel('V [m/s]'); title('Aerodynamic Variables'); legend('Airspeed','Trim'); grid on;
subplot(3,1,2); plot(t,rad2deg(rec.alpha),'b','LineWidth',lw);
ylabel('\alpha [deg]'); grid on;
subplot(3,1,3); plot(t,rad2deg(rec.beta),'b','LineWidth',lw);
ylabel('\beta [deg]'); xlabel('Time [s]'); grid on;

% 6. Battery + BMS
figure('Color','w','Name','Battery & BMS','NumberTitle','off');
subplot(4,1,1); plot(t,bms.soc*100,'Color',[.2 .6 .2],'LineWidth',lw);
hold on;
N_steps = length(t);
soc_req_rem = mfeas.SOC_req_total * ((N_steps - (1:N_steps)) / N_steps);
plot(t, (soc_req_rem + p.bat.soc_warn)*100, '--', 'Color', [.9 .7 0], 'LineWidth', 0.8);
plot(t, (soc_req_rem + p.bat.soc_crit)*100, '--', 'Color', [.9 .15 .15], 'LineWidth', 0.8);
ylabel('SOC [%]'); ylim([0 105]); title('Battery & BMS'); grid on;

subplot(4,1,2);
if isfield(bms,'I_req')
    plot(t,bms.I_req,'Color',[.8 .4 .4],'LineWidth',.8,'LineStyle','--'); hold on;
end
plot(t,bms.current,'Color',[.85 .33 .1],'LineWidth',lw);
ylabel('I [A]'); legend('Requested','BMS Filtered'); grid on;

subplot(4,1,3); plot(t,bms.voltage,'Color',[0 .45 .74],'LineWidth',lw);
yline(p.bat.V_cutoff,'r--','LineWidth',.8);
ylabel('V [V]'); legend('Terminal','Cutoff'); grid on;

subplot(4,1,4); plot(t,bms.power,'Color',[.5 0 .5],'LineWidth',lw);
ylabel('P [W]'); xlabel('Time [s]'); grid on;

% 7. 3D trajectory
figure('Color','w','Name','3D Trajectory','NumberTitle','off');
plot3(rec.x_d,rec.y_d,rec.z_d,'r--','LineWidth',1.5); hold on;
plot3(rec.x,rec.y,rec.z,'b','LineWidth',1.2);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('Ref','Act'); title('3D Trajectory'); grid on; axis equal; view(35,25);

% 8. Remaining flight time
rem_t = zeros(size(t));
dt = t(2) - t(1);
tau = 2.0;
alpha_f = dt / (tau + dt);
P_cruise = p.bat.P_base + p.bat.k_thrust * 0.5;
P_filt = P_cruise;
for k = 1:length(t)
    P_filt = P_filt + alpha_f * (bms.power(k) - P_filt);
    eff_power = max(p.bat.P_base, P_filt);
    rem_t(k) = bms.soc(k) * p.bat.capacity_As * p.bat.V_nom / eff_power;
end
figure('Color','w','Name','Remaining Flight Time','NumberTitle','off');
plot(t,rem_t,'Color',[0 .45 .74],'LineWidth',lw);
ylabel('Time [s]'); xlabel('Time [s]'); title('Remaining Flight Time'); grid on;

fprintf('  8 analysis figures generated.\n');
end
