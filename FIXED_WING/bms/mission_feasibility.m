function r = mission_feasibility(p, rec, bms, t)
%MISSION_FEASIBILITY  Post-simulation energy analysis using actual BMS data.

P = bms.power;

dt = t(2)-t(1);
E_req   = sum(P)*dt / 3600;                          % [Wh] energy used
E_total = (p.bat.capacity_mAh/1000) * p.bat.V_nom;   % [Wh] full battery
E_avail = E_total * p.bat.SOC0;                       % [Wh] at start

% Actual final SOC from simulation
SOC_final = bms.soc(end);

avg_P   = mean(P);
avg_I   = avg_P / p.bat.V_nom;
t_fly   = (E_avail*3600) / max(avg_P, 1);

% SOC consumed during mission
SOC_req = p.bat.SOC0 - SOC_final;

% Dynamic thresholds
r.soc_crit_req = SOC_req + p.bat.soc_crit;
r.soc_warn_req = SOC_req + p.bat.soc_warn;

% Feasibility
if p.bat.SOC0 < r.soc_crit_req
    r.status='MISSION IN DANGER'; r.color=[.9 .15 .15];
elseif p.bat.SOC0 < r.soc_warn_req
    r.status='RISKY';        r.color=[.9 .7 0];
else
    r.status='SAFE';         r.color=[.2 .7 .2];
end

r.SOC_req_total = SOC_req;
r.E_req=E_req; r.E_avail=E_avail; r.SOC_final=SOC_final;
r.avg_P=avg_P; r.avg_I=avg_I; r.t_fly=t_fly;

fprintf('\n===== MISSION FEASIBILITY =====\n');
fprintf('  Duration:  %.1f s\n', t(end));
fprintf('  Avg power: %.1f W | Avg current: %.2f A\n', avg_P, avg_I);
fprintf('  Energy:    %.2f Wh req / %.2f Wh avail\n', E_req, E_avail);
fprintf('  SOC:       %.0f%% -> %.0f%% (used %.0f%%)\n', p.bat.SOC0*100, SOC_final*100, SOC_req*100);
fprintf('  Status:    %s\n', r.status);
fprintf('===============================\n\n');
end
