%% bms_params.m — Battery and BMS protection parameters (fixed-wing)
%  Adds p.bat fields to the existing 'p' struct.
%  Usage: called from main_fw.m after fw_params.m

%% Battery (4S LiPo, 3000 mAh — fixed-wing uses larger battery)
p.bat.capacity_mAh = 3000;
p.bat.capacity_As  = p.bat.capacity_mAh * 3.6;
p.bat.V_nom  = 14.8;  p.bat.V_full = 16.8;  p.bat.V_empty = 12.0;
p.bat.R_int  = 0.030;  p.bat.I_max = 50;  p.bat.SOC0 = 1.0;
p.bat.P_base = 25;     % base avionics power [W] (lower than quadcopter)
p.bat.k_thrust = 50.0; % throttle power coefficient
p.bat.k_ctrl   = 3.0;  % control surface servo power coefficient
p.bat.soc_warn = 0.30;  p.bat.soc_crit = 0.15;

%% BMS Protection
p.bat.tau_bms  = 0.3;    % current filter time constant [s]
p.bat.slew_max = 20.0;   % max current rate of change [A/s]
p.bat.V_cutoff = 12.5;   % under-voltage cutoff [V]
