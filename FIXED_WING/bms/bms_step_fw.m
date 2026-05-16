function [I_act, soc, V, P_act, k_avail, I_req] = bms_step_fw(d_t, d_e, d_a, I_act, soc, bat, dt)
%BMS_STEP_FW  One step of BMS for fixed-wing UAV (no rudder).
%
%  Battery + BMS coupled model:
%    1. Power demand:    P_req = P_base + k_thrust·|d_t| + k_ctrl·(|d_e|+|d_a|)
%    2. Requested I:     I_req = P_req / V_nom
%    3. BMS filter:      dI/dt = clamp((I_req - I_act)/tau, -slew, +slew)
%    4. Saturation:      0 <= I_act <= I_max
%    5. Voltage:         V = V_oc(SOC) - I_act·R_int
%    6. Under-V cutoff:  throttle if V < V_cutoff
%    7. SOC depletion:   I_act = 0 if SOC <= 0
%    8. SOC ODE:         dSOC/dt = -I_act / Q
%    9. Coupling:        k_avail = I_act / I_req  (0 to 1)

P_req = bat.P_base + bat.k_thrust*abs(d_t) ...
      + bat.k_ctrl*(abs(d_e) + abs(d_a));
I_req = P_req / bat.V_nom;

% BMS current filter ODE (Euler step)
dI = (I_req - I_act) / bat.tau_bms;
dI = max(-bat.slew_max, min(bat.slew_max, dI));
I_act = I_act + dI * dt;
I_act = max(0, min(bat.I_max, I_act));

% Equivalent circuit voltage
V = (bat.V_empty + soc*(bat.V_full - bat.V_empty)) - I_act*bat.R_int;
V = max(bat.V_empty * 0.85, V);

% Under-voltage protection
if V < bat.V_cutoff
    I_act = I_act * (V / bat.V_cutoff);
end

% SOC depletion
if soc <= 0
    I_act = 0;
end

% SOC ODE
soc = max(0, min(1, soc - (I_act / bat.capacity_As) * dt));

% Actual power
P_act = I_act * V;

% Coupling ratio
if I_req > 1e-6
    k_avail = min(1.0, I_act / I_req);
else
    k_avail = 1.0;
end
end
