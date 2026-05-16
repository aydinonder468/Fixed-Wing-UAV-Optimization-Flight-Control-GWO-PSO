function p_out = apply_pid_gain_vector(p_in, g)
%APPLY_PID_GAIN_VECTOR  Map a 21-element gain vector to the 7 PID structs.
%  Overwrites Kp, Ki, Kd for each PID while preserving Tf, lo, hi.
%
%  Gain ordering (cascaded architecture, no rudder):
%    g(1:3)   -> pid_alt         [Kp Ki Kd]  alt err -> theta_cmd
%    g(4:6)   -> pid_speed       [Kp Ki Kd]  speed err -> delta_t
%    g(7:9)   -> pid_hdg         [Kp Ki Kd]  heading err -> phi_cmd
%    g(10:12) -> pid_pitch_att   [Kp Ki Kd]  theta err -> q_ref
%    g(13:15) -> pid_pitch_rate  [Kp Ki Kd]  q err -> delta_e
%    g(16:18) -> pid_roll_att    [Kp Ki Kd]  phi err -> p_ref
%    g(19:21) -> pid_roll_rate   [Kp Ki Kd]  p err -> delta_a

    p_out = p_in;

    p_out.pid_alt.Kp        = g(1);  p_out.pid_alt.Ki        = g(2);  p_out.pid_alt.Kd        = g(3);
    p_out.pid_speed.Kp      = g(4);  p_out.pid_speed.Ki      = g(5);  p_out.pid_speed.Kd      = g(6);
    p_out.pid_hdg.Kp        = g(7);  p_out.pid_hdg.Ki        = g(8);  p_out.pid_hdg.Kd        = g(9);
    p_out.pid_pitch_att.Kp  = g(10); p_out.pid_pitch_att.Ki  = g(11); p_out.pid_pitch_att.Kd  = g(12);
    p_out.pid_pitch_rate.Kp = g(13); p_out.pid_pitch_rate.Ki = g(14); p_out.pid_pitch_rate.Kd = g(15);
    p_out.pid_roll_att.Kp   = g(16); p_out.pid_roll_att.Ki   = g(17); p_out.pid_roll_att.Kd   = g(18);
    p_out.pid_roll_rate.Kp  = g(19); p_out.pid_roll_rate.Ki  = g(20); p_out.pid_roll_rate.Kd  = g(21);
end
