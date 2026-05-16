function [u, xi, xf] = pid_ct(e, xi, xf, K, dt)
%PID_CT  Continuous-time PID — numerical integration step.
%  U(s) = Kp·E + Ki/s·E + Kd·s/(Tf·s+1)·E
%  States: xi=integrator, xf=filter
%  ODEs: dxi/dt=e, dxf/dt=(e-xf)/Tf
%  Anti-windup: freeze integrator when saturated.

dxf = (e - xf) / K.Tf;
xf  = xf + dxf * dt;

u_raw = K.Kp * e + K.Ki * xi + K.Kd * dxf;
u = max(K.lo, min(K.hi, u_raw));

if abs(u - u_raw) < 1e-10
    xi = xi + e * dt;
end
end
