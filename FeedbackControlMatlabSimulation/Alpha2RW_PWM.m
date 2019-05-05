function [wRW_new, PWM] = Alpha2RW_PWM(w,wdot,wRW,dt)
% Converts 

Vrail = 8; % Rail voltage

%motor constants
Kt = 0.00713; % Nm/A
Ke = 0.00713332454; % V/rad/s
R = 92.7; % Ohms


Jsc = bodyInertiaMatrix(3,.3,.1,.1); % predetermined 3U cubesat values
A = eye(3); % 3x3 reaction wheel identity matrix
Jw = rwInertiaMatrix(); % 3x3 reaction wheel inertia matrix.


torque = -A\((Jsc*wdot)+(cross(w,A*Jw*wRW+Jsc*w)));

wRW_dot = Jw\torque;

wRW_new = wRW + wRW_dot * dt;

volts = (wRW_new * Ke) + (torque * R)/(Kt);
PWM = (volts/Vrail)*100;
end