%> @brief Converts reaction wheel parameters to PMW
%>
%> Inputs:
%> @param  w :   Craft's angular velocity (rad/s)
%> @param  wdot: Derivative term
%> @param  wRW:  Reaction wheel angular velocity (rad/s)
%> @param  dt:   Time step
%>
%> Outputs:
%> @retval  wRW_new: new reaction wheel angular velocity (rad/s)
%> @retval  PWM: Corresponding pulse width modulation signal
%**************************************************************************

function [wRW_new, PWM] = wdot2RW_PWM(w,wdot,wRW,dt)
    
%Max voltage
Vrail = 8; % Rail voltage

%Motor constants
Kt = 0.00713; % Nm/A
Ke = 0.00713332454; % V/rad/s
R = 92.7; % Ohms

%Craft parameters
Jsc = bodyInertia; % predetermined 2U cubesat values
A = eye(3); % 3x3 reaction wheel identity matrix
Jw = rwInertia; % 3x3 reaction wheel inertia matrix.

torque = -A\((Jsc*wdot)+(cross(w, A*Jw*wRW+Jsc*w)));

wRW_dot = Jw\torque;

wRW_new = wRW + wRW_dot * dt;

volts = (wRW_new * Ke) + (torque * R)/(Kt);
PWM = (volts/Vrail)*100;
end