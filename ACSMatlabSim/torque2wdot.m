%> @brief Converts torque on the craft into the angular acceleration
%>
%> Inputs
%> @param  w:      angular velocity of craft
%> @param  w_rw:   current reaction wheel angular rates
%> @param  t:      torque acting on craft
%>
%> Outputs
%> @retval  wdot:   time derivative of angular velocity of craft
%**************************************************************************

function wdot = torque2wdot(w,w_rw,t)

Jsc = bodyInertia; % predetermined 3U cubesat values
A = [1 0 0; 0 1 0; 0 0 1]; % 3x3 reaction wheel identity matrix
Jw= rwInertia; % 3x3 reaction wheel inertia matrix

wdot= Jsc\((-A*t)-(cross(w, A*Jw*w_rw + Jsc*w))); % This results in a 3x1 matrix. Returns the speed of the spacecraft with three reaction wheels.

end


















