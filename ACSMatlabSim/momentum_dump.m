function [m, newTorque] = momentum_dump(w_rw, mag_body)
% momentum_dump adjusts the total torque to dump momentum from the system.
% Inputs: all 3x1 vectors
%   w_rw = angular velocity of the reaction wheels
%   mag_body = magnetic field in the body frame
% Output: 3x1 vector
%   newTorque = torque capable of being exerted by torque rods
%*************************************************************************

% Maximum dipole strength
max_dpl = [2;2;2]; % A*m^2

% 3x3 reaction wheel inertia matrix
J_rw = rwInertiaMatrix(); 

% Reaction wheel angular momentum
h_rw = J_rw*w_rw;

%Cross normalized vector with mag field
m = cross(h_rw/norm(h_rw) , mag_body);

%Ensure desired dipole moment is not greater than torque rod capacity
m = 1000.*m;
for i=1:3
if (m(i) > max_dpl(i))
     m(i) = 2;
else
     m(i) = m(i);
end
end

%Find torque from dipole moment and mag field
newTorque = cross (m,mag_body); 

end

