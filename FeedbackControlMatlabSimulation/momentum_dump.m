function [m, newTorque] = momentum_dump(w_rw, mag_body, bdot)
% momentum_dump adjusts the total torque to dump momentum from the system.
% Inputs: all 3x1 vectors
%   w_rw = angular velocity of the reaction wheels
%   mag_body = magnetic field in the body frame
%   bdot = bang bang analytical bdot (bdotControl function)
% Output: 3x1 vector
%   newTorque = torque capable of being exerted by torque rods
%*************************************************************************

% Maximum dipole strength
max_dpl = [2;2;2]; % A*m^2

% 3x3 reaction wheel inertia matrix
J_rw = rwInertiaMatrix(); 

% Reaction wheel angular momentum
h_rw = J_rw*w_rw;

%Cross normalized vector with b-dot to create desired dipole moment
m = cross(h_rw/norm(h_rw) , mag_body);

%Ensure desired dipole moment is not greater than torque rod capacity
for i=1:3
if (m(i) > max_dpl(i))
     m(i) = max_dpl(i);
else
     m(i) = m(i);
end
end

%Find torque from dipole moment and mag field
%GAIN IT!
newTorque = 1000.*cross (m,mag_body); 
m = m;
%Set direction based on direction of reaction wheel's angular velocity
%newTorque = [ sign(w_rw(1))*newTorque(1) ; sign(w_rw(2))*newTorque(2) ; sign(w_rw(3))*newTorque(3)];

%newTorque = [0;0;0];

end

