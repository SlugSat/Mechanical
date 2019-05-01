function newTorque = momentum_dump(w_rw, mag_body, bdot)
% momentum_dump adjusts the total torque to dump momentum from the system.
% Inputs: all 3x1 vectors
%   w_rw = angular velocity of the reaction wheels
%   mag_body = magnetic field in the body frame
%   bdot = bang bang analytical bdot (bdotControl function)
% Output: 3x1 vector
%   newTorque = torque capable of being exerted by torque rods
%*************************************************************************

% Maximum dipole strength
max_dpl = 2; % A*m^2

% 3x3 reaction wheel inertia matrix
J_rw = rwInertiaMatrix(); 

% Reaction wheel angular momentum
h_rw = -J_rw*w_rw;

%Cross normalized vector with b-dot to create desired dipole moment
m = cross(h_rw/norm(h_rw) , bdot);

%Ensure desired dipole moment is not greater than torque rod capacity
if m > max_dpl
    m = max_dpl;
else 
    m = m;
end

%Find torque from dipole moment and mag field
newTorque = cross (m,mag_body); 

%Set direction based on direction of reaction wheel's angular velocity
newTorque = [ -sign(w_rw(1))*newTorque(1) ; -sign(w_rw(2))*newTorque(2) ; -sign(w_rw(3))*newTorque(3)];

end

