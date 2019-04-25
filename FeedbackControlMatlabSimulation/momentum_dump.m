function newTorque = momentum_dump(w_rw,mag_body)
% momentum_dump adjusts the total torque to dump momentum from the system.
% Inputs: all 3x1 vectors
%   w_rw = angular velocity of the reaction wheels
%   mag_body = magnetic field read from the body frame
%
% Output: 3x1 vector
%   newTorque = adjusted torque where momentum is dumped
%*************************************************************************

% Maximum dipole strength
max_dpl = 2;% A*m^2

% 3x3 reaction wheel inertia matrix
J_rw = rwInertiaMatrix(); 

% reaction wheel angular momentum
h_rw = J_rw*w_rw;

newTorque = max_dpl*(cross(h_rw,mag_body)/(norm(cross(h_rw, mag_body))))
end

