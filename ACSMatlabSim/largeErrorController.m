function wdot_desired = largeErrorController(w, err, dt, first_step)
% Feedback controller for large errors i.e. >20 degrees
% Inputs:
%   w: Craft angular velocity vector (rad/s)
%   w_rw: Reaction wheel angular velocity vector (rad/s)
%   err: Error between craft rotation and desired rotation
%   last_err: Error from previous timestep
%   dt: Time since last step (seconds)
%   first_step: 1 on the first step that the controller is called on,
%   otherwise 0
% Output:
%   wdot_desired: the desired angular acceleration
%**************************************************************************

% Feedback constants
K = 1;
Kp = K*0.05;
Kd = K*0;

persistent last_w_err
w_desired = -0.017*err/norm(err); % 1 deg/s in rad/s
w_err = w - w_desired;

if first_step
    wdot_desired = [0; 0; 0];
else
    wdot_desired = -(Kp*w_err + Kd*(w_err - last_w_err)/dt);
end

last_w_err = w_err;
end