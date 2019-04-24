function wdot_desired = stabilizationController(w, w_rw, err, dt, first_step)
% Feedback controller for small errors
% Inputs:
%   w: Craft angular velocity vector (rad/s)
%   w_rw: Reaction wheel angular velocity vector (rad/s)
%   err: Error between craft rotation and desired rotation
%   last_err: Error from previous timestep
%   dt: Time since last step (seconds)
%   first_step: 1 on the first step that the controller is called on,
%   otherwise 0

% Feedback constants
% Angular speed portion
K_wdot = 0.3;
Kp_wdot = K_wdot*0.006;
Kd_wdot = K_wdot*0.4;

% Torque portion
K_t = 0.0005;
Kp_t = K_t*1.5;
Ki_t = K_t*0.05;
Kd_t = K_t*8;

persistent torque_integrator last_err

if first_step
    torque_integrator = [0; 0; 0];
    wdot_desired = [0; 0; 0];
else
    % Add momentum dumping here
    % 1. Find torque exerted from torque rods
    torque_integrator = torque_integrator + Ki_t*err*dt;
    controller_torque = Kp_t*err + torque_integrator + Kd_t*(err - last_err)/dt; % Subtract torque rod torque
    wdot_desired = -(Kp_wdot*err + Kd_wdot*(err - last_err)/dt);
    wdot_desired = wdot_desired + torque2wdot(w, w_rw, controller_torque);
    % Return torque rod torque
end

last_err = err;
end