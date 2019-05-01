function [wdot_desired, torque_tr] = stabilizationController(w, w_rw, err, dt, mag_body, bdot, first_step)
% Feedback controller for small errors
% Inputs:
%   w: Craft angular velocity vector (rad/s)
%   w_rw: Reaction wheel angular velocity vector (rad/s)
%   err: Error between craft rotation and desired rotation
%   dt: Time since last step (seconds)
%   mag_body: Magnetic field in body frame
%   bdot: bang bang bdot
%   first_step: 1 on the first step that the controller is called on,
%   otherwise 0
% Outputs:
%   wdot_desired: Desired change in angular velocity
%   torque_tr: torque to be exerted by torque rods when dumping momentum

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
    torque_tr = [0; 0; 0];
    
else
        
% Momentum dumping    
    %Determine available torque
    torque_tr = momentum_dump(w_rw, mag_body, bdot);
    
    %Use angular velocity of each axis to see if momentum dumping is needed
    
    %X-axis
     if (w_rw(1) >100)
         torque_tr(1) = torque_tr(1);
     else 
         torque_tr(1) = 0;
     end
         
     %Y-Axis
     if (w_rw(2) > 100)
         torque_tr(2) = torque_tr(2);
     else 
         torque_tr(2) = 0;
     end
         
     %Z-axis    
     if (w_rw(3) > 100)
         torque_tr(3) = torque_tr(3);
     else 
         torque_tr(3) = 0;
     end
         
    %Calculate reaction wheel torque    
    torque_integrator = torque_integrator + Ki_t*err*dt;
    controller_torque = Kp_t*err + torque_integrator + (Kd_t*(err - last_err)/dt) + torque_tr; % Subtract torque rod torque
    %Calculate wdot desired
    wdot_desired = -(Kp_wdot*err + Kd_wdot*(err - last_err)/dt);
    wdot_desired = wdot_desired + torque2wdot(w, w_rw, controller_torque);
    
     
    end        
    
last_err = err;
     
end