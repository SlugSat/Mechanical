%> @brief Feedback controller for small errors
%>
%> Inputs:
%> @param  w:          Craft angular velocity vector (rad/s)
%> @param  w_rw:       Reaction wheel angular velocity vector (rad/s)
%> @param  err:        Error between craft rotation and desired rotation
%> @param  dt:         Time since last step (seconds)
%> @param  mag_body:   Magnetic field in body frame
%> @param  first_step: 1 on the first step that the controller is called on,
%>                     otherwise   0
%>
%> Outputs:
%> @retval  wdot_desired:   Desired change in angular velocity
%> @retval  torque_tr:      torque to be exerted by torque rods when dumping 
%>                          momentum
%**************************************************************************

function [m,wdot_desired, torque_tr] = stabilizationController(w, w_rw, err, dt, mag_body, first_step)

% Feedback constants
% Angular speed portion
K_wdot =  -0.3;
Kp_wdot = K_wdot*0.006;
Kd_wdot = K_wdot*0.4;

% Torque portion
K_t = 5e-4;
Kp_t = K_t*1.5;
Ki_t = K_t*0.05;
Kd_t = K_t*8;

persistent torque_integrator last_err

    if first_step
    torque_integrator = [0; 0; 0];
    wdot_desired = [0; 0; 0];
    torque_tr = [0; 0; 0];
    m = [0,0,0];
    
else
        
%Momentum dumping    
    %Determine available torque
    [m,torque_tr] = momentum_dump(w_rw, mag_body);
    
    %Use angular velocity of each axis to see if momentum dumping is needed
    for i=1:3
     if (abs(w_rw(i)) > 100)
         torque_tr(i) = torque_tr(i);
     else 
         torque_tr(i) = 0;
     end
    end
    %Calculate reaction wheel torque
    P = Kp_t*err;
    torque_integrator = torque_integrator + Ki_t*err*dt;
    controller_torque = Kp_t*err + torque_integrator;
    controller_torque = Kp_t*err + torque_integrator + (Kd_t*(err - last_err)/dt) + torque_tr; % Subtract torque rod torque
    %Calculate wdot desired
    wdot_desired = (Kp_wdot*err + Kd_wdot*(err - last_err)/dt);
    wdot_desired = wdot_desired + torque2wdot(w, w_rw, controller_torque);
    end        
    
last_err = err;
     
end