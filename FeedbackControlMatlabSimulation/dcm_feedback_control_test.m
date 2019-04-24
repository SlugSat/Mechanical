% Test script for 3-axis feedback control. Drives the +X unit vector in
% the body frame to a desired position in the intertial frame.
%
% Created by Galen Savidge, 2/23/2019


clear all
close all

% Feedback constants
Kp = 0.1;
Ki = 0;
Kd = 0.2;

R = eye(3); % DCM
w = [0; 0; 0]; % Angular velocity vector
wdot_i = [0; 0; 0]; % Integrator
dt = 0.1; % Time between steps
t = 0; % Time

xhat_B = [2; 1; 0]; % xhat in body frame
xhat_B = xhat_B/norm(xhat_B);
xhat_I_des = [-1; 2; 1]; % Desired xhat direction in inertial frame
xhat_I_des = xhat_I_des/norm(xhat_I_des);

last_err = rcross(R'*xhat_I_des)*xhat_B;

figure(2)
[sX,sY,sZ]=sphere(30);
surf(sX,sY,sZ,'FaceAlpha',.1,'EdgeColor','none');
axis equal;
xlabel('x')
ylabel('y')
zlabel('z')
hold on
    
quiver3([0,0,0],[0,0,0],[0,0,0],[xhat_I_des(1),0,0],[xhat_I_des(2),0,0],[xhat_I_des(3),0,0])
    

i = 0; % Iteration
while 1
    % Find error
    xhat_I = R*xhat_B;
    err = rcross(R'*xhat_I_des)*xhat_B;
    
    % Feedback controller
    wdot_i = wdot_i + Ki*err*dt;
    wdot = -Kp*err - wdot_i - Kd*(err - last_err)/dt;
    
    last_err = err;
    
    w = w + wdot*dt; % Integrate to get angular velocity
    R=R*Rexp(w); % Integrate w to get angular position
    
    t = t + dt;
    i = i + 1;
    
    %xerr(i) = err(1);
    %yerr(i) = err(2);
    %zerr(i) = err(3);
    %time(i) = t;
    
    % Plot things
    figure(1)
    hold on
    title('Error On Each Axis')
    ylabel('Error')
    xlabel('Time')
    plot(t, err(1), 'b*')
    plot(t, err(2), 'g*')
    plot(t, err(3), 'r*')
    legend('x', 'y', 'z')
    
    %Plots xhat_I to the desired xhat_I_des
    figure(2)
    hold on
    quiver3([0,0,0],[0,0,0],[0,0,0],[xhat_I(1),0,0],[xhat_I(2),0,0],[xhat_I(3),0,0])
    
    pause(.1)
end