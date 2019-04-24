% Test script for 3-axis feedback control.



clear all
close all

set(gcf,'Color','w');

numSteps = 100;

% Feedback constants
Kp = 0.1;
Kd = 0.2;

%inertial vectors
c_I = [0;1;0]; %position of the craft wrt to inertial
s_I = [1;0;0]; %position of the sun wrt to inertial

maxTorque = [25*10^-3   0         0;
               0      25*10^-3    0;
               0        0       25*10^-3];
Jsc = bodyInertiaMatrix(3,.2,.1,.1); %inertia of the space craft

R_BI_curr = eye(3); % DCM
R_BI_des = desiredRotation(c_I, s_I);
%R_BI_des = rotz(5)*roty(5)*rotx(5);%R_BI_des';
w = [0; 0; 0]; % Angular velocity vector                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
dt = 0.1; % Time between steps
t = 0; % Time

delta_R = R_BI_des * R_BI_curr';
errorM = delta_R - eye(3);
last_err = [errorM(3,2);errorM(1,3);errorM(2,1)];

figure(2)
set(gcf,'Color','w');
[sX,sY,sZ]=sphere(30);
surf(sX,sY,sZ,'FaceAlpha',.1,'EdgeColor','none');
axis equal;
title('20cm')
xlabel('x')
ylabel('y')
zlabel('z')
hold on
    
x = R_BI_des * [1;0;0];
y = R_BI_des * [0;1;0];
z = R_BI_des * [0;0;1];

quiver3([0,0,0],[0,0,0],[0,0,0],[x(1),y(1),z(1)],[x(2),y(2),z(2)],[x(3),y(3),z(3)])
    

i = 0; % Iteration
for i=1:numSteps
    % Find error
    %xhat_I = R_BI_curr*xhat_B;
    %err = rcross(R_BI_curr'*xhat_I_des)*xhat_B;
    delta_R = R_BI_des * R_BI_curr';
    errorM = delta_R - eye(3);
    err = [errorM(3,2);errorM(1,3);errorM(2,1)];
    
    % Feedback controller
    pwm = Kp*err + Kd*(err - last_err)/dt;
    
    torque = maxTorque*pwm;
    
    wdot = Jsc\torque;
    
    last_err = err;
    
    w = w + wdot*dt; % Integrate to get angular velocity
    R_BI_curr=R_BI_curr*Rexp(w) % Integrate w to get angular position
    R_BI_des = R_BI_des
    
    t = t + dt;
    i = i + 1;
    
    
    % Plot things
    figure(1)
    hold on
    title('Error On Each Axis')
    ylabel('Error')
    xlabel('Time')
    plot(t, err(1), 'y*')
    plot(t, err(2), 'c*')
    plot(t, err(3), 'm*')
    legend('x', 'y', 'z')
    
    x = R_BI_curr * [1;0;0];
    y = R_BI_curr * [0;1;0];
    z = R_BI_curr * [0;0;1];
    
    %Plots xhat_I to the desired xhat_I_des
    figure(2)
    hold on
    quiver3([0,0,0],[0,0,0],[0,0,0],[x(1),y(1),z(1)],[x(2),y(2),z(2)],[x(3),y(3),z(3)])
    
    pause(.1)
end