%%Bdot control test
clear all
close all
clc

%Torque rod
mmax=2.0; %Maximum magnetic moment (A*m^2)

%Inertial body frame
J = diag([0.33,0.37,0.35]);

%Initialize
bold = [0;0;0];

set(gcf,'Color','w');

%Rotate a 10x10x20 cube centered at (0,0,0) with a rotation matrix
%Define cube dimensions
xwidth = .20;
ywidth = .10; 
zwidth = .10;

faceColor = [1.0000    0.3492    0.6508];

% Define eight cube vertices
v1 = [-1*xwidth/2;   ywidth/2;        zwidth/2];
v2 = [xwidth/2;      ywidth/2;        zwidth/2];
v3 = [xwidth/2;      -1*ywidth/2;     zwidth/2];
v4 = [-1*xwidth/2;   -1*ywidth/2;     zwidth/2];
v5 = [-1*xwidth/2;   -1*ywidth/2;     -1*zwidth/2];
v6 = [xwidth/2;      -1*ywidth/2;     -1*zwidth/2];
v7 = [xwidth/2;      ywidth/2;        -1*zwidth/2];
v8 = [-1*xwidth/2;   ywidth/2;        -1*zwidth/2];

simulation_time = 1000;% Amount of time to be simulated (seconds)
dt = 0.5; % Time between steps (seconds)
numSteps = simulation_time/dt;

% Inertial vectors
c_I = [0;1;0]; %position of the craft wrt to inertial
s_I = [1;0;0]; %position of the sun wrt to inertial

R = rotx(90)*roty(160)*rotz(-30); % Initial craft DCM
w = [0.08; -0.08; 0.07]; % Angular velocity vector
bdot_i = [0; 0; 0]; % Integrator                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
t = 0; % Current time

% Disturbance torque vector in inertial frame
disturbance_mag = 60e-6; % In Nm (orbital disturbance = ~60 uNm)
disturbance_vec = [1;1;1];
disturbance_vec = disturbance_mag*disturbance_vec/norm(disturbance_vec)
disttorq = disturbance_vec;

% Initialize last error for derivative
last_err = error_twovector(R, c_I, s_I);
% last_err = error_deltaR(R, c_I, s_I);

% Initialize quiver plot
figure(1)
set(gcf,'Color','w');
[sX,sY,sZ]=sphere(30);
surf(sX,sY,sZ,'FaceAlpha',.1,'EdgeColor','none');
axis equal;
title('Craft Orientation Over Time')
xlabel('x')
ylabel('y')
zlabel('z')
hold on

R_BI_des = desiredRotation(c_I, s_I);

% Update quiver plot
figure(1)
hold on
xhat_I = R_BI_des*[1.25;0;0];
yhat_I = R_BI_des*[0;1.25;0];
zhat_I = R_BI_des*[0;0;1.25];
quiver3([0,0,0],[0,0,0],[0,0,0],[xhat_I(1),yhat_I(1),zhat_I(1)],[xhat_I(2),yhat_I(2),zhat_I(2)],[xhat_I(3),yhat_I(3),zhat_I(3)])

% Initialize error history
err_hist = zeros(numSteps, 3);
%pwm_hist = zeros(numSteps, 3);
t_hist = zeros(numSteps, 1);
Mcmd_hist = zeros(numSteps, 3);
bdot_hist = zeros(numSteps, 3);
b_hist = zeros(numSteps, 3);
w_hist = zeros(numSteps, 3);
wdot_hist = zeros(numSteps, 3);


for i=1:numSteps
    %Update inertial vectors
    craft_angle = 2*pi*t/6000; % Full orbit every 10min for now
    c_I = [sin(craft_angle); cos(craft_angle); 0];
    
    % % Find error
    err = error_twovector(R, c_I, s_I);
    % % err = error_deltaR(R, c_I, s_I);
    err_hist(i,:) = err;
	
    %Earth's magnetic field
    b = R * magField(t); %Magnetic field vector in body frame   
   
    %Rotational rate of B
    brot = ((b-bold))/dt; %Rotational rate of B
    bold = b; %Saves last value
    
    %Analytic solution, rotational rate of the magnetic field minus
    %rotational rate of the satellite
    bdot = cross( (brot - w), b); %Analytic bdot
    
    % Feedback controller    
    %Bang-Bang Bdot
    m = -mmax * sign(bdot);
    
    %Option 2: Using gain calculated from attitude
    %m=getMC1(kw/norm(b)^2,bdot1,mmax);
	      
    % Simulate the craft's kinematics
	wdot = J \ ( ( disttorq - cross(m,b)) - cross(w, J*w) ); %derivative of w  
	w = w + wdot*dt; %Integrate wdot to get angular velocity
	R = R*Rexp(w*dt); %Integrate R to the attitude    
    
    %Advance time
    t = t + dt;    
    
	%History
    m_hist(i,:) = m;
    t_hist(i) = t;
    w_hist(i,:) = w;
    wdot_hist(i,:) = wdot;
    bdot_hist(i,:)= bdot;
    brot_hist(i,:)= brot;
    b_hist(i,:)= b;
    
    % Plot cube
    figure(4)        
    
    % Update quiver plot
    figure(1)
    hold on
    xhat_I = R*[1;0;0];
    yhat_I = R*[0;1;0];
    zhat_I = R*[0;0;1];
    quiver3([0,0,0],[0,0,0],[0,0,0],[xhat_I(1),yhat_I(1),zhat_I(1)],[xhat_I(2),yhat_I(2),zhat_I(2)],[xhat_I(3),yhat_I(3),zhat_I(3)])
    
    % pause()
end

% Plot error over time
figure(2)
hold on
grid on
title('Error On Each Axis')
ylabel('Error (rad/s^2)')
xlabel('Time (s)')
plot(t_hist, err_hist(:,1), 'r')
plot(t_hist, err_hist(:,2), 'b')
plot(t_hist, err_hist(:,3), 'k')
legend('x', 'y', 'z')

% Plot pwm over time
figure(3)
hold on
grid on
title('PWM On Each Axis')
ylabel('PWM')
xlabel('Time (s)')
plot(t_hist, pwm_hist(:,1), 'r')
plot(t_hist, pwm_hist(:,2), 'b')
plot(t_hist, pwm_hist(:,3), 'k')
legend('x', 'y', 'z')