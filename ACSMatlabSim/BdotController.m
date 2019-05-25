%> @file: BdotController_light.m
%> @brief: Light version of the Bdot control simulation. Outputs angular 
%>           velocity plott on each axis
%**************************************************************************
clear all
close all
clc

%Max dipole moment of torque rod(A*m^2)
mmax=2;

%Inertial body
J = diag([0.60579, 0.01330, 0.59753]);

%Simulation parameters
simulation_time = 800;% Amount of time to be simulated (seconds)
dt = 0.5; % Time between steps (seconds)
numSteps = simulation_time/dt;
R = rotx(90)*roty(160)*rotz(-30); % Initial craft DCM
w = [0.06; 0.06 ; 0.06]; % Angular velocity vector                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
t = 0; % Current time

%Initialize history
t_hist = zeros(numSteps, 1);
m_hist = zeros(numSteps, 3);
brot_hist = zeros(numSteps, 3);
bdot_hist = zeros(numSteps, 3);
b_hist = zeros(numSteps, 3);
w_hist = zeros(numSteps, 3);
wdot_hist = zeros(numSteps, 3);
bold = [0;0;0];

% Disturbance torque vector in inertial frame
disturbance_mag = 60e-6; % In Nm (orbital disturbance = ~60 uNm)
disturbance_vec = [1;1;1];
disturbance_vec = disturbance_mag*disturbance_vec/norm(disturbance_vec);
disttorq = disturbance_vec;

A = eye(3); % 3x3 identity matrix

for i=1:numSteps
    %Update inertial vectors
    craft_angle = 2*pi*t/6000; % Full orbit every 10min for now
    c_I = [sin(craft_angle); cos(craft_angle); 0];
    
    %Earth's magnetic field 
    b = R * magField(t); %Magnetic field vector in body frame   
    
    %Rotational rate of B
    brot1 = ((b-bold))/dt; %Rotational rate of B
    brot = cross(bold, b);
    bold = b; %Saves last value
  
    %Analytic solution, rotational rate of the magnetic field minus
    %rotational rate of the satellite
    bdot = cross((brot-w), b); %Analytic bdot
     
    % Feedback controller    
    %Bang-Bang Bdot
    m = -mmax *(bdot/norm(bdot));
    
    % Simulate the craft's kinematics      
	wdot = J \ (cross(m,b)- cross(w,J*w) + (R'*disttorq)); %derivative of w
    
    if norm(w) > 0.001
        w = w + wdot*dt; %Integrate wdot to get angular velocity
    end
    R = R*Rexp(w*dt); %Integrate R to the attitude    
    
    %Advance time
    t = t + dt;    
    
	%History
    m_hist(i,:) = m;
    t_hist(i) = t;
    w_hist(i,:) = w;
    wdot_hist(i,:) = wdot;
    bdot_hist(i,:)= bdot;
    b_hist(i,:)= b;
    brot_hist(i,:)= brot;
      
   
end

% Plot angular velocity over time
figure(1)
hold on
grid on
title('Angular Velocity On Each Axis')
ylabel('\omega (rad/s)')
xlabel('Time (s)')
plot(t_hist, w_hist(:,1), 'r')
plot(t_hist, w_hist(:,2), 'b')
plot(t_hist, w_hist(:,3), 'k')
set(gcf,'Color','w');
legend('x', 'y', 'z')
