% Automates motor friction testing using RW_Demo
%
% Created by Galen Savidge, 5/30/2019

clc
clear all
close all

% SETTINGS
pwm_step_size = 10;
wait_time = 5; % Time to wait for speed to stabilize (seconds)
v_rail = 8;
K = 0.00713332454; % Motor constant (Kt = Ke)
R = 92.7; % Ohms


% SET UP UART
% Close all ports
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

% Open microcontroller UART port
s_ports = seriallist; % Change this if the script chooses the wrong port
s = serial(s_ports(length(s_ports)), 'BaudRate', 9600)
fopen(s)
flushinput(s)

measured_speeds = [];


% MAIN LOOP
for pwm = pwm_step_size:pwm_step_size:100
    % Write PWM to microcontroller
    if(brake)
        fprintf(s, '%6.2fb\n', pwms(i))
    else
        fprintf(s, '%6.2f\n', pwms(i))
    end
    
    % Read speed from microcontroller
    measured_speeds(i) = fscanf(s, '%f\n');
    
    measured_speeds(i)
    
    % Wait for speed to stabilize
    pause(wait_time)
end


% GET TORQUE MODEL FROM MOTOR DYNAMICS
v = pwm(2:end)*v_rail;
w = measured_speed(2:end);
trq = (K/R)*(v - w*K);
p = polyfit(w, trq, 1) % Find torque as a linear function of motor speed

% Evaluate linear model
wfunc = 0:1:1000;
trqfunc = polyval(c, wfunc);


% PLOT RESULTS
figure(1)
hold on
grid on
scatter(w, trq, 'bo')
plot(wfunc, trqfunc, 'r--')
title('Reaction Wheel Friction Torque Model')
xlabel('\omega (rad/s)')
ylabel('Friction (Nm)')