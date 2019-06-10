% Automates motor friction testing using RW_Demo
%
% Created by Galen Savidge, 5/30/2019

clc
clear all
close all

% SETTINGS
pwm_step_size = 2;
max_pwm = 60;
wait_time = 15; % Time to wait for speed to stabilize (seconds)
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


% MAIN LOOP
for i = 2:max_pwm/pwm_step_size
    % Write PWM to microcontroller
    pwms(i) = i*pwm_step_size;
    fprintf(s, '%6.2f\n', pwms(i))
    fscanf(s, '%f\n'); % Clear speed from serial buffer
    
    % Wait for speed to stabilize
    pause(wait_time)
    
    % Read speed from microcontroller
    fprintf(s, '%6.2f\n', pwms(i))
    measured_speeds(i) = fscanf(s, '%f\n');
    
    measured_speeds(i)
end

% Stop the motor
fprintf(s, '%6.2fb\n', 100)


% GET TORQUE MODEL FROM MOTOR DYNAMICS
v = pwms(2:end)*v_rail/100;
w = measured_speeds(2:end);
trq = (K/R)*(v - w*K);
p = polyfit(w, trq, 1) % Find torque as a linear function of motor speed

% Evaluate linear model
wfunc = 0:1:100*ceil(max(w)/100);
trqfunc = polyval(p, wfunc);


% PLOT RESULTS
figure(1)
hold on
grid on
scatter(v, w, 'o')
title('Reaction Wheel Speed vs. Voltage')
xlabel('Motor Voltage (V)')
ylabel('Motor Speed (rad/s)')

figure(2)
hold on
grid on
scatter(w, trq, 'bo')
plot(wfunc, trqfunc, 'r--')
title('Reaction Wheel Friction Torque Model')
xlabel('\omega (rad/s)')
ylabel('Friction (Nm)')

% Close all ports
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end