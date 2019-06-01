% Measures reaction wheel speed and compares it to speed from 42
% 
% To use, run alongside RW_Demo. Filename should be the path to a reaction 
% wheel speed log from 42.
% 
% Created by Galen Savidge, 5/30/2019

clc
clear all
close all

% SETTINGS
steps = 60; % How many steps to run for (each is 1 second)
filename = 'rwSpeeds.csv';
wheel = 0; % Which reaction wheel to use (0 -> X, 1 -> Y, 2 -> Z)

% READ INPUT FILE
M = csvread(filename, wheel+2, 0);

% Find first line with a nonzero PWM
start_line = 1;
while M(start_line, wheel+4) == 0
    start_line = start_line + 1;
end

% Separate 42 data into arrays
theoretical_speeds = M(start_line:start_line+steps, wheel+1);
pwms = M(start_line:start_line+steps, wheel+4);
brake = M(start_line:start_line+steps, wheel+7);
t = 1:length(pwms); % Timesteps

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

measured_speeds = zeros(length(pwms), 1);

% Stop reaction wheel
fprintf(s, '%6.2fb\n', 100)

% MAIN LOOP
for i = t
    pwm = pwms(i)
    b = brake(i)
    
    % Write PWM to microcontroller
    if brake(i) == 1
        fprintf(s, '%6.2fb\n', pwms(i))
    else
        fprintf(s, '%6.2f\n', pwms(i))
    end
    
    % Read speed from microcontroller
    measured_speeds(i) = fscanf(s, '%f\n');
    
    speed = measured_speeds(i)
    
    % Wait 1s
    pause(1)
end

% Stop reaction wheel
fprintf(s, '%6.2fb\n', 100)

% PLOT RESULTS
figure(1)
hold on
grid on
plot(t, abs(theoretical_speeds))
plot(t, measured_speeds)
title('Measured vs. Theoretical Motor Speeds')
xlabel('Time (s)')
ylabel('Speed (rad/s)')
legend('Thoeretical', 'Measured')

% Close all ports
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end