% 
%
% Created by Galen Savidge, 3/17/2019

clc
clear all
close all

% SETTINGS
steps = 5; % How many steps to run for (each is 1 second)

% READ INPUT FILE
M = csvread('rwSpeeds.csv', 2, 0);

% Find first line with a nonzero PWM
start_line = 1;
while M(start_line, 4) == 0
    start_line = start_line + 1;
end

% Separate 42 data into arrays
theoretical_speeds = M(start_line:start_line+steps, 1);
pwms = M(start_line:start_line+steps, 4);
brake = M(start_line:start_line+steps, 7);
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

% MAIN LOOP
for i = t
    % Write PWM to microcontroller
    if(brake)
        fprintf(s, '%6.2fb\n', pwms(i))
    else
        fprintf(s, '%6.2f\n', pwms(i))
    end
    
    % Read speed from microcontroller
    measured_speeds(i) = fscanf(s, '%f\n');
    
    measured_speeds(i)
    
    % Wait 1s
    pause(1)
end


% PLOT RESULTS
figure(1)
hold on
grid on
plot(t, theoretical_speeds)
plot(t, measured_speeds)
title('Measured vs. Theoretical Motor Speeds')
xlabel('Time (s)')
ylabel('Speed (rad/s)')
legend('Thoeretical', 'Measured')