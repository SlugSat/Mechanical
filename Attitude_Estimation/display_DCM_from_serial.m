% This script reads DCMs from USB UART and displays them as a 3D object
% onscreen.
%
% Created by Galen Savidge, 3/17/2019

clear all

% SET UP UART
% Close all ports
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

% Open microcontroller UART port
s_ports = seriallist; % Change this if the script chooses the wrong port
s = serial(s_ports(length(s_ports)), 'BaudRate', 115200)
fopen(s)

% CUBE SETUP
% Rotate a 10x10x20 cube centered at (0,0,0) with a rotation matrix
% Define cube dimensions
xwidth = .10;
ywidth = .10; 
zwidth = .20;

faceColor = [1.0000    0.3492    0.6508];
quiverColor = 'b';

% Define eight cube vertices
v1 = [-1*xwidth/2;   ywidth/2;        zwidth/2];
v2 = [xwidth/2;      ywidth/2;        zwidth/2];
v3 = [xwidth/2;      -1*ywidth/2;     zwidth/2];
v4 = [-1*xwidth/2;   -1*ywidth/2;     zwidth/2];
v5 = [-1*xwidth/2;   -1*ywidth/2;     -1*zwidth/2];
v6 = [xwidth/2;      -1*ywidth/2;     -1*zwidth/2];
v7 = [xwidth/2;      ywidth/2;        -1*zwidth/2];
v8 = [-1*xwidth/2;   ywidth/2;        -1*zwidth/2];

% Set up plot
title('Estimated Attitude from UART')
xlabel('x_I [m]')
ylabel('y_I [m]')
zlabel('z_I [m]')
set(gcf,'Color','w')

% Body frame vectors
xhat_b = [.125; 0; 0];
yhat_b = [0; .125; 0];
zhat_b = [0; 0; .125];

% MAIN LOOP
while 1
    R = zeros(3);
    i = 1;
    
%     flushinput(s)
%     
%     % Wait for beginning of DCM
%     DCM_vector = fscanf(s, '%f\t%f\t%f\t');
%     while ~isempty(DCM_vector)
%         DCM_vector = fscanf(s, '%f\t%f\t%f\t');
%     end
    
    % Read DCM
    while i <= 3
        DCM_vector = fscanf(s, '%f\t%f\t%f\t');
        if(length(DCM_vector) == 3)
            R(i,1:3) = DCM_vector;
            i = i + 1;
        else
            i = 1;
        end
    end
    
    % R = R % Print DCM
    
    % Clear graph and reformat axis
    cla
    axis equal
    xlim([-.15 .15])
    ylim([-.15 .15])
    zlim([-.15 .15])
    axis on
    grid on
    hold on
    view([60, 30])
    
    % Draw the cube
    plot_3D_prism(v1, v2, v3, v4, v5, v6, v7, v8, R, 1, faceColor)
    
    % Draw body frame axes
    xhat_i = R*xhat_b;
    yhat_i = R*yhat_b;
    zhat_i = R*zhat_b;
    q = quiver3([0,0,0],[0,0,0],[0,0,0],[xhat_i(1),yhat_i(1),zhat_i(1)],[xhat_i(2),yhat_i(2),zhat_i(2)],[xhat_i(3),yhat_i(3),zhat_i(3)],0);
    q.Color = quiverColor;
    hold off
    
    pause(.0001);
end