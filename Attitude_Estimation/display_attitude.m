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
flushinput(s)

% CUBE SETUP
% Rotate a 10x10x20 cube centered at (0,0,0) with a rotation matrix
% Define cube dimensions
xwidth = .10;
ywidth = .10; 
zwidth = .20;

faceColor = [1.0000    0.3492    0.6508];
bodyQuiverColor = 'b';
inertialQuiverColor = 'g';

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
set(gcf,'Color', 'w') % Set background color
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.15, 0.10, 0.60 0.90]) % Size window

% Body frame vectors
xhat_b = [.125; 0; 0];
yhat_b = [0; .125; 0];
zhat_b = [0; 0; .125];

% MAIN LOOP
while 1
    R = zeros(3);
    i = 1;
    
    % Prevents accumulating time delay when Matlab reads slower than the
    % microcontroller sends data
    flushinput(s)
    
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
    clf('reset')
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
    q.Color = bodyQuiverColor;
    
    % Draw body axis labels
    xtextpos = xhat_i*1.02;
    ytextpos = yhat_i*1.02;
    ztextpos = zhat_i*1.02;
    text(xtextpos(1), xtextpos(2), xtextpos(3), 'x', 'FontSize', 16, 'Color', bodyQuiverColor)
    text(ytextpos(1), ytextpos(2), ytextpos(3), 'y', 'FontSize', 16, 'Color', bodyQuiverColor)
    text(ztextpos(1), ztextpos(2), ztextpos(3), 'z', 'FontSize', 16, 'Color', bodyQuiverColor)
    
    % Draw inertial frame axes
    q = quiver3([0,0,0],[0,0,0],[0,0,0],[xhat_b(1),yhat_b(1),zhat_b(1)],[xhat_b(2),yhat_b(2),zhat_b(2)],[xhat_b(3),yhat_b(3),zhat_b(3)],0);
    q.Color = inertialQuiverColor;
    
    % Draw body axis labels
    xtextpos = xhat_b*1.02;
    ytextpos = yhat_b*1.02;
    ztextpos = zhat_b*1.02;
    text(xtextpos(1), xtextpos(2), xtextpos(3), 'x', 'FontSize', 16, 'Color', inertialQuiverColor)
    text(ytextpos(1), ytextpos(2), ytextpos(3), 'y', 'FontSize', 16, 'Color', inertialQuiverColor)
    text(ztextpos(1), ztextpos(2), ztextpos(3), 'z', 'FontSize', 16, 'Color', inertialQuiverColor)
    
    % Draw legend
    legend([{'Body'},{'Inertial'}])
    
    hold off
    pause(.00001);
end