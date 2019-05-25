%> @file twovector_control_simulation.m
%> @brief Test script for 3-axis feedback control. Drives the Z+ unit vector in
%> the body frame to c_I, the craft position vector in the inertial frame.
%> Also drives a corner of the craft to s_I, the sun position in the
%> interial frame.
%
%> Created by Galen Savidge, 2/23/2019
%> Edited by Gabriel Barbosa
%**************************************************************************

clear all
clc
close all

% SIMULATION SETTINGS
simulation_time = 600; % Amount of time to be simulated (seconds)
dt = 1; % Time between steps (seconds)
draw_cube = 0; % Set to 1 to render the cube and quiver plot
orbit_time = 5400; % Craft orbital period (seconds); 0 for static position
R = rotx(60)*roty(-160)*rotz(150); % Initial craft DCM
w = [0; 0; 0]; % Initial angular velocity
w_rw = [0; 0; 0]; % Initial reaction wheel angular velocity vector
mag_i = magField(1); % Initial mag readings
mag_body  = R'*mag_i;
torque_tr = [0;0;0];

% Disturbance torque vector in inertial frame
use_42_disturbance = 0;
if use_42_disturbance == 0
    disturbance_mag = 5e-6; % In Nm (orbital disturbance = ~60 uNm)
    disturbance_vec = [1;1;1];
    disturbance_vec = disturbance_mag*disturbance_vec/norm(disturbance_vec);
    %disturbance_vec = envtorque(1)';
end

% GIF SETTINGS
make_gif = 0; % Set to 1 to output into the file names set below
cube_filename = 'cube.gif';
quiver_filename = 'quiver.gif';
gif_timescale = 20; % Speed multiplier (e.g. 10 -> gif is at 10x speed)

% ---------- End settings ----------

% SIMULATION SETUP
num_steps = simulation_time/dt;
t = 0; % Current time
initial_step = 1; % Don't change this
acs_state = 0; % 0: large angles; 1: small angles
set(gcf, 'Color', 'w');

% Inertial vectors
c_I = [0;0;1]; % Initial position of the craft wrt to inertial
s_I = [0;-1;0]; % Position of the sun wrt to inertial

% Initialize last error for derivative
last_err = error_twovector(R, c_I, s_I);

% Initialize bdot
bold = [0;0;0];

% Set up cube and quiver plots
if draw_cube
    % Define cube dimensions
    xwidth = 1;
    ywidth = 1;
    zwidth = 2;

    faceColor = [1.0000    0.3492    0.6508];
    
    % Define eight cube vertices
    v1 = [-1*xwidth/2;    ywidth/2;        zwidth/2];
    v2 = [xwidth/2;      ywidth/2;        zwidth/2];
    v3 = [xwidth/2;      -1*ywidth/2;     zwidth/2];
    v4 = [-1*xwidth/2;   -1*ywidth/2;     zwidth/2];
    v5 = [-1*xwidth/2;   -1*ywidth/2;     -1*zwidth/2];
    v6 = [xwidth/2;      -1*ywidth/2;     -1*zwidth/2];
    v7 = [xwidth/2;      ywidth/2;        -1*zwidth/2];
    v8 = [-1*xwidth/2;   ywidth/2;        -1*zwidth/2];

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
    xhat_I = R_BI_des*[1.25;0;0];
    yhat_I = R_BI_des*[0;1.25;0];
    zhat_I = R_BI_des*[0;0;1.25];
    qDesired = quiver3([0,0,0],[0,0,0],[0,0,0],[xhat_I(1),yhat_I(1),zhat_I(1)],[xhat_I(2),yhat_I(2),zhat_I(2)],[xhat_I(3),yhat_I(3),zhat_I(3)]);
    qDesired.LineWidth = 1;

    xhat_Ides = xhat_I;
    yhat_Ides = yhat_I;
    zhat_Ides = zhat_I;
end

% Set up gif capture
if draw_cube == 0
    make_gif = 0;
end
if make_gif == 1
    frame_time = dt/gif_timescale;
end

% Initialize history
t_hist = zeros(num_steps, 1);
err_hist = zeros(num_steps, 3);
pwm_hist = zeros(num_steps, 3);
w_hist = zeros(num_steps, 3);
w_rwhist = zeros(num_steps, 3);
torque_trhist = zeros(num_steps, 3);
mag_bodyhist = zeros(num_steps, 3);
mhist = zeros(num_steps, 3);
bdot_hist = zeros(num_steps, 3);

transition_times = [];


% RUN SIMULATION
for i=1:num_steps    
    % Update inertial vectors if applicable
    if orbit_time > 0
        craft_angle = 2*pi*t/orbit_time;
        c_I = [sin(craft_angle); cos(craft_angle); 0];
    end
    
    % Find error between current and desired orientation
    [z_err, n_err] = error_twovector(R, c_I, s_I);
    err = z_err + n_err;
    
    % Run ACS state machine logic and feedback controllers
    if acs_state == 0
        if norm(z_err) < deg2rad(5) % About 5 degrees in radians
            acs_state = 1;
            initial_step = 1;
            transition_times = [transition_times t];
%             if draw_cube
%                 pause
%             end
        else
            controller_wdot = largeErrorController(w, z_err, dt, initial_step);
            torque_tr = [0; 0; 0];
            initial_step = 0;
        end
    else
        if norm(z_err) > deg2rad(10) % About 10 degrees in radians
            acs_state = 0;
            initial_step = 1;
            transition_times = [transition_times t];
%             if draw_cube
%                 pause
%             end
        else
            % Bang Bang bdot for momentum dumping
            bdot = bdotControl(w, mag_body, bold);
            bdot_hist(i,:) = bdot;
            % Stabilization control
            [m , controller_wdot, torque_tr] = stabilizationController(w, w_rw, err, dt, mag_body, initial_step);
            initial_step = 0;
            mhist(i,:) = m;
            trPWM_hist(i,:) = m*100/2;
        end
    end
    
    err_hist(i,:) = 180*err/pi;
    
    % Find PWM as a function of angular rates of the satellite, reaction
    % wheels, desired angular acceleration, and dt
    w_rw_old = w_rw;
    [w_rw, pwm] = wdot2RW_PWM(w, controller_wdot, w_rw, dt);
    pwm_hist(i,:) = pwm;
   
    
    % Simulate the craft's dyanamics
    w_rw_dot = (w_rw - w_rw_old)/dt;
    if use_42_disturbance
        disturbance_vec = envtorque(floor(t)+1)';
    end
    torque = rwInertia*w_rw_dot + R'*disturbance_vec -torque_tr; % Add torque rod torque here
    
    torque_trhist(i,:) = torque_tr;
    
    
    w_dot = torque2wdot(w, w_rw, torque);
    w = w + w_dot*dt; % Integrate wdot to get angular velocity
    w_hist(i,:) = w;
    R = R*Rexp(w*dt); % Integrate w to get attitude
    mag_body  = R'*mag_i;
    t = t + dt;
    t_hist(i) = t;
    mag_bodyhist(i,:)= mag_body;
    
    
     w_rwhist(i,:) = w_rw;
    
    if draw_cube
        % Update quiver plot
        figure(1)
        hold on
        xhat_I = R*[1;0;0];
        yhat_I = R*[0;1;0];
        zhat_I = R*[0;0;1];
        quiver3([0,0,0],[0,0,0],[0,0,0],[xhat_I(1),yhat_I(1),zhat_I(1)],[xhat_I(2),yhat_I(2),zhat_I(2)],[xhat_I(3),yhat_I(3),zhat_I(3)])
        % movegui(1,'north');

        % Plot cube
        figure(2)
        clf(2)
        plot_3D_prism(v1, v2, v3, v4, v5, v6,v7, v8, R, 1, faceColor)
        hold on
        q = quiver3([0,0,0],[0,0,0],[0,0,0],[xhat_I(1),yhat_I(1),zhat_I(1)]*1.5,[xhat_I(2),yhat_I(2),zhat_I(2)]*1.5,[xhat_I(3),yhat_I(3),zhat_I(3)]*1.5);
        qdes = quiver3([0,0,0],[0,0,0],[0,0,0],[xhat_Ides(1),yhat_Ides(1),zhat_Ides(1)],[xhat_Ides(2),yhat_Ides(2),zhat_Ides(2)],[xhat_Ides(3),yhat_Ides(3),zhat_Ides(3)]);
        q.Color = 'b';
        qdes.Color = 'g';
        qdes.AutoScale = 'on';
        axis equal
        title('Craft Orientation Using Feedback Controller')
        xlabel('x')
        ylabel('y')
        zlabel('z')
        xlim([-1.5 1.5])
        ylim([-1.5 1.5])
        zlim([-1.5 1.5])
        axis on
        grid on
        set(gcf,'Color','w');
        view(3)
    end
    
    % Capture gifs
    if make_gif == 1
        % Quiver gif
        f = figure(1);
        frame = getframe(f);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        if i == 1
            imwrite(imind, cm, quiver_filename, 'gif', 'Loopcount', inf);
        else
            imwrite(imind, cm, quiver_filename, 'gif', 'WriteMode', 'append', 'DelayTime', frame_time);
        end
        
        % Cube gif
        f = figure(2);
        frame = getframe(f);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        if i == 1
            imwrite(imind, cm, cube_filename, 'gif', 'Loopcount', inf);
        else
            imwrite(imind, cm, cube_filename, 'gif', 'WriteMode', 'append', 'DelayTime', frame_time);
        end
    end
end

% Print list of state transition times
transition_times %#ok<NOPTS>

% PLOT DATA
% Plot error over time
figure(3)
hold on
grid on
title('Output of Two Vector Error Determination Function')
ylabel('Error (degrees)')
xlabel('Time (s)')
plot(t_hist, err_hist(:,1), 'r')
plot(t_hist, err_hist(:,2), 'b')
plot(t_hist, err_hist(:,3), 'k')
set(gcf,'Color','w');
legend('x', 'y', 'z')
movegui(3,'northwest');

% Plot pwm over time
figure(4)
hold on
grid on
title('PWM On Each Axis')
ylabel('PWM')
xlabel('Time (s)')
plot(t_hist, pwm_hist(:,1), 'r')
plot(t_hist, pwm_hist(:,2), 'b')
plot(t_hist, pwm_hist(:,3), 'k')
set(gcf,'Color','w');
legend('x', 'y', 'z')
movegui(4,'northeast');

% Plot angular velocity over time
figure(5)
hold on
grid on
title('CubeSat Angular Velocity On Each Axis')
ylabel('\omega (rad/s)')
xlabel('Time (s)')
plot(t_hist, w_hist(:,1), 'r')
plot(t_hist, w_hist(:,2), 'b')
plot(t_hist, w_hist(:,3), 'k')
set(gcf,'Color','w');
legend('x', 'y', 'z')
movegui(5,'north');

% Plot angular velocity over time
figure(6)
hold on
grid on
title('Reaction Wheel Angular Velocity On Each Axis')
ylabel('\omega (rad/s)')
xlabel('Time (s)')
plot(t_hist, w_rwhist(:,1), 'r')
plot(t_hist, w_rwhist(:,2), 'b')
plot(t_hist, w_rwhist(:,3), 'k')
set(gcf,'Color','w');
legend('x', 'y', 'z')
movegui(5,'north');

% Plot transition times on the plots
if ~isempty(transition_times)
    for i = 3:6
        figure(i)
        hold on
        vline(transition_times)
    end
end