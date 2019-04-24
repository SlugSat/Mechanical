function Rw = rwInertiaMatrix()
% rwInertiaMatrix will return a 3x3 inertia matrix for each reaction wheel
% on each axis of the body frame.
% Inputs:
%   density = density of flywheel in kg/meters^2
%   height = height of flywheel in meters
%   radius = radius of flywheel in meters

d=2700; % density of aluminum in kg/meters^3
r=0.01265; % radius of reaction wheel in meters
h=0.005;  % height of reaction wheel in meters 

% m is the mass of the flywheel
m = pi*r^2*h*d;
        
Rw = [(0.5*m*r^2)     0          0;
           0     (0.5*m*r^2)     0;
           0          0     (0.5*m*r^2)];
end
    