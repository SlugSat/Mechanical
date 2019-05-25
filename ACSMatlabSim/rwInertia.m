%> rwInertiaMatrix will return a 3x3 inertia matrix for each reaction wheel
%> on each axis of the body frame.
%>
%> Inputs:
%> @param NONE
%>
%> Output:
%> @retval Rw: inertia matrix of the reaction wheels
%**************************************************************************

function Rw = rwInertia()

d = 8700;   % density of brass in kg/meters^3
r = 0.010;  % radius of reaction wheel in meters
h = 0.010;  % height of reaction wheel in meters 

% m is the mass of the flywheel
m = pi*r^2*h*d;
        
Rw = [(0.5*m*r^2)     0          0;
           0     (0.5*m*r^2)     0;
           0          0     (0.5*m*r^2)];
end
    