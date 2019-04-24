function out = bodyInertiaMatrix(mass,hieght,length,width)
% InertiaMatrix will return a 3x3 inertia matrix for a rectangular prism
% Inputs:
%   mass = mass of prism in kg
%   height = hieght of prism in meters
%   length = length of prism in meters
%   width = width of prism in meters

m = mass;
h = hieght;
l = length;
w = width;


out = [m/12*(l^2+h^2)      0            0;
         0           m/12*(w^2+h^2)     0;
         0                 0       m/12*(l^2+w^2)];
     
out = [   0.60579            0                0;
            0              0.01330            0;
            0                0         0.59753];
end
    
