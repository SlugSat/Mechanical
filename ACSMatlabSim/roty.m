function R = roty(x)
% Returns a rotation matrix x degrees about the Y axis
%**************************************************************************
R = [cos(deg2rad(x))     0      sin(deg2rad(x));
    0                    1                    0;
    -sin(deg2rad(x))     0      cos(deg2rad(x))];