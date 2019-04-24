function R = roty(x)
R = [cos(deg2rad(x))     0      sin(deg2rad(x));
    0                    1                    0;
    -sin(deg2rad(x))     0      cos(deg2rad(x))];