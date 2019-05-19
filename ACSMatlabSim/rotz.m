function R = rotz(x)
R = [cos(deg2rad(x))  -sin(deg2rad(x))    0;
     sin(deg2rad(x))   cos(deg2rad(x))    0;
                   0                 0    1];