function R = rotx(x)
R = [1               0                  0;
    0   cos(deg2rad(x))  -sin(deg2rad(x));
    0   sin(deg2rad(x))   cos(deg2rad(x))];