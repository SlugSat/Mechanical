%> @brief Returns a rotation matrix x degrees about the X axis
%>
%> Input:
%> @param x: amount of rotation in degrees
%>
%> Output:
%> @retval R: 3x3 rotation matrix about X axis
%**************************************************************************

function R = rotx(x)

R = [1               0                  0;
    0   cos(deg2rad(x))  -sin(deg2rad(x));
    0   sin(deg2rad(x))   cos(deg2rad(x))];