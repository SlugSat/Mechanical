%> @brief Returns a rotation matrix x degrees about the Z axis
%>
%> Input:
%> @param x: amount of rotation in degrees
%>
%> Output:
%> @retval R: 3x3 rotation matrix about Z axis
%**************************************************************************

function R = rotz(x)

R = [cos(deg2rad(x))  -sin(deg2rad(x))    0;
     sin(deg2rad(x))   cos(deg2rad(x))    0;
                   0                 0    1];