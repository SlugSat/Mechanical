%> @brief Returns a rotation matrix x degrees about the Y axis
%>
%> Input:
%> @param x: amount of rotation in degrees
%>
%> Output:
%> @retval R: 3x3 rotation matrix about Y axis
%**************************************************************************

function R = roty(x)

R = [cos(deg2rad(x))     0      sin(deg2rad(x));
    0                    1                    0;
    -sin(deg2rad(x))     0      cos(deg2rad(x))];