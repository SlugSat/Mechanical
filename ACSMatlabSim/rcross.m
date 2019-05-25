%> @brief forms the skew symmetric x-product matrix of a 3x1 vector
%>
%> Input:
%> @param r: 3x1 vector
%>
%> Output:
%> @retval rx: skew symmetric matrix of r
%**************************************************************************

function rx = rcross(r)

rx=[0    -r(3)  r(2);
    r(3)  0    -r(1);
   -r(2)  r(1)  0];
end