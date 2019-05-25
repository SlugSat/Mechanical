%> @brief bodyInertia will return a 3x3 inertia matrix for our crafts body derived 
%> from the solidworks model. (includes antennas)
%>  
%> @ param NONE
%>
%> @ retval out: 3x3 inertia matrix
%**************************************************************************
 
function out = bodyInertia()
   
out = [   0.01151603           0                0;
            0           0.10225145          0;
            0                0        0.09528751];
end
    
