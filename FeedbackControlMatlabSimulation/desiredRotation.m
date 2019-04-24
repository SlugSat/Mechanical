function R_BIdes = desiredRotation(c_I,s_I)
%desiredRotation returns a  desired 3x3 rotation matrix from body to inertial 
%using the triad method. Zbody vector will be aligned with the c vector.
%Inputs:
%   c_I = a 3x1 vector representing the position of the craft from the earth
%       in the inertial frame.
%   s_I = a 3x1 vector representing the position of the sun from the earth in
%       inertial frame.

   z_BI = c_I/norm(c_I);
   x_BI = (rcross(z_BI)*s_I)/norm(rcross(z_BI)*s_I);
   y_BI = rcross(z_BI)*x_BI;
   
   R_BI = [x_BI';y_BI';z_BI'];
   
   %still need rotate 45 degrees about the z_BI (c_I) axis
   R_BIdes = R_BI;
   
end

