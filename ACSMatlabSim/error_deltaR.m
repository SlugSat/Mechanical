function error = error_deltaR(R_BI,c_I,s_I)
%Returns error between current and desires body inertial vectors
%Inputs:
%   c_I: a 3x1 vector representing the position of the craft from the earth
%       in the inertial frame.
%   s_I: a 3x1 vector representing the position of the sun from the earth in
%       inertial frame.
%   R_BI: Inerial rotation matrix
%Outputs:
%   error: error between current and desired inertial vectors.

R_BI_des = desiredRotation(c_I, s_I);

delta_R = R_BI_des * R_BI';
errorM = delta_R - eye(3);
error = -[(errorM(3,2)-errorM(2,3))/2;(errorM(1,3)-errorM(3,1))/2;(errorM(2,1)-errorM(1,2))/2];
end

