function error = error_deltaR(R_BI,c_I,s_I)
%error_3_axis returns the error vector
%   Detailed explanation goes here

R_BI_des = desiredRotation(c_I, s_I);

delta_R = R_BI_des * R_BI';
errorM = delta_R - eye(3);
error = -[(errorM(3,2)-errorM(2,3))/2;(errorM(1,3)-errorM(3,1))/2;(errorM(2,1)-errorM(1,2))/2];
end

