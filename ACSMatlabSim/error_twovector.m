function [z_err, n_err] = error_twovector(R_BI, c_I, s_I)
% Error determination function using two vectors
% Instead of using a desired DCM, this function finds error as a sum of 
% error between craft inertial and desired craft inertial vectors, and 
% error between sun inertial and desired sun inertial vectors.
% Inputs:
%   c_I = a 3x1 vector representing the position of the craft from the 
%       Earth in the inertial frame.
%   s_I = a 3x1 vector representing the position of the sun from the Earth
%       in the inertial frame.
%   R_BI: Inerial rotation matrix
%Outputs:
%   z_err:
%   n_err:

zhat_B = [0; 0; 1];
xhat_B = [1; 0; 0];

c_I = c_I/norm(c_I);
s_I = s_I/norm(s_I);


c_B = R_BI'*c_I; % Earth to craft vector in body frame

% Find z error
z_err = rcross(c_B)*zhat_B/2;

% z_err_mag = sin(acos(dot(c_B, zhat_B))/2);
% z_err = z_err_mag*z_err/norm(z_err); % Magnitude of z error

% Cross product of craft vector and sun vector: returns normalized vector 
% normal to the plane containing the craft, sun, and Earth
n_I = rcross(c_I)*s_I;
n_I = n_I/norm(n_I);
n_B = R_BI'*n_I;

corner_B = [sign(n_B(1)); sign(n_B(2)); 0];
corner_B = corner_B/norm(corner_B);

n_err = rcross(n_B)*corner_B/2;

% x_err_mag = sin(acos(dot(n_B, xhat_B))/2);
% x_err = x_err_mag*x_err/norm(x_err); % Magnitude of x error

end