function m = bdotControl(w, b, bold, mmax)
% Calculates analytical Bdot from change in magnetic field
%
% Inputs:
%   w: craft's angular velocity (rad/s)
%   b: magnetic field in body frame (mT)
%   bold: old mag field 
%   mmax: maximum dipole moments (A*m^2)
%
% Outputs:
%   m: bdot
%**************************************************************************

%Max dipole moment
mmax = 2.0;

%Rotation on B field in body frame
brot = cross(bold, b);
bold = b;     

%Analytic solution, rotational rate of the magnetic field minus
%rotational rate of the satellite
bdot = cross( (brot - w), b); %Analytic bdot

%Bang-Bang Bdot
m = -mmax*sign(bdot);
end 