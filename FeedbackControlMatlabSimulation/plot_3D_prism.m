function plot_3D_prism(v1, v2, v3, v4, v5, v6,v7, v8, R, alphaVal, color)

% Author: Natalie King (narking) 
% Plots and rotates a rectangular prism from 8 vertices
% R -- the rotation matrix
%      if R == 0, no rotation is applied
% alpha -- the transparency value 
%          0 (fully transparent) to 1 (fully opaque)
% color -- color value or matrix

if (R == 0) 
    % Fill all 6 faces
    r1=fill3([v1(1) v2(1) v3(1) v4(1)],[v1(2) v2(2) v3(2) v4(2)],[v1(3) v2(3) v3(3) v4(3)] ,color,'facealpha',0.6); %1 2 3 4
    r2=fill3([v1(1) v2(1) v7(1) v8(1)],[v1(2) v2(2) v7(2) v8(2)],[v1(3) v2(3) v7(3) v8(3)] ,color,'facealpha',0.6); %1 2 7 8
    r3=fill3([v5(1) v6(1) v7(1) v8(1)],[v5(2) v6(2) v7(2) v8(2)],[v5(3) v6(3) v7(3) v8(3)] ,color,'facealpha',0.6); %5 6 7 8
    r4=fill3([v5(1) v6(1) v3(1) v4(1)],[v5(2) v6(2) v3(2) v4(2)],[v5(3) v6(3) v3(3) v4(3)] ,color,'facealpha',0.6); %5 6 3 4
    r5=fill3([v2(1) v3(1) v6(1) v7(1)],[v2(2) v3(2) v6(2) v7(2)],[v2(3) v3(3) v6(3) v7(3)] ,color,'facealpha',0.6); %2 3 6 7
    r6=fill3([v1(1) v4(1) v5(1) v8(1)],[v1(2) v4(2) v5(2) v8(2)],[v1(3) v4(3) v5(3) v8(3)] ,color,'facealpha',0.6); %1 4 5 8
else
    % Apply rotation
    v1r = R*v1;
    v2r = R*v2;
    v3r = R*v3;
    v4r = R*v4;
    v5r = R*v5;
    v6r = R*v6;
    v7r = R*v7;
    v8r = R*v8;
    
    % Fill all 6 faces
    hold on
    r1=fill3([v1r(1) v2r(1) v3r(1) v4r(1)],[v1r(2) v2r(2) v3r(2) v4r(2)],[v1r(3) v2r(3) v3r(3) v4r(3)] ,color,'facealpha',0.6); %1 2 3 4
    r2=fill3([v1r(1) v2r(1) v7r(1) v8r(1)],[v1r(2) v2r(2) v7r(2) v8r(2)],[v1r(3) v2r(3) v7r(3) v8r(3)] ,color,'facealpha',0.6); %1 2 7 8
    r3=fill3([v5r(1) v6r(1) v7r(1) v8r(1)],[v5r(2) v6r(2) v7r(2) v8r(2)],[v5r(3) v6r(3) v7r(3) v8r(3)] ,color,'facealpha',0.6); %5 6 7 8
    r4=fill3([v5r(1) v6r(1) v3r(1) v4r(1)],[v5r(2) v6r(2) v3r(2) v4r(2)],[v5r(3) v6r(3) v3r(3) v4r(3)] ,color,'facealpha',0.6); %5 6 3 4
    r5=fill3([v2r(1) v3r(1) v6r(1) v7r(1)],[v2r(2) v3r(2) v6r(2) v7r(2)],[v2r(3) v3r(3) v6r(3) v7r(3)] ,color,'facealpha',0.6); %2 3 6 7
    r6=fill3([v1r(1) v4r(1) v5r(1) v8r(1)],[v1r(2) v4r(2) v5r(2) v8r(2)],[v1r(3) v4r(3) v5r(3) v8r(3)] ,color,'facealpha',0.6); %1 4 5 8
    hold off
end

% Set face transparency
% alpha(r1,alphaVal);
% alpha(r2,alphaVal);
% alpha(r3,alphaVal);
% alpha(r4,alphaVal);
% alpha(r5,alphaVal);
% alpha(r6,alphaVal);

end