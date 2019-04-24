%% Magnetic Field Model (Tilted dipole)
%Beta1 + wot = angle between the vernal equinox and the instantaneous line
%of intersection of the equatorial plane with the geomagnetic equator
%with Beta1 being its initial value.
%Reference: https://www.aero.iitb.ac.in/~hbhablani/Papers_for_Homepage/JGCD_1995_NovDec_MagControllers.pdf

%Orbital Parameters

% rc = Orbit Radius (km)
% Torb = Orbital period% 
% Mt = Magnetic moment earth (T*km^3)% 
% wo = orbital speed% 
% t = time% 
% wot = true anomoly measured from the ascending node line% 
% In = Inclination angle (radians)% 
% Gta = Geomagnetic tilt angle (radians)% 
% Beta1 = Initial value% 
% kmg = Mt/rc^3 %Dipole magnitude


function b=magField(t)
%Inputs
%t: time


%Orbital parameters
rc=4350; %Orbit radius (km)
Mt=7.8379e6; %Magnetic moment Earth T*km^3
Torb=5855; %Orbital period (s)
Gta=11.44*pi/180; %Geomagnetic tilt angle (rad)
In=51.6*pi/180; %Inclination (rad)
kmg=Mt/rc^3; %Dipole Magnitude (T)
wo=2*pi/Torb; %Orbital speed (rad/s)
we=7.2921150e-5; %Earth rotation speed (rad/s)
Beta1= we*t;
wot = wo*t;

cosepsm=cos(In)*cos(Gta)+sin(In)*sin(Gta)*cos(Beta1);

nim=atan2(-sin(Gta)*sin(Beta1),sin(In)*cos(Gta)-...
cos(In)*sin(Gta)*cos(Beta1));

if sin(nim)==0;

    sinepsm=sin(In)*cos(Gta)-cos(In)*sin(Gta)*cos(Beta1)/cos(nim);
else
    
    sinepsm=-sin(Gta)*sin(Beta1)/sin(nim);
end

b=kmg*[sinepsm*cos(wot-nim);-cosepsm;2*sinepsm*sin(wot-nim)];
end


