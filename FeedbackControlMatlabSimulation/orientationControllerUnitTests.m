clear all

R_BI = [6.6341e-01  1.0504e-01  7.4084e-01
        3.8302e-01  8.0287e-01  -4.5683e-01
        -6.4279e-01 5.8682e-01  4.9240e-01]';
c_I = [-3; 2; 3];
c_I = c_I/norm(c_I);
s_I = [4; 1; -5];
s_I = s_I/norm(s_I);
w = [0; 0; 0];
w_rw = [0; 0; 0];
[z_err, n_err] = error_twovector(R_BI, c_I, s_I)
err = z_err + n_err;
dt = 1;

% pause

% Orientation Controller Tests
% Test 1
wdot_desired = largeErrorController(w, z_err, dt, 1)
[wdot_rw, pwm] = Alpha2RW_PWM(w, wdot_desired, w_rw, dt)

% Test 2
wdot_desired = largeErrorController(w, z_err, dt, 0)
[wdot_rw, pwm] = Alpha2RW_PWM(w, wdot_desired, w_rw, dt)

% Test 3
w = [0.03; -0.015; 0.01];
w_rw = [-30; -150; -190];
dt = 0.1;
wdot_desired = largeErrorController(w, z_err, dt, 0)
[wdot_rw, pwm] = Alpha2RW_PWM(w, wdot_desired, w_rw, dt)

% % Stabilization Controller Tests
% % Test 1
% wdot_desired = stabilizationController(w, w_rw, err, dt, 1)
% [wdot_rw, pwm] = Alpha2RW_PWM(w, wdot_desired, w_rw, dt)
% 
% % Test 2
% wdot_desired = stabilizationController(w, w_rw, err, dt, 0)
% [wdot_rw, pwm] = Alpha2RW_PWM(w, wdot_desired, w_rw, dt)
% 
% % Test 3
% w = [0.08; 0; -0.025];
% w_rw = [110; 20; -45];
% wdot_desired = stabilizationController(w, w_rw, err, dt, 0)
% [wdot_rw, pwm] = Alpha2RW_PWM(w, wdot_desired, w_rw, dt)
% 






% Stabilization Controller Test With Momentum Dumping
% Test 1
w =  [-0.000823412819641066; 0.000823855246784669; 4.35504321779471e-06];
err = [0.007725642184553; 0.008020302029273; 0.000736637852137372];
w_rw = [-99.9985371159399; 48.2929441678575; -100.039367809058];
mag_body =  [6.46573091078488e-05; 6.44556597758570e-05; -2.70540145919345e-05];

[m,wdot_desired, torque_tr] = stabilizationController(w, w_rw,  err, dt, mag_body, 1);
[wdot_rw, pwm] = Alpha2RW_PWM(w, wdot_desired, w_rw, dt);

RW_PMW = pwm
TR_PWM = m*100/2


% Test 2

w= [-0.000823579563934107; 0.000823757048340383; 4.24355780514207e-06];
err =[0.00771186333215223; 0.00805678987845969; 0.000867889782590301];
w_rw = [-100.125053532384;	48.4925570989862;	-99.9018495699234];
mag_body = [6.46798248042214e-05; 6.44776226872199e-05; -2.69476499837976e-05];


[m,wdot_desired, torque_tr] = stabilizationController(w, w_rw, err, dt, mag_body, 0);
[wdot_rw, pwm] = Alpha2RW_PWM(w, wdot_desired, w_rw, dt);

RW_PMW = pwm
TR_PWM = m*100/2


% Test 3
w= [-0.000823349787583247; 0.000823707364659599; 4.16524492954831e-06];
err =[0.00768929875279296;	0.00808645654605530; 0.000995954298261411];
w_rw = [-99.7900939458284;	48.6923354286124;	-100.257260015589];
mag_body = [6.47022465808677e-05; 6.44994967823076e-05; -2.68412668472860e-05];

[m,wdot_desired, torque_tr] = stabilizationController(w, w_rw, err, dt, mag_body, 0);
[wdot_rw, pwm] = Alpha2RW_PWM(w, wdot_desired, w_rw, dt);

RW_PMW = pwm
TR_PWM = m*100/2






 
