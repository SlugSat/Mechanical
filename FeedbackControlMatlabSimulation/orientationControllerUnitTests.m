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

pause

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

% Stabilization Controller Tests
% Test 1
wdot_desired = stabilizationController(w, w_rw, err, dt, 1)
[wdot_rw, pwm] = Alpha2RW_PWM(w, wdot_desired, w_rw, dt)

% Test 2
wdot_desired = stabilizationController(w, w_rw, err, dt, 0)
[wdot_rw, pwm] = Alpha2RW_PWM(w, wdot_desired, w_rw, dt)

% Test 3
w = [0.08; 0; -0.025];
w_rw = [110; 20; -45];
wdot_desired = stabilizationController(w, w_rw, err, dt, 0)
[wdot_rw, pwm] = Alpha2RW_PWM(w, wdot_desired, w_rw, dt)