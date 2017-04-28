function K = calculate_kalman_gain_1d(P)
%assume initial state to be zero
theta = 0;
theta_dot = 0;
DC = 0;
x_pre = [theta_dot];
%inital kalamn parameters
u = DC;
dt = 0.01;
R = 1/12;
H = 1;
I = 1;
P_pre = 1;
sigma = 0.9;
Q = dt * sigma^2;
Full_acc = 7.2083e+03;
%% calculate
S = H*P*H'+R;
K=P*H*S^-1;
