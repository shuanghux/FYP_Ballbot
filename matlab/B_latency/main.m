function main()
load('control.mat');
input = vect;
T = size(input,2);
F = 1; H=1; Q=0.1; R=0.4;
B = [-0.001 -0.001 -0.002];
u = input;
U_vec = zeros(3,T);
U_vec(:,1)=[0;0;u(1)];
U_vec(:,2)=[0;u(1);u(2)];
for t = 3:T
    U_vec(:,t) = [u(t-2);u(t-1);u(t)];
end
output = sim_para(F,H,B,Q,R,U_vec);
%load('measure.mat');
%output = vect;
clear vect;
input = input(1,1:end);
output = output(1,1:end);

[F,H,B_vec,Q,R,pi1,V1] = set_initial(output);
num_iter = 30000;
xhat1i = [];
for i = 1:num_iter
    [F,H,B_vec,Q,R,pi1,V1,loglike,num_iter,xhat1i] = recursion(output,input,F,H,B_vec,Q,R,pi1,V1,num_iter,xhat1i);
    %L_curr = loglike;
    %converged  = test_converge(L_pre,L_curr);
    %L_pre = L_curr;
end
F
H
B_vec
Q
R

end