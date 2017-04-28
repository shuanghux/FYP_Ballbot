function main_mul_veri()
load('control.mat');
input = vect;
F = 1; H=1; B=-0.002; Q=0.1; R=1e-2;
output = sim_para(F,H,B,Q,R,input);
%load('measure.mat');
%output = vect;
clear vect;
input = input(1,1:end);
output = output(1,1:end);

[F,H,B,Q,R,pi1,V1] = set_initial(output);

Fv = [0.6 0.6 0.6 0.6];
Bv = [-0.002 -0.01 -0.01 -0.01];
Qv = [0.1 0.1 0.3 0.3];
Rv = [0.01 0.01 0.01 0.5];
num_iter = 40;
xhat1i = [];
log_v = zeros(4,20);
for test = 1:4
   F = Fv(test);B = Bv(test);Q = Qv(test);R = Rv(test);
   for i = 1:20
    [F,H,B,Q,R,pi1,V1,loglike,num_iter,xhat1i] = recursion(output,input,F,H,B,Q,R,pi1,V1,num_iter,xhat1i);
    %L_curr = loglike;
    %converged  = test_converge(L_pre,L_curr);
    %L_pre = L_curr;
    log_v(test,i) = loglike;
   end 
end
%{
c=H;
H=1;
Q=Q*c*c;
B=B*c;
%}
count_iter = 1:20;
plot(count_iter,log_v(1,:),'o-');
hold on
plot(count_iter,log_v(2,:),'o-');
plot(count_iter,log_v(3,:),'o-');
plot(count_iter,log_v(4,:),'o-');
legend('1','2','3','4');

F
H
B
Q
R
verifyF(output,input,F,H,B,Q,R,pi1,V1,num_iter,xhat1i)
disp('aaaa')
end