function main()
load('control.mat');
input = vect;
F = 1; H=1; B=-0.002; Q=0.01; R=0.01;
output = sim_para(F,H,B,Q,R,input);
%load('measure.mat');
%output = vect;
clear vect;
input = input(1,300:end);
output = output(1,300:end);
H_true = H;
[F,H,B,Q,R,pi1,V1] = set_initial(output);
F = 1; H = 2; B = -2; Q = 0.1; R = 0.1;
num_iter = 40000;
xhat1i = [];
loglike = 0;
%B = getB(input,output);
for i = 1:num_iter
    [F,H,B,Q,R,pi1,V1,loglike,num_iter,xhat1i] = recursion(output,input,F,H,B,Q,R,pi1,V1,num_iter,xhat1i);
    %L_curr = loglike;
    %converged  = test_converge(L_pre,L_curr);
    %L_pre = L_curr;
    %A = [F,H,B,Q,R,pi1,V1,loglike];
    %{
    if sum(isnan(A)) ~= 0
        %disp(A);
        pause;
    end
    %}
end
%{
c=H;
H=1;
Q=Q*c*c;
B=B*c;
%}
F
H
B
Q
R
loglike
c = H/H_true;
H = H_true;
Q=Q*c*c;
B=B*c;
verifyF(output,input,F,H,B,Q,R,pi1,V1,num_iter,xhat1i)
 disp('aaaa')
%cH_true = H; c = H/H_true;
end