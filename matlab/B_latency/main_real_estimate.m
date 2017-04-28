function main_real_estimate()

load('control.mat');
input = vect;
load('measure.mat');
output = vect;
clear vect;
input = input(1,1:end);
output = output(1,1:end);
for i = 1:size(input,2)
    if input(i) > 100
        input(i) = 100;
    elseif input(i) < -100
        input(i) = -100;
    end
end
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