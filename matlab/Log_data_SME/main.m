function main()

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

[F,H,B,Q,R,pi1,V1] = set_initial(output);

xhat1i = [];
num_iter = 50000;
converged = 0;
L_pre = 0; L_curr = 0;
loglk = zeros(1,num_iter);
n = 1:num_iter;
for i = 1:num_iter
    [F,H,B,Q,R,pi1,V1,loglike,num_iter,xhat1i] = recursion(output,input,F,H,B,Q,R,pi1,V1,num_iter,xhat1i);
    %L_curr = loglike;
    %converged  = test_converge(L_pre,L_curr);
    %L_pre = L_curr;
    loglk(i) = loglike;
end
plot(n,loglk,'b-');
num_iter
F
H
B
Q
R

end