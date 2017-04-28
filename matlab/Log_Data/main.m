function main()

load('control.mat');
input = vect;
load('measure.mat');
output = vect;
clear vect;
for i = 1:size(input,2)
    if input(i) > 100
        input(i) = 100;
    elseif input(i) < -100
        input(i) = -100;
    end
end

[B,Q,R,pi1,V1] = set_initial(output);

xhat1i = [];
num_iter = 0;
converged = 0;
L_pre = 0; L_curr = 0;
while ~converged
    [B,Q,R,pi1,V1,loglike,num_iter,xhat1i] = recursion(output,input,B,Q,R,pi1,V1,num_iter,xhat1i);
    L_curr = loglike;
    converged  = test_converge(L_pre,L_curr);
    L_pre = L_curr;
end
disp('B Q R num_iter:');
disp([B Q R num_iter]);


end