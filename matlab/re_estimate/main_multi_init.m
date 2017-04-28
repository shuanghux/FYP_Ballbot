function main_multi_init()
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

F_init = linspace(0.1,2,3);
H_init = linspace(0.1,2,3);
B_init = logspace(-0.01,-0.0001,3);
Q_init = logspace(0.001,0.1,3);
R_init = logspace(0.001,0.1,3);
temp_LL = 0;
tempF=0;tempH=0;tempB=0;tempQ=0;tempR=0;
count = 0;
for a = 1:3
    for b = 1:3
        for ct = 1:3
            for d = 1:3
                for e = 1:3
                    F = 1; H=0.7; B = -0.5; Q = 0.1; R = 0.01;
                    num_iter = 10000;
                    xhat1i = [];
                    for i = 1:num_iter
                        [F,H,B,Q,R,pi1,V1,loglike,num_iter,xhat1i] = recursion(output,input,F,H,B,Q,R,pi1,V1,num_iter,xhat1i);
                        %L_curr = loglike;
                        %converged  = test_converge(L_pre,L_curr);
                        %L_pre = L_curr;
                    end
                    c=H;
                    H=1;
                    Q=Q*c*c;
                    B=B*c;
                    loglike = verifyF(output,input,F,H,B,Q,R,pi1,V1,num_iter,xhat1i);
                    if temp_LL < loglike
                        temp_LL = loglike;
                        tempF = F;tempH = H;tempB = B;tempQ = Q;tempR = R;
                        count = count + 1;
                    end
                    
                end
            end
        end
    end
end
tempF
tempH
tempB
tempQ
tempR
temp_LL
disp('aaaa')
end