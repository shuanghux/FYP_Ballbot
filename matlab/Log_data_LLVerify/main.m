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


%better parameter is F = 1; H = 1; B = -0.002; Q = 0.1; R = 1.38e-12; pi1 = 0; V1 = 0.1;
%However in order to show the optimization process we choose relatively
%worse initial states.

[F,H,B,Q,R,pi1,V1] = set_initial(output);
xhat1i = [];
Flog = zeros(1,2000);
Hlog = zeros(1,2000);
Blog = zeros(1,2000);
Qlog = zeros(1,2000);
Rlog = zeros(1,2000);
Fm = 0.001:0.001:2.00;
Hm = linspace(0,20000,2000);
Bm = linspace(-0.01,0.01,2000);
Qm = logspace(-3,2,2000);
Rm = logspace(-50,-6,2000);
num_iter = 10;
hold on;

for n = 1:num_iter
    for i = 1:1:2000
    Flog(i) = verifyF(output,input,Fm(i),H,B,Q,R,pi1,V1,num_iter,xhat1i);
    Hlog(i) = verifyF(output,input,F,Hm(i),B,Q,R,pi1,V1,num_iter,xhat1i);
    Blog(i) = verifyF(output,input,F,H,Bm(i),Q,R,pi1,V1,num_iter,xhat1i);
    Qlog(i) = verifyF(output,input,F,H,B,Qm(i),R,pi1,V1,num_iter,xhat1i);
    Rlog(i) = verifyF(output,input,F,H,B,Q,Rm(i),pi1,V1,num_iter,xhat1i);
    end
    maxF = max(Flog);
    maxH = max(Hlog);
    maxB = max(Blog);
    maxQ = max(Qlog);
    maxR = max(Rlog);
    max_vec = [maxF maxH maxB maxQ maxR];
    maxlog = min(max_vec);
    figure(1);
    hold on;
    subplot(2,3,1); plot(Fm,Flog);title('F');
    figure(1);
    hold on;
    subplot(2,3,2); plot(Hm,Hlog);title('H');
    figure(1);
    hold on;
    subplot(2,3,3); plot(Bm,Blog);title('B');
    figure(1);
    hold on;
    subplot(2,3,4); loglog(Qm,Qlog);title('Q');
    figure(1);
    hold on;
    subplot(2,3,5); loglog(Rm,Rlog);title('R');
    figure(1);
    hold on;
    subplot(2,3,6); plot(n,maxlog,'r*');title(['Maxloglike ' num2str(n) ' iterations)']);
    disp([num2str(n) 'iteration']);
    [F,H,B,Q,R,pi1,V1,loglike,num_iter,xhat1i] = recursion(output,input,F,H,B,Q,R,pi1,V1,num_iter,xhat1i);
    plot(n,loglike,'b*');
end

hold off
end