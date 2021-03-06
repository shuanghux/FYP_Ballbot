function main_3D()


load('control.mat');
input = vect;
F = 1; H=1; B=-0.002; Q=1e-6; R=1e-3;
output = sim_para(F,H,B,Q,R,input);
[F,H,B,Q,R,pi1,V1] = set_initial(output);
%load('measure.mat');
%output = vect;
clear vect;
input = input(1,1:end);
output = output(1,1:end);
loglikeBQ = zeros(100,100);
B1 = getB(input,output);
Hm = linspace(0,2,300);
Bm = linspace(-0.005,0,100);
Qm = linspace(3*10^-6,10^-7,100);
num_iter = 10;
xhat1i = [];
%{
for x = 1:300
    for y = 1:300
        for z = 1:300
            loglike3D(x,y,z) = verifyF(output,input,F,Hm(x),Bm(y),Qm(z),R,pi1,V1,num_iter,xhat1i);
        end
    end
end
%}

figure;
%% Here we set H to 1 and log relationship between B and Q
Bm = linspace(-0.01,0,100);
Qm = linspace(8.8*10^-6,9.0*10^-6,100);
for x = 1:100
    for y = 1:100
        loglikeBQ(x,y) = verifyF(output,input,F,1,Bm(x),Qm(y),R,pi1,V1,num_iter,xhat1i);
    end
end
figure(1);
hold on;
%subplot(1,3,1);
%contour(Bm,Qm,loglikeBQ);
%surf(Bm,Qm,loglikeBQ);
shading interp
title('varing B/Q')
xlabel('B')
ylabel('Q')

%% Here we set B to -0.002 and log relationship between H and Q
Qm = linspace(2.9*10^-5,3*10^-4,100);
Hm = linspace(0.1,3,100);
loglikeHQ = zeros(100,100);
for x = 1:100
    for y = 1:100
        loglikeHQ(x,y) = verifyF(output,input,F,Hm(x),-0.002,Qm(y),R,pi1,V1,num_iter,xhat1i);
    end
end
figure(1);
hold on;
%subplot(1,3,2);
%imagesc(Hm,Qm,loglikeHQ)
%contour(Hm,Qm,loglikeHQ);
%surf(Hm,Qm,loglikeHQ);
shading interp
title('varing H/Q')
xlabel('H')
ylabel('Q')

%% Here we set Q to 1e^-6 and log relationship between H and B

loglikeHB = zeros(100,100);
for x = 1:100
    for y = 1:100
        loglikeHB(x,y) = verifyF(output,input,F,Hm(x),Bm(y),1e-6,R,pi1,V1,num_iter,xhat1i);
    end
end
figure(1);
hold on;
%subplot(1,3,3);
contour(Hm,Bm,loglikeHB);
%surf(Hm,Bm,loglikeHB);
title('varing H/B')
shading interp
xlabel('H')
ylabel('B')

end