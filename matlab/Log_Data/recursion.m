function [B,Q,R,pi1,V1,loglike,num_iter,xhat1i] = recursion(measure,input,B,Q,R,pi1,V1,num_iter,xhat1i)
% takes num_iter before this loop and return iteration taken after this loop
% This is function that runs the forward kalman filtering to get first
% estimation of detected values
% load data with simpler notation
y = measure;
u = input;
T = size(measure,2);


%% initialize
xtt=zeros(1,T); Vtt=zeros(1,T); xtt1=zeros(1,T); Vtt1=zeros(1,T);
xtT=zeros(1,T); VtT=zeros(1,T); J=zeros(1,T); Vtt1T=zeros(1,T);
% where tt1 denotes _t^(t-1) or (inter)

%% Forward Kalman Recursion %Equation 1

for t = 1:T
    if t == 1
        Vtt(1) = V1;
        xtt1(1) = pi1;
    else
        xtt1(t) = xtt(t-1) + B*u(t-1); %starts from x_2^1
        Vtt1(t) = Vtt(t-1) + Q;
    end
    Kt = Vtt1(t)/(Vtt1(t)+R);
    xtt(t) = xtt1(t) + Kt*(y(t)-xtt1(t));
    Vtt(t) = Vtt1(t)-Kt*Vtt1(t);
end
KT = Kt;

%% Backward Rauch Recursion %Equation 2
xtT(T) = xtt(T);
VtT(T) = Vtt(T);
for t = T:-1:2
    J(t-1) = Vtt(t-1)/Vtt1(t);
    xtT(t-1) = xtt(t-1) + J(t-1)*(xtT(t)-(xtt(t-1)+B*u(t-1)));
    VtT(t-1) = Vtt(t-1) + J(t-1)*(VtT(t)-Vtt1(t))*J(t-1);
end

%% Another Recursion
xhat = xtT;
Pt = VtT + xtT.^2; %size 1*T
Vtt1T(T) = (1-KT)*Vtt(T-1);
for t = T:-1:3
   Vtt1T(t-1) = Vtt(t-1)*J(t-2) + J(t-1)*(Vtt1T(t)-Vtt(t-1))*J(t-2);  %Equation 3
end
%Ptt1=[NaN Vtt1T(2:T)+xtT(2:T).*xtT(1:(T-1))];
Ptt1 = [0 (Vtt1T(2:T)+xtT(2:T).*xtT(1:T-1))]; %Equation 4
%loglikelihood
loglike = -sum((y-xhat).^2)/(2*R) - T/2*log(abs(R))...
    -sum((xhat(2:T)-xhat(1:T-1)-B*u(1:T-1)).^2)/(2*Q) - (T-1)/2*log(abs(Q))...
    -sum((xhat(1)-pi1).^2)/(2*V1) - 0.5*log(abs(V1)) - T*log(2*pi);

%% Update Parameters
% B and Q derived by alex Blocker
%B = (sum(  xhat(2:T) .* u(2:T)  )...
%    - (sum(Ptt1(2:T)) / sum(Pt(1:T)) * sum(xhat(1:T-1).*u(2:T)))  )...
%    /( sum(u(2:t).^2)...
%    - ( sum(u(2:T) .* sum(xhat(1:T-1))) / sum(Pt(1:T)) * sum(xhat(1:T-1) .* u(2:T))));

%Q = 1/(T-1)*sum(Pt(2:T) - Ptt1(2:T) - B*u(2:T).*xhat(2:T));

%new version of B and Q
B = sum((xhat(2:T).*u(1:T-1)-xhat(1:T-1).*u(1:T-1))/(u(1:T-1).^2));

Q = 1/(T-1)*sum(Pt(2:T)+Pt(1:T-1)-2*Ptt1(2:T)+2*B*xhat(1:T-1).*u(1:T-1)+B*B*u(1:T-1).*u(1:T-1));

R = 1/T*sum(y.*y - xhat.*y);
pi1 = xhat(1);


% initial state vector
N = num_iter + 1;
xhat1i = [xhat1i xhat(1)];
xhatbar1 = xhatbar(xhat,N,1);
V1 = Pt(1) - xhat(1)^2 + 1/N*sum((xhat1i - xhatbar1).^2);
num_iter = N;
V1 = 0.1; % Otherwise V1 is always 0 so loglike becomes infinite
end