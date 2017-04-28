function loglike = verifyF(measure,input,F,H,B,Q,R,pi1,V1,num_iter,xhat1i)
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
        Vtt1(1) = V1;
        xtt1(1) = pi1;
    else
        xtt1(t) = F*xtt(t-1) + B*u(t-1); %starts from x_2^1
        Vtt1(t) = F*Vtt(t-1)*F + Q;
    end
    Kt = Vtt1(t)*H/(H*Vtt1(t)*H+R);
    xtt(t) = xtt1(t) + Kt*(y(t)-H*xtt1(t));
    Vtt(t) = Vtt1(t)-Kt*H*Vtt1(t);
end
KT = Kt;

%% Backward Rauch Recursion %Equation 2
xtT(T) = xtt(T);
VtT(T) = Vtt(T);
for t = T:-1:2
    J(t-1) = Vtt(t-1)*F/Vtt1(t);
    xtT(t-1) = xtt(t-1) + J(t-1)*(xtT(t)-F*xtt(t-1)-B*u(t-1));
    VtT(t-1) = Vtt(t-1) + J(t-1)*(VtT(t)-Vtt1(t))*J(t-1);
end

%% Another Recursion
xhat = xtT;
accuracy = test_ratio(xhat,y);
Pt = VtT + xhat.^2; %size 1*T
Vtt1T(T) = (1-KT*H)*F*Vtt(T-1);
for t = T:-1:3
   Vtt1T(t-1) = Vtt(t-1)*J(t-2) + J(t-1)*(Vtt1T(t)-F*Vtt(t-1))*J(t-2);  %Equation 3
end
for t = T:-1:2
   Ptt1(t) = Vtt1T(t) + xhat(t)*xhat(t-1); 
end
%Ptt1=[NaN Vtt1T(2:T)+xtT(2:T).*xtT(1:(T-1))];
%Equation 4
%loglikelihood

loglike = -sum((y-H*xhat).^2)/(2*R) - T/2*log(abs(R))...
    -sum((xhat(2:T)-F*xhat(1:T-1)-B*u(1:T-1)).^2)/(2*Q) - (T-1)/2*log(abs(Q))...
    -sum((xhat(1)-pi1).^2)/(2*V1) - 0.5*log(abs(V1)) - T*log(2*pi);
cost_by_H = test_sum(H,xhat,y)/R/2;

end