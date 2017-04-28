function KalmanGain()
@my_matrix_norm_log;
%Notation: 
% _pre: previous state----------t-1 | t-1
% _inter: intermediate step-----t | t-1
% _curr: current state----------t | t
%assume initial state to be zero
theta = 0;
theta_dot = 0;
x_pre = [theta; theta_dot];
%inital kalamn parameters
dt = 0.01;
sigma = 0.4;
F = [1 dt; 0 1];
u = [0;0];
B = [dt^2/2 ; dt];
Q = [dt^3/3 dt^2/2 ; dt^2/2 dt];
R = 1/12;
H = 1;
I = [1 0;0 1];
P_pre = [1 0; 0 1];

%True value
i=0;
TrueValue = [0*i; 0*i];
%Plotting parameters
t = 1:100;
measure = 1:100;
estimate = 1:100;
hold on
optimal_est_x = 1:1:100;
optimal_est_y = 1:1:100;
%Create identicle noise for each sigma
yi=zeros(2,50);
for i_ycreate = 1:50
   yi(:,i_ycreate)= [0;0] +  randn(2,1);
end
%Sigma loop
for count = 1:1:1000
    sigma = count/100;
    temp = [0];
    Q = [dt^3/3 dt^2/2 ; dt^2/2 dt];
    Q = Q*sigma;
    for i=1:50
        %predict
        x_inter = F * x_pre;
        P_inter = F * P_pre * F.' + Q;
        %Update
        K = P_inter * H.' * (H * P_inter * H.' + R)^(-1);
        y = yi(:,i);
        x_curr = x_inter + K * (y - x_inter);
        P_curr = (I - K * H) * P_inter;
        %Plot
        t(1,i) = i;
        measure(1,i) = y(1,1);
        estimate(1,i) = x_curr(1,1);
        %plot(t(1,i),measure(1,i),'bo');
        %plot(t(1,i),estimate(1,i),'r*');
        %legend('Measure','Estimate');
        %shif
        x_pre = x_curr;
        P_pre = P_curr;
        yt=y;
        mu=H*x_inter;
        sig=H * P_inter * H.' + R;
        sig = (sig+sig')/2;%SM: quick fix that avoids the need to have a square-root implementation of a Kalman filter
        multiplier = my_matrix_norm_log(yt,mu,sig);%SM: you probably need to write your own logmvnpdf(.) function
        temp = temp + log(multiplier);
    end
    optimal_est_x(1,count) = sigma;
    optimal_est_y(1,count) = temp;%temp(1,1)+temp(1,2)+temp(2,1)+temp(2,2);
    plot(optimal_est_x(1,count)),abs(optimal_est_y(1,count),'ro--');
end

%plot(t,TrueValue)