function P = gyro_kalman_collect_1D()
%-------------------Kalman Filter Section---------------------------------


%Notation: 
% _pre: previous state----------t-1 | t-1
% _inter: intermediate step-----t | t-1
% _curr: current state----------t | t

%assume initial state to be zero
theta = 0;
theta_dot = 0;
DC = 0;
x_pre = [theta_dot];
%inital kalamn parameters
u = DC;
dt = 0.01;
R = 1/12;
H = 1;
I = 1;
P_pre = 1;
sigma = 0.9;
Q = dt * sigma^2;
%--------------------LEGO Section-----------------------------------------
mylego = legoev3('usb');
mygyrosensor = gyroSensor(mylego);
t = 1:100;
%initialize output value
gyrorate_est = 1:100;
gyrorate_mea = 1:100;
gyroangle_est = 1:100;
gyroangle_mea = 1:100;
%Editing 1st Feb 2016 Optimizing P matrices
P_opt = 1:100;
%End editing

resetRotationAngle(mygyrosensor);
tic
delta=1:100;
for i=1:100
    %Sensor Measure Start
    t(1,i) = toc;
    gyrorate_mea(1,i) = readRotationRate(mygyrosensor);
    if i > 1
        dt = t(1,i) - t(1,i-1);
        %dt=0.05;
        delta(1,i)=dt;
        gyroangle_mea(1,i) = gyroangle_mea(1,i-1)+ gyrorate_mea(1,i-1) * dt;
    else
        gyroangle_mea(1,i) = 0;
        dt = 0.05;
        delta(1,i)=0.2;
    end
    %Sensor Measure End
    
    %----Kalman Parameter-------
    F = 1;
    %B = [dt^2/2 ; dt];
    %----Kalman Parameter-------
    
    %------Kalman Filtering Start--------
    %Measure
    y = gyrorate_mea(1,i);
    %predict
    x_inter = F * x_pre;
    P_inter = F * P_pre * F.' + Q;
    %Update
    K = P_inter * H.' * (H * P_inter * H.' + R)^(-1);
    x_curr = x_inter + K * (y - x_inter);
    P_curr = (I - K * H) * P_inter;
    %Plotting values
    %gyroangle_est(1,i) = x_curr(1,1);
    gyrorate_est(1,i) = x_curr;
    P_opt(1, i) = P_curr(1,1);
    %shift to next state
    x_pre = x_curr;
    P_pre = P_curr;
    %------Kalman Filtering End--------
    %--------Plotting Start-------------
    hold on;
    %plot(t(1,i),gyrorate_mea(1,i),'ro');
    %plot(t(1,i),gyroangle_mea(1,i),'r*');
    %plot(t(1,i),gyrorate_est(1,i),'bo');
    %plot(t(1,i),gyroangle_est(1,i),'b*');
    plot(t(1,i),P_opt(1,i),'go')
    drawnow;
    %legend('GyroRateMeasured','CalculatAngleMeasured','GyroRateEstimated','CalculatAngleEstimated');
    %--------Plotting End---------------
    if i == 100
        P = P_opt(1,100);
    end
end
grid on;
hold on;
%plot(t,gyroangle_mea,'r-');
%plot(t,gyroangle_est,'b-');

end

