% This is the program which simulates the performance of the inverted
% pendulum. The feedback gain is calculated based on the state space model obtained beforehand.
% By Shuang Hu
%State space matrix
A=[0 1;9.8/0.2 1/0.2]; 
B=[0;1/0.2];
%Poles could be assigned anywhere because it has been verified that the
%System is completely controllable
P=[-4,-5];  
%Feedback gain calculated
K1=place(A,B,P);
%The following code simulate the performance the inverted pendulum in 20
%seconds
t=0;
timeset=20;
dt=0.01;
%The initial state of the pendulum is set to be standing upright
x=[0;0];z=[0;0];
X=[];T=[];Z=[];
count=0;
%LQR method feedback gain---------------------------
R=[1];Q=diag([1/(2*pi) 1]);
K2=lqr(A,B,Q,R);
%LQR method gain calculation finished---------------
while(t<timeset)
    count=count+1;
    X=[X,x];T=[T;t];
    Z=[Z,z];
    z=z+dt.*(A-B*K2)*z;
    x=x+dt.*(A-B*K1)*x;
    t=t+0.01;
    %For every 4 seconds, a new tilt(random number within -15 to 15
    %degrees) is applied to the rod (both tilt angle and angular velocity
    %are set).
    if rem(count,400) == 0
        x1=30*(rand()-0.5);
        x2=30*(rand()-0.5);
        x=[x1;x2];z=[x1;x2];
    end
end
hold on;
grid on;
title('Rod angle in time domain');
xlabel('time(s)');
ylabel('Angle(degree)')
plot(T,X(1,:),'b');
plot(T,Z(1,:),'r');
disp(K1);
disp(K2);

