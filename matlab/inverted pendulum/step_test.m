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
D=[0];
C=[1 0];
states = {'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'phi'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
poles = eig(A)
co = ctrb(sys_ss);
controllability = rank(co)
ob = obsv(sys_ss);
observability = rank(ob)
