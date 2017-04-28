function [F,H,B,Q,R,pi1,V1] = set_initial(measurement)
% Set initial state variables
%based on the parameter estimates presented in Holmes and Fagan (2002)
T = length(measurement);
B = (tilde(measurement,T-3)-tilde(measurement,1))/(T-4);
tilde_vect = zeros(1,T-3);
for i = 1:T-3
    tilde_vect(i) = tilde(measurement,i);
end
Q = 1/3*(var(tilde_vect(5:end)-tilde_vect(1:end-4))-var(tilde_vect(2:end)-tilde_vect(1:end-1)));
R = 1/2*(var(tilde_vect(2:end)-tilde_vect(1:end-1))-Q);
if(Q < 0)
    Q = 0.0001; 
end
if(R < 0)
    R = 0.0001; 
end
pi1 = measurement(1); %pi is Ghahramani and Hinton's notation, but pi reserved
V1 = 0.4; %variance of initx`
F = 1;
H = 1;

end