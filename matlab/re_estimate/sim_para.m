function y = sim_para(F,H,B,Q,R,u)
T = size(u,2);
x = zeros(1,T);
y = zeros(1,T);
sigma_state = sqrt(Q);
sigma_measure = sqrt(R);
for t = 2:T
    uncertainty_state = normrnd(0,sigma_state);
    x(t) = F*x(t-1) + B*u(t-1) + uncertainty_state;
end
for t = 1:T
   uncertainty_measure = normrnd(0,sigma_measure); 
   y(t) = H*x(t) + uncertainty_measure;
end
end