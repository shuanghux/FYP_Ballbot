function logp = my_matrix_norm_log(x,mu,sigma)
p = 0.5 * log(2*pi*det(sigma))- 0.5*(x-mu).'*inv(sigma)*(x-mu);
logp = log(abs(p));
end