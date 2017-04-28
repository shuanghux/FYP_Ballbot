function my_matrix_norm(x,mu,sigma)
p = 0.5 * log(2*pi*det(sigma))- 0.5*(x-mu).'*inv(sigma)*(x-mu)
end