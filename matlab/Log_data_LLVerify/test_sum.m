function result = test_sum(H,xhat,y)
error = y-H*xhat;
result = -sum(error.^2);
end