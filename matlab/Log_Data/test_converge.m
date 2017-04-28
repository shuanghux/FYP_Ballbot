function converged  = test_converge(L_pre,L_curr)

comparator = ((L_curr)-abs(L_pre))/(abs(L_curr)+abs(L_pre));
if abs(comparator) < 0.00001
    converged = 1;
else
    converged = 0;
end

end