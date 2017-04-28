function B = getB(input,output)
T = length(input);
diff = zeros(1,T-1);
for i = 2:T-1
   diff(i) = (output(i+1)-output(i))/input(i);
end
B = mean(diff);
end