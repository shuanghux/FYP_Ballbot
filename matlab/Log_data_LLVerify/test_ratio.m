function r = test_ratios(a,b)
T = size(a,2);
count = 0;
for i = 1:T
   if (a(i)-b(i))/a(i)<0.01
       count = count + 1;
   end
end
r = count/T;
end