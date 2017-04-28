function correlation()

load('loglike3D.mat');
LL = loglike3D;
Hm = linspace(0,2,300);
Bm = linspace(-0.004,0,300);
Qm = linspace(10^-7,10^-5,300);

m = length(Hm);
temp = zeros(300,300);
for i = 1:300
    for j = 1:300
        temp(i,j) = LL(150,i,j);
    end
end
for x = 1:m
    for y = 1:m
        surf(Bm,Qm,temp);
    end
end
