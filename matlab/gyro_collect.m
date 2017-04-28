function gyro_collect()
mylego = legoev3('usb');
mygyrosensor = gyroSensor(mylego);
%temp0 = 0;
%temp1 = toc;
t = 1:100;
gyrorate = 1:100;
gyroangle = 1:100;
%mea_ang = 1:100;
resetRotationAngle(mygyrosensor);
tic
for i=1:100
    t(1,i) = toc;
    %mea_ang(1,i) = readRotationAngle(mygyrosensor);
    gyrorate(1,i) = readRotationRate(mygyrosensor);
    if i > 1
        gyroangle(1,i) = gyroangle(1,i-1)+ gyrorate(1,i-1) * (t(1,i)-t(1,i-1));
    else
        gyroangle(1,i) = 0;
    end
    hold on;
    plot(t(1,i),gyrorate(1,i),'bo--');
    plot(t(1,i),gyroangle(1,i),'r*-');
    %plot(t(1,i),mea_ang(1,i),'g*-');
    drawnow;
    legend('GyroRate','CalculatAngle');
    %temp = toc;
end
%hold on;
plot(t,gyrorate,'b--');
plot(t,gyroangle,'r-');
%plot(t,mea_ang,'g*-');
%legend('GyroRate','CalculatAngle','MeasureAngle');
grid on;