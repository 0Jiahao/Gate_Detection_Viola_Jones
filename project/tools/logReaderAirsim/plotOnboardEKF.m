function [] = plotOnboardEKF(SENSOR,GT)
for i = 1:length(SENSOR.TIME)
   if SENSOR.X_EKF(i) ~= 0
       m = i;
       break;
   end
end

p = 1;
for i = 1:length(SENSOR.TIME)
    if(SENSOR.X(i))~= 0
        detection(p,:) = [SENSOR.TIME(i) SENSOR.X(i) SENSOR.Y(i) SENSOR.Z(i)];
        p = p+1;
    end
end

figure()
subplot(3,1,1)
hold on
plot(SENSOR.TIME(:),SENSOR.X_EKF(:));
plot(detection(:,1),detection(:,2),'.');
plot(GT.TIME,GT.X-18.2);
subplot(3,1,2)
hold on
plot(SENSOR.TIME(:),SENSOR.Y_EKF(:));
plot(detection(:,1),detection(:,3),'.');
plot(GT.TIME,GT.Y);
plot(GT.TIME,zeros(length(GT.TIME),1));
subplot(3,1,3)
hold on
plot(SENSOR.TIME(m:end),SENSOR.Z_EKF(m:end));
plot(detection(:,1),detection(:,4),'.');

figure()
subplot(3,1,1)
hold on
plot(SENSOR.TIME(m:end),SENSOR.BX_EKF(m:end));
subplot(3,1,2)
hold on
plot(SENSOR.TIME(m:end),SENSOR.BY_EKF(m:end));
subplot(3,1,3)
hold on
plot(SENSOR.TIME(m:end),SENSOR.BZ_EKF(m:end));
end