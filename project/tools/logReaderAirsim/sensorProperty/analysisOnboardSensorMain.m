function [outputArg1,outputArg2] = analysisOnboardSensorMain(inputArg1,inputArg2)

OB = import_onboard_data('00002');

N = length(OB.TIME);
fs = 512;
i = 0:N-1;
t = i/fs;
y = fft(OB.IMU.AX,N);
mag = abs(y);
f = (0:N-1)*fs/N;
power=mag.^2;
figure
semilogx(f,power)
% plot(f,power)
end

