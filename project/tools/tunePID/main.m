function [outputArg1,outputArg2] = main(inputArg1,inputArg2)
%MAIN Summary of this function goes here
%   Detailed explanation goes here

clear
clc
close all

k_p_x = 1.2;
k_p_v = 0.4;

SimTime = 10;
step = 0.001;
states = zeros(SimTime/step,2);
x_ref = ones(SimTime/step,1);
time = ones(SimTime/step,1);
input = ones(SimTime/step,1);
for i = 1:SimTime/step
    time(i) = step*i;
    if i == 1
        states(i,:) = [0,0];
        errorX = x_ref(i) - states(i,1);
        desiredV = errorX*k_p_x;
        errorV = desiredV-states(i,2);
        desiredPhi = errorV*k_p_v;
        input(i) = desiredPhi;
        continue;
    end
    dx = model(states(i-1,:), desiredPhi);
    states(i,:) = states(i-1,:) + dx*(time(i)-time(i-1));
    errorX = x_ref(i) - states(i,1);
    desiredV = errorX*k_p_x;
   % errorV = desiredV-states(i,2);
   currentV = (states(i,1) - states(i-1,1))/((time(i)-time(i-1)));
   errorV = desiredV-currentV;
    desiredPhi = errorV*k_p_v;
    input(i) = desiredPhi;
end

figure(1)
subplot(2,1,1)
hold on
plot(time,states(:,1));
plot(time,x_ref);
xlabel('time [s]')
ylabel('x[m]')
subplot(2,1,2)
plot(time,input(:,1)/pi*180);
xlabel('time [s]')
ylabel('phi [deg]')

end



function [dx] = model(states, input)
v = states(2);
phi = input;
dx = [v 9.8*tan(phi)];
end