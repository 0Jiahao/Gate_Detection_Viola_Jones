function [outputArg1,outputArg2] = INS(GT,SENSOR)
%INS Summary of this function goes here
%   Detailed explanation goes here
EKF_states = zeros(length(SENSOR.TIME),7);

SENSOR.AX = smooth(SENSOR.AX,10);
SENSOR.AY = smooth(SENSOR.AY,10);
SENSOR.AZ = smooth(SENSOR.AZ,10);

Q = diag([0.01 0.01 0.01 0.01 0.01 0.01 0.01 ]);

R = (3 * eye(3))^2;
flag_start = 0;
p = 1;
for i = 1:length(SENSOR.TIME)
    if SENSOR.TIME(i) > 16 && SENSOR.X(i) ~= 14.5 && flag_start == 0
       start = i; 
       flag_start = 1;
    end
    if SENSOR.X(i) == 14.5
        SENSOR.X(i) = 0;
        SENSOR.Y(i) = 0;
        SENSOR.Z(i) = 0;
    end
end


for i = start:length(SENSOR.TIME)
    if i == start
        EKF_states(i,:) = [SENSOR.X(i) SENSOR.Y(i) SENSOR.Z(i) 0 0 0 0];
        P_k_k = 10*eye(7);
        P_trace(i) = trace(P_k_k);
        continue;
    end
    

    deltaT = SENSOR.TIME(i) - SENSOR.TIME(i-1);


    inputs = [SENSOR.PHI(i-1) SENSOR.THETA(i-1) SENSOR.PSI(i-1) ...
            SENSOR.AX(i-1) SENSOR.AY(i-1) SENSOR.AZ(i-1) SENSOR.P(i-1) SENSOR.Q(i-1) ...
            SENSOR.R(i-1)];
        
    EKF_states(i,:) = EKF_states(i-1,:) + predictModelINS(EKF_states(i-1,:),inputs)'*deltaT;
    [F,H_k] = jacobianMatrix(EKF_states(i-1,:),inputs);
    [PHI_k_k_1] = eye(7)+F*deltaT;
%     PHI_k_k_1 = c2d(F,deltaT,'zoh');
    P_k_1_k_1 = P_k_k;
    P_k_k_1 = PHI_k_k_1*P_k_1_k_1*PHI_k_k_1'+Q;
   
    
    if SENSOR.X(i) ~= 0
        Z_k_k_1 = [EKF_states(i,1) EKF_states(i,2) EKF_states(i,3)];
        Z_k = [SENSOR.X(i) SENSOR.Y(i) SENSOR.Z(i)]';
        K_k = P_k_k_1*H_k'/ (H_k*P_k_k_1*H_k'+R);
        delta_x_k_k = K_k*(Z_k-Z_k_k_1');
        EKF_states(i,:) = EKF_states(i,:) + delta_x_k_k';
        P_k_k = (eye(7)-K_k*H_k)*P_k_k_1;
         measurements(p,:) = [SENSOR.TIME(i) SENSOR.X(i) SENSOR.Y(i) SENSOR.Z(i)];
        p = p+1;
    else
        P_k_k = P_k_k_1;
    end
    P_trace(i) = trace(P_k_k);
end

figure()
subplot(3,1,1)
%plot(GT.TIME(start:end),GT.X(start:end));
hold on
plot(SENSOR.TIME(start:end),EKF_states(start:end,1));
plot(measurements(:,1),measurements(:,2),'*');
ylabel('x')

subplot(3,1,2)
ylabel('y')
%plot(GT.TIME,GT.Y);
hold on
plot(SENSOR.TIME(start:end),EKF_states(start:end,2));
plot(measurements(:,1),measurements(:,3),'*');
subplot(3,1,3)
hold on
plot(SENSOR.TIME(start:end),EKF_states(start:end,3));
plot(measurements(:,1),measurements(:,4),'*');
ylabel('z')

figure()
subplot(3,1,1)
plot(SENSOR.TIME(start:end),EKF_states(start:end,5));
subplot(3,1,2)
plot(SENSOR.TIME(start:end),EKF_states(start:end,6));
ylabel('bias');

subplot(3,1,3)
plot(SENSOR.TIME(start:end),EKF_states(start:end,7));
figure()
plot(P_trace);
end

