function [] = kalmanFilterAHRS(SENSOR,GT)

FIGURE = 1;

EKF_states = zeros(length(SENSOR.TIME),6);
P_trace = zeros(length(SENSOR.TIME),1);
Q = diag([(0.5/180*pi)^2 (0.5/180*pi)^2 (0.5/180*pi)^2 (0.05/180*pi)^2 (0.05/180*pi)^2 (0.05/180*pi)^2]);
R = diag([(100)^2 (100)^2 0]);

for i = 1:length(SENSOR.TIME)
    if i == 1
        EKF_states(i,:) = [0 0 0 0 0 0];
        last_updated_states_EKF =  EKF_states(i,:);
        last_updated_inputs = [SENSOR.P(i) SENSOR.Q(i) SENSOR.R(i)];
        P_k_k = 10*eye(6);
        P_trace(i) = trace(P_k_k);
        continue;
    end
    deltaT = SENSOR.TIME(i)-SENSOR.TIME(i-1);
    inputs = [SENSOR.P(i-1) SENSOR.Q(i-1) SENSOR.R(i-1)];
    EKF_states(i,:) = EKF_states(i-1,:) + prediction_model_kalman_filter(EKF_states(i-1,:),inputs)*deltaT;
    [F,H_k,G] = jacobian_matrix(last_updated_states_EKF,last_updated_inputs,EKF_states(i,:));
    
    [PHI_k_k_1, Gamma] = c2d(F, G, deltaT);
    P_k_1_k_1 = P_k_k;
    P_k_k_1 = PHI_k_k_1*P_k_1_k_1*PHI_k_k_1'+Q;
    
    Z_k_k_1 = [EKF_states(i,1) EKF_states(i,2) EKF_states(i,3)];
 
    Z_k = [atan2(-SENSOR.AY(i),-SENSOR.AZ(i)) atan2(SENSOR.AX(i),sqrt(SENSOR.AY(i)^2+SENSOR.AZ(i)^2)) 0]';
%     Z_k = [0 0 0]';

%     figure(9)
%     hold on
%     plot(atan2(SENSOR.AX(i),sqrt(SENSOR.AY(i)^2+SENSOR.AZ(i)^2))/pi*180,'*');
%     
    K_k = P_k_k_1*H_k'/ (H_k*P_k_k_1*H_k'+R);
    delta_x_k_k = K_k*(Z_k-Z_k_k_1');
    EKF_states(i,:) = EKF_states(i,:) + delta_x_k_k';
    %P_k_k = (eye(6)-K_k*H_k)*P_k_k_1*(eye(6)-K_k*H_k)'+K_k*R*K_k';
    P_k_k = (eye(6)-K_k*H_k)*P_k_k_1;
    last_updated_states_EKF = EKF_states(i,:);
    last_updated_inputs = inputs;
     P_trace(i) = trace(P_k_k);
end

if FIGURE == 1
   figure(6)
   subplot(3,1,1)
   hold on
   grid on;
   plot(GT.TIME,GT.PHI/pi*180);
   plot(SENSOR.TIME,EKF_states(:,1)/pi*180);
   ylabel('phi [deg]');
   subplot(3,1,2);
   hold on
   grid on;
   plot(GT.TIME,GT.THETA/pi*180);
   plot(SENSOR.TIME,EKF_states(:,2)/pi*180);
   ylabel('theta [deg]');
   subplot(3,1,3)
   hold on
   grid on;
   plot(GT.TIME,GT.PSI/pi*180);
   plot(SENSOR.TIME,EKF_states(:,3)/pi*180);
   ylabel('psi [deg]');
   
   figure(7)
   subplot(3,1,1)
   hold on
   grid on;
   plot(SENSOR.TIME,EKF_states(:,4)/pi*180);
   ylabel('b_p')
   
    subplot(3,1,2)
   hold on
   grid on;
   plot(SENSOR.TIME,EKF_states(:,5)/pi*180);
   ylabel('b_q')
   
   subplot(3,1,3)
   hold on
   grid on;
   plot(SENSOR.TIME,EKF_states(:,6)/pi*180);
   ylabel('b_r')
   
   figure(8)
   plot(SENSOR.TIME,P_trace);
end



end