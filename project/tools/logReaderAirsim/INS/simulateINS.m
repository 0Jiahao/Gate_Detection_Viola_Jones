function [] = simulateINS(GT)
detectionFrequency = 10;
numVision = round(GT.TIME(end)*detectionFrequency);
maskDetection = zeros(length(GT.TIME),1);
index = randperm(length(GT.TIME),numVision);
maskDetection(index) = 1;
% maskDetection = ones(length(GT.TIME),1);
sigmaEuler = 0.0/180*pi;
sigmaAcc = 0.0;
sigmaRate = 0.0/180*pi;
sigmaVision = 0.5;

% sigmaEuler = 0/180*pi;
% sigmaAcc = 0;
% sigmaRate = 0/180*pi;
% sigmaVision = 0;

ACC = zeros(length(GT.TIME),3);
PQR = zeros(length(GT.TIME),3);
ATT = zeros(length(GT.TIME),3);
VISION = zeros(length(GT.TIME),3);
% generate noisy sensor reading

for  i = 1:length(GT.TIME)
   ACC(i,:) = [GT.AX(i) GT.AY(i) GT.AZ(i)]...
       +[normrnd(0,sigmaAcc) normrnd(0,sigmaAcc) normrnd(0,sigmaAcc)]; 
   PQR(i,:) = [GT.P(i) GT.Q(i) GT.R(i)]...
       +[normrnd(0,sigmaRate) normrnd(0,sigmaRate) normrnd(0,sigmaRate)];
   ATT(i,:) = [GT.PHI(i) GT.THETA(i) GT.PSI(i)]...
       +[normrnd(0,sigmaEuler) normrnd(0,sigmaEuler) normrnd(0,sigmaEuler)];
   if maskDetection(i) == 1
      VISION(i,:) = [GT.X(i) GT.Y(i) GT.Z(i)] ...
       +[normrnd(0,sigmaVision) normrnd(0,sigmaVision) normrnd(0,sigmaVision)];
   end
end

ACC(:,1) = smooth(ACC(:,1),50); 
ACC(:,2) = smooth(ACC(:,2),50);
ACC(:,3) = smooth(ACC(:,3),50);

%% verify generated measurement

% figure(1)
% subplot(3,1,1)
% hold on
% plot(GT.TIME,GT.AX);
% plot(GT.TIME,ACC(:,1),'.');
% ylabel('acc')
% subplot(3,1,2)
% hold on
% plot(GT.TIME,GT.AY);
% plot(GT.TIME,ACC(:,2),'.');
% ylabel('acc')
% subplot(3,1,3)
% hold on
% plot(GT.TIME,GT.AZ);
% plot(GT.TIME,ACC(:,3),'.');
% ylabel('acc')
% 
% figure(2)
% subplot(3,1,1)
% hold on
% plot(GT.TIME,GT.PHI);
% plot(GT.TIME,ATT(:,1),'.');
% ylabel('phi')
% subplot(3,1,2)
% hold on
% plot(GT.TIME,GT.THETA);
% plot(GT.TIME,ATT(:,2),'.');
% ylabel('theta')
% subplot(3,1,3)
% hold on
% plot(GT.TIME,GT.PSI);
% plot(GT.TIME,ATT(:,3),'.');
% ylabel('acc')
% 
% figure(3)
% subplot(3,1,1)
% hold on
% plot(GT.TIME,GT.P);
% plot(GT.TIME,PQR(:,1),'.');
% ylabel('P')
% subplot(3,1,2)
% hold on
% plot(GT.TIME,GT.Q);
% plot(GT.TIME,PQR(:,2),'.');
% ylabel('q')
% subplot(3,1,3)
% hold on
% plot(GT.TIME,GT.R);
% plot(GT.TIME,PQR(:,3),'.');
% ylabel('r')
% 
% figure(4)
% subplot(3,1,1)
% hold on
% plot(GT.TIME,GT.X);
% plot(GT.TIME,VISION(:,1),'.');
% ylabel('x')
% subplot(3,1,2)
% hold on
% plot(GT.TIME,GT.Y);
% plot(GT.TIME,VISION(:,2),'.');
% ylabel('y')
% subplot(3,1,3)
% hold on
% plot(GT.TIME,GT.Z);
% plot(GT.TIME,VISION(:,3),'.');
% ylabel('z')


%% Kalman filter
EKF_states = zeros(length(GT.TIME),7);
Q = diag([0.02 0.02 0.02 0.05 0.001 0.001 0.001 ]);
R = (10 * eye(3))^2;
p = 1;
for i = 1:length(GT.TIME)
   if i == 1
       P_k_k = 10*eye(7);
       P_trace(i) = trace(P_k_k);
      continue; 
   end
   
   deltaT = GT.TIME(i) - GT.TIME(i-1);
   inputs = [ATT(i-1,1) ATT(i-1,2) ATT(i-1,3) ...
       ACC(i-1,1) ACC(i-1,2) ACC(i-1,3) PQR(i-1,1) PQR(i-1,2) ...
       PQR(i-1,3)];
   EKF_states(i,:) = EKF_states(i-1,:) + predictModelINS(EKF_states(i-1,:),inputs)'*deltaT;
   [F,H_k] = jacobianMatrix(EKF_states(i-1,:),inputs);
   [PHI_k_k_1] = eye(7)+F*deltaT;
    P_k_1_k_1 = P_k_k;
    P_k_k_1 = PHI_k_k_1*P_k_1_k_1*PHI_k_k_1'+Q;
    if maskDetection(i) == 1
        Z_k_k_1 = [EKF_states(i,1) EKF_states(i,2) EKF_states(i,3)];
        Z_k = [VISION(i,1) VISION(i,2) VISION(i,3)]';
        K_k = P_k_k_1*H_k'/ (H_k*P_k_k_1*H_k'+R);
        delta_x_k_k = K_k*(Z_k-Z_k_k_1');
        EKF_states(i,:) = EKF_states(i,:) + delta_x_k_k';
        P_k_k = (eye(7)-K_k*H_k)*P_k_k_1;
        measurements(p,:) = [GT.TIME(i) VISION(i,1) VISION(i,2) VISION(i,3)];
        p = p+1;
    else
        P_k_k = P_k_k_1;
    end
    P_trace(i) = trace(P_k_k);
end

figure(5)
subplot(3,1,1)
hold on
plot(GT.TIME,GT.X);
plot(GT.TIME,EKF_states(:,1));
% plot(measurements(:,1),measurements(:,2),'.');
ylabel('x')
subplot(3,1,2)
hold on
plot(GT.TIME,GT.Y);
plot(GT.TIME,EKF_states(:,2));
% plot(measurements(:,1),measurements(:,3),'.');
ylabel('y')
subplot(3,1,3)
hold on
plot(GT.TIME,GT.Z);
plot(GT.TIME,EKF_states(:,3));
% plot(measurements(:,1),measurements(:,4),'.');
ylabel('z')

figure(6)
subplot(3,1,1)
plot(GT.TIME,EKF_states(:,5));
subplot(3,1,2)
plot(GT.TIME,EKF_states(:,6));
ylabel('bias');

subplot(3,1,3)
plot(GT.TIME,EKF_states(:,7));
figure(7)
plot(P_trace);


end