function [] = verifyPredictionModel(GT)

predictionStates = zeros(length(GT.TIME),7);

for i = 1:length(GT.TIME)
   if i == 1
       phi = GT.PHI(i);
       theta = GT.THETA(i);
       psi = GT.PSI(i);
       R_E_B = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
           sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
           sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
           cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
           cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
       velocityBody = R_E_B*[GT.VX(i) GT.VY(i) GT.VZ(i)]';
      predictionStates(i,:) = [GT.X(i) GT.Y(i) GT.Z(i) velocityBody(3) 0 0 0];   
      continue;
   end

   deltaT = GT.TIME(i) - GT.TIME(i-1);
   currentStates = predictionStates(i-1,:);
   currentInputs = [GT.PHI(i-1) GT.THETA(i-1) GT.PSI(i-1) GT.AX(i-1) GT.AY(i-1) GT.AZ(i-1) GT.P(i-1) GT.Q(i-1)];
   dx = predictModelINS(currentStates,currentInputs);
   predictionStates(i,:) = predictionStates(i-1,:) + dx'*deltaT;
end

figure()
subplot(3,1,1)
plot(GT.TIME,GT.X);
hold on
plot(GT.TIME,predictionStates(:,1));
subplot(3,1,2)
plot(GT.TIME,GT.Y);
hold on
plot(GT.TIME,predictionStates(:,2));
subplot(3,1,3)
plot(GT.TIME,GT.Z);
hold on
plot(GT.TIME,predictionStates(:,3));

end