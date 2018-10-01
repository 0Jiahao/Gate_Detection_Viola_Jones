function verifyDroneModelBodyFrame(GT)

predictionStates = zeros(length(GT.TIME),6);

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
       predictionStates(i,:) = [GT.X(i) GT.Y(i) GT.Z(i) velocityBody'];
       continue;
   end
   deltaT = GT.TIME(i) - GT.TIME(i-1);
   currentStates = predictionStates(i-1,:);
   currentInputs = [GT.PHI(i-1) GT.THETA(i-1) GT.PSI(i-1) GT.AX(i-1) GT.AY(i-1) GT.AZ(i-1) GT.P(i-1) GT.Q(i-1) GT.R(i-1)];
   dx = droneModelBody(currentStates,currentInputs);
   predictionStates(i,:) = predictionStates(i-1,:) + dx'*deltaT;
end

figure()
subplot(3,1,1)
plot(GT.TIME,GT.X,'.-');
hold on
plot(GT.TIME,predictionStates(:,1),'.-');
subplot(3,1,2)
plot(GT.TIME,GT.Y,'.-');
hold on
plot(GT.TIME,predictionStates(:,2),'.-');
subplot(3,1,3)
plot(GT.TIME,GT.Z,'.-');
hold on
plot(GT.TIME,predictionStates(:,3),'.-');

end

function dx = droneModelBody(state,input)

vx_b = state(4);
vy_b = state(5);
vz_b = state(6);
phi = input(1);
theta = input(2);
psi = input(3);
AX = input(4);
AY = input(5);
AZ = input(6);
p = input(7);
q = input(8);
r = input(9);


R_E_B = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
    sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
    cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
    cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
R_B_E = R_E_B';

dp = R_B_E * [vx_b vy_b vz_b]';
dv = R_E_B*[0 0 9.8]'+[AX AY AZ]'-cross([p q r]',[vx_b vy_b vz_b]');
dx = [dp;dv];
end