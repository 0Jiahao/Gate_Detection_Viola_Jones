function verifyDroneModelEarthFrame(GT)

predictionStatesEarth = zeros(length(GT.TIME),9);
predictionStatesBody = zeros(length(GT.TIME),9);

for i = 1:length(GT.TIME)
   if i == 1
       predictionStatesEarth(i,:) = [GT.X(i) GT.Y(i) GT.Z(i) GT.VX(i) GT.VY(i) GT.VZ(i) GT.PHI(i) GT.THETA(i) GT.PSI(i)];
       
       phi = GT.PHI(i);
       theta = GT.THETA(i);
       psi = GT.PSI(i);
       R_E_B = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
           sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
           sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
           cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
           cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
       predictionStatesBody(i,:) = [GT.X(i) GT.Y(i) GT.Z(i) [R_E_B*[GT.VX(i) GT.VY(i) GT.VZ(i)]']' GT.PHI(i) GT.THETA(i) GT.PSI(i)];
       continue;
   end
   deltaT = GT.TIME(i) - GT.TIME(i-1);
   currentStates = predictionStatesEarth(i-1,:);
   currentInputs = [GT.AX(i-1) GT.AY(i-1) GT.AZ(i-1) GT.P(i-1) GT.Q(i-1) GT.R(i-1)];

   dx = droneModelEarth(currentStates,currentInputs);
   predictionStatesEarth(i,:) = predictionStatesEarth(i-1,:) + dx'*deltaT;
   currentStates = predictionStatesBody(i-1,:);
   dx = droneModelBody(currentStates,currentInputs);
   predictionStatesBody(i,:) = predictionStatesBody(i-1,:) + dx'*deltaT;
end

figure(1)
subplot(3,1,1)
plot(GT.TIME,GT.X,'.-');
hold on
plot(GT.TIME,predictionStatesEarth(:,1));
plot(GT.TIME,predictionStatesBody(:,1),'.');
subplot(3,1,2)
plot(GT.TIME,GT.Y,'.-');
hold on
plot(GT.TIME,predictionStatesEarth(:,2));
plot(GT.TIME,predictionStatesBody(:,2),'.');
subplot(3,1,3)
plot(GT.TIME,GT.Z,'.-');
hold on
plot(GT.TIME,predictionStatesEarth(:,3));
plot(GT.TIME,predictionStatesBody(:,3),'.');

figure(2)
subplot(3,1,1)
plot(GT.TIME,GT.PHI/pi*180,'.-');
hold on
plot(GT.TIME,predictionStatesEarth(:,7)/pi*180);
subplot(3,1,2)
plot(GT.TIME,GT.THETA/pi*180,'.-');
hold on
plot(GT.TIME,predictionStatesEarth(:,8)/pi*180);
subplot(3,1,3)
plot(GT.TIME,GT.PSI/pi*180,'.-');
hold on
plot(GT.TIME,predictionStatesEarth(:,9)/pi*180);


end

function dx = droneModelEarth(state,input)

vx = state(4);
vy = state(5);
vz = state(6);
phi = state(7);
theta = state(8);
psi = state(9);
AX = input(1);
AY = input(2);
AZ = input(3);
p = input(4);
q = input(5);
r = input(6);

R_E_B = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
    sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
    cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
    cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
R_B_E = R_E_B';

dp = [vx vy vz]';
dv = [0 0 9.8]'+R_B_E*[AX AY AZ]';
dAtt = [1 tan(theta)*sin(phi) tan(theta)*cos(phi);...
    0 cos(phi) -sin(phi);...
    0 sin(phi)/cos(theta) cos(phi)/cos(theta)]*[p q r]';
dx = [dp;dv;dAtt];
end

function dx = droneModelBody(state,input)

vx_b = state(4);
vy_b = state(5);
vz_b = state(6);
phi = state(7);
theta = state(8);
psi = state(9);
AX = input(1);
AY = input(2);
AZ = input(3);
p = input(4);
q = input(5);
r = input(6);


R_E_B = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
    sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
    cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
    cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
R_B_E = R_E_B';

dp = R_B_E * [vx_b vy_b vz_b]';
dv = R_E_B*[0 0 9.8]'+[AX AY AZ]'-cross([p q r]',[vx_b vy_b vz_b]');
dAtt = [1 tan(theta)*sin(phi) tan(theta)*cos(phi);...
    0 cos(phi) -sin(phi);...
    0 sin(phi)/cos(theta) cos(phi)/cos(theta)]*[p q r]';
dx = [dp;dv;dAtt];
end