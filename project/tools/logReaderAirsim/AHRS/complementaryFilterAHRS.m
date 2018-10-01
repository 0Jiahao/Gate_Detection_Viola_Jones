function [] = complementaryFilterAHRS(SENSOR,GT)
FIGURE = 1;

% statesAHRS = zeros(length(SENSOR.TIME),3);
% KP =0.03;
% KI = 0.01;
% sumError = 0;
% 
% for i = 1:length(SENSOR.TIME)
%     if i == 1
%         statesAHRS(1,:) = [0,0,0];
%         dTheta = [0 0 0]';
%         continue;
%     end
%     
%     deltaT = SENSOR.TIME(i)-SENSOR.TIME(i-1);
%     statesAHRS(i,:) = statesAHRS(i-1,:) + (dTheta*deltaT)';
%     
%     phi = statesAHRS(i,1);
%     theta = statesAHRS(i,2);
%     psi = statesAHRS(i,3);
%     
%     R_E_B = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
%         sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
%         sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
%         cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
%         cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
%     
%     g_b = R_E_B*[0 0 -9.8]';
%     g_b = g_b/norm(g_b);
%     
%     acc = [SENSOR.AX(i),SENSOR.AY(i),SENSOR.AZ(i)]';
%     
%     accUnit = acc/norm(acc);
%     
%     angularVelocityError = cross(accUnit,g_b);
%     sumError = sumError + angularVelocityError*deltaT;
%     
%     angularVelocity = KP*angularVelocityError+KI*sumError+[SENSOR.P(i),SENSOR.Q(i),SENSOR.R(i)]';
%     
%     
%     dTheta = [1 tan(theta)*sin(phi) tan(theta)*cos(phi);...
%         0 cos(phi) -sin(phi) ;...
%         0 sin(phi)/cos(theta) cos(phi)/cos(theta)]*angularVelocity;      
% end

if FIGURE == 1
   figure(5)
   subplot(3,1,1)
   hold on
   grid on;
%    plot(GT.TIME,GT.PHI/pi*180);
   plot(SENSOR.TIME,SENSOR.PHI/pi*180);
   ylabel('phi [deg]');
   subplot(3,1,2);
   hold on
   grid on;
%    plot(GT.TIME,GT.THETA/pi*180);
    plot(SENSOR.TIME,SENSOR.THETA/pi*180);
   ylabel('theta [deg]');
   subplot(3,1,3)
   hold on
   grid on;
   plot(GT.TIME,GT.PSI/pi*180);
    plot(SENSOR.TIME,SENSOR.PSI/pi*180);
   ylabel('psi [deg]');
end

temp = 1;

end