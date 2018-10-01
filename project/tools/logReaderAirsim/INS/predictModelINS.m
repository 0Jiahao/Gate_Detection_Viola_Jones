function [dx] = predictModelINS(currentStates,currentInputs)

g = 9.8;

x = currentStates(1);
y = currentStates(2);
z = currentStates(3);
v_z_B = currentStates(4);
b_x = currentStates(5);
b_y = currentStates(6);
b_z = currentStates(7);

phi = currentInputs(1);
theta = currentInputs(2);
psi = currentInputs(3);
ax = currentInputs(4);
ay = currentInputs(5);
az = currentInputs(6);
p = currentInputs(7);
q = currentInputs(8);
r = currentInputs(9);

R_E_B = [cos(theta)*cos(psi) cos(theta)*sin(psi) -sin(theta);...
    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)...
    sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) sin(phi)*cos(theta);...
    cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)...
    cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi) cos(phi)*cos(theta)];
R_B_E = R_E_B';

v_x_b = -2 * (ax-b_x);
v_y_b = -2 * (ay-b_y);
v_z_b = v_z_B;
dX = R_B_E*[v_x_b v_y_b v_z_b]';
dv = R_E_B*[0 0 9.8]'+[ax ay az]'-[b_x b_y b_z]-cross([p q r]',[v_x_b v_y_b v_z_b]');
dVz = az - b_z + g*cos(theta)*cos(phi)+q*(ax-b_x)/(-0.5)-p*(ay-b_y)/(-0.5);
% dVz = dv(3);

dBias = [0 0 0]';

dx = [dX;dVz;dBias];


end