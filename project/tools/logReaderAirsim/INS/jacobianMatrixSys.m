function [] = jacobianMatrixSys()
syms x y z v_z_B b_x b_y b_z phi theta psi_heading
syms ax ay az p q K_X K_Y g

R_E_B = [cos(theta)*cos(psi_heading) cos(theta)*sin(psi_heading) -sin(theta);...
     sin(phi)*sin(theta)*cos(psi_heading)-cos(phi)*sin(psi_heading)...
     sin(phi)*sin(theta)*sin(psi_heading)+cos(phi)*cos(psi_heading) sin(phi)*cos(theta);...
     cos(phi)*sin(theta)*cos(psi_heading)+sin(phi)*sin(psi_heading)...
cos(phi)*sin(theta)*sin(psi_heading)-sin(phi)*cos(psi_heading) cos(phi)*cos(theta)];

R_B_E = transpose(R_E_B);

dx = R_B_E*[1/K_X 0 0;0 1/K_Y 0;0 0 1]*[ax-b_x; ay-b_y;v_z_B];
dV_Z_B = az - b_z + g*cos(theta)*cos(phi)+q*(ax-b_x)/(K_X)-p*(ay-b_y)/(K_Y);
dBias = [0;0;0];
model = [dx;dV_Z_B;dBias];
states = [x y z v_z_B b_x b_y b_z];
F_sys = jacobian(model,states);
F_sim = simplify(F_sys);

end