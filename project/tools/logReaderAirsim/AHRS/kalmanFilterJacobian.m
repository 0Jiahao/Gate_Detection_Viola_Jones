clear
clc

syms phi theta psi p q r b_p b_q b_r

dx = [1 tan(theta)*sin(phi) tan(theta)*cos(phi);...
      0 cos(phi) -sin(phi) ;...
      0 sin(phi)/cos(theta) cos(phi)/cos(theta)]*[p-b_p;q-b_q;r-b_r];
model = [dx;0;0;0];       
states = [phi theta psi  b_p b_q b_r ];
        
F_sys = jacobian(model,states);
 F_sim = simplify(F_sys);
        
 temp = 1;
        