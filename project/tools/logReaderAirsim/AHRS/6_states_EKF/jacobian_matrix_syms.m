function [F_syms,H_syms,G_syms,model,h,states] = jacobian_matrix_syms()
syms phi theta psi_heading
syms b_p b_q b_r
syms p_m q_m r_m g

R_q_phi = [1 tan(theta)*sin(phi) tan(theta)*cos(phi);...
    0 cos(phi) -sin(phi);...
    0 sin(phi)/cos(theta) cos(phi)/cos(theta)];

d_phi = R_q_phi*transpose([p_m-b_p q_m-b_q r_m-b_r]);
d_bias = transpose([0 0 0]);
model = [d_phi;d_bias];
states = [phi theta psi_heading b_p b_q b_r];
F_syms = jacobian(model,states);
F_sim= simplify(F_syms);


h = [sin(theta)*g;-sin(phi)*cos(theta)*g;-cos(theta)*cos(phi)*g;psi_heading];
H_syms = jacobian(h,states);
H_sim = simplify(H_syms);
G_syms = [R_q_phi;zeros(3,3)];
G_sim = simplify(G_syms);
end
