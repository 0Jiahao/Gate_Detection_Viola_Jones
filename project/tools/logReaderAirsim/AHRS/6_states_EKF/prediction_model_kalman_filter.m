function dx = prediction_model_kalman_filter(current_states,input)
phi = current_states(1);
theta = current_states(2);
psi = current_states(3);
b_p = current_states(4);
b_q = current_states(5);
b_r = current_states(6);

p_m = input(1);
q_m = input(2);
r_m = input(3);

R_q_phi = [1 tan(theta)*sin(phi) tan(theta)*cos(phi);...
    0 cos(phi) -sin(phi);...
    0 sin(phi)/cos(theta) cos(phi)/cos(theta)];

d_phi = R_q_phi*[p_m-b_p q_m-b_q r_m-b_r]';
d_phi = d_phi';
d_bias = [0 0 0];
dx = [d_phi d_bias];
end