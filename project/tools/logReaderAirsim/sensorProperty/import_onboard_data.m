function OB = import_onboard_data(name)

OB = struct;

%data = csvread(['onboard_log ' name '.csv'],0,0);
data = csvread([ name '.csv'],0,0);
% IMU = struct;
% OTOB = struct;
% BAT = struct;
% COM = struct;
% ACT = struct;
g = 9.8124;
scale_pqr = 0.0139882;   %deg/s
scale_acc = 1/1024; %1/1024
%scale_acc = 0.0009766;

S_acc = 4096; % 2048 according to datasheet
S_om = 1879.3; % 16.4 according to datasheet

OB.COUNTER     = data(:,1);
OB.TIME    = data(:,2);
%OB.IMU.TIME    = data(:,2);
OB.IMU.P       = data(:,3) * scale_pqr;
OB.IMU.Q       = data(:,4) * scale_pqr;
OB.IMU.R       = data(:,5) * scale_pqr;
OB.IMU.AX      = data(:,6) * scale_acc;
OB.IMU.AY      = data(:,7) * scale_acc;
OB.IMU.AZ      = data(:,8) * scale_acc;
% OB.IMU.P_UNSCALED = data(:,10) / S_om;
% OB.IMU.Q_UNSCALED = data(:,11) / S_om;
% OB.IMU.R_UNSCALED = data(:,12) / S_om;
% OB.IMU.AX_UNSCALED = data(:,13) / S_acc;
% OB.IMU.AY_UNSCALED = data(:,14) / S_acc;
% OB.IMU.AZ_UNSCALED = data(:,15) / S_acc;
OB.IMU.MAGX    = data(:,9);
OB.IMU.MAGY    = data(:,10);
OB.IMU.MAGZ    = data(:,11);
OB.IMU.PHI     = data(:,12);%*57.3;
OB.IMU.THETA   = data(:,13);%*57.3;
OB.IMU.PSI     = data(:,14);%*57.3;

OB.OTOB.PosNED = data(:,15:17);
OB.OTOB.VelNED = data(:,18:20);

OB.COM.THRUST = data(:,21);
OB.COM.PHI = data(:,22);
OB.COM.THETA = data(:,23);
OB.COM.PSI = data(:,24);
OB.BAT.VOLTAGE = data(:,25);
OB.BAT.CURRENT = data(:,26);
OB.COM.RPM1 = data(:,27);
OB.COM.RPM2 = data(:,28);
OB.COM.RPM3 = data(:,29);
OB.COM.RPM4 = data(:,30);

OB.ACT.RPM1 = data(:,31);
OB.ACT.RPM2 = data(:,32);
OB.ACT.RPM3 = data(:,33);
OB.ACT.RPM4 = data(:,34);

OB.arc_x = data(:,36);
OB.arc_y = data(:,37);
OB.arc_z = data(:,38);

OB.arc_v_x_f = data(:,39);
OB.arc_v_y_f = data(:,40);
OB.arc_v_z_f = data(:,41);

OB.phi_cmd_arc = data(:,42);
OB.theta_cmd_arc = data(:,43);
OB.psi_cmd_arc = data(:,44);
OB.flag_in_arc = data(:,45);

OB.accel.P.p = data(:,55)* scale_pqr;
OB.accel.P.q = data(:,56)* scale_pqr;
OB.accel.P.r = data(:,57)* scale_pqr;
OB.accel.I.p = data(:,58)* scale_pqr;
OB.accel.I.q = data(:,59)* scale_pqr;
OB.accel.I.r = data(:,60)* scale_pqr;
OB.gps.P.p = data(:,61)* scale_pqr;
OB.gps.P.q = data(:,62)* scale_pqr;
OB.gps.P.r = data(:,63)* scale_pqr;
OB.gravity_factor = data(:,64);

OB.rate_correction_p =  data(:,65)* scale_pqr;
OB.rate_correction_q =  data(:,66)* scale_pqr;
OB.rate_correction_r =  data(:,67)* scale_pqr;

OB.rate_bias_p =  data(:,68)* scale_pqr;
OB.rate_bias_q =  data(:,69)* scale_pqr;
OB.rate_bias_r =  data(:,70)* scale_pqr;

end