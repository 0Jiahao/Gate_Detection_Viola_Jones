clear
clc
close all

[SENSOR,GT] = readLog();
%   checkGroundTruthAngularVelocity(GT);
%  verifyDroneModelEarthFrame(GT);

%  complementaryFilterAHRS(SENSOR,GT)
%  kalmanFilterAHRS(SENSOR,GT);

% verifyPredictionModel(GT);

% simulateINS(GT);


% INS(GT,SENSOR);
plotOnboardEKF(SENSOR,GT);
    temp = 1;