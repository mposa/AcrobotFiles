clear all

p = AcrobotPlantSmooth;

R_ekf = diag([1e-4;3e-4]); % measurement covariance, from tick resolution
Q_ekf = diag([1e-7;1e-7;.01;.01]); %process noise covariance
estimator = AcrobotEKFEstimator(p,R_ekf,Q_ekf,[0.004;-62.8444]);
P0 = diag([1;1;1;1]);  % initial covariance

init_state = estimator.wrapState(0,[0;0;0;0],P0);

manualRunAcrobotLCM(estimator,[],init_state,.005);