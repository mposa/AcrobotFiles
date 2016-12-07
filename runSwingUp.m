checkDependency('lcm');
javaaddpath('LCMTypes/acrobot_types.jar')

qcal = [pi;0];

q0 = [0;0];
u0 = 0;

x0 = [q0;0;0];
% Calibrate first
x_offset = calibrateAcrobot(qcal);
q_offset = x_offset(1:2);


%%
% p = AcrobotPlantSmooth;

load twoPumpSwingUp
xtraj = xtraj.setOutputFrame(p.getStateFrame);
utraj = utraj.setOutputFrame(p.getInputFrame);

Q = diag([10;10;1;1]);
R = 5;
Qf = 10*Q;

[ltvsys,V] = p.tvlqr(xtraj,utraj,Q,R,Qf);

Ktraj = -ltvsys.D;

%%
%%
uc = AcrobotFeedbackController(p,utraj,Ktraj,xtraj);
R_ekf = diag([1e-4;3e-4]); % measurement covariance, from tick resolution
Q_ekf = diag([1e-7;1e-7;.01;.01;1e-4;1e-4]); %process noise covariance
estimator = AcrobotBEKFEstimator(p,R_ekf,Q_ekf,q_offset);
sys = estimator.cascade(uc);
P0 = diag([1;1;1;1;.05;.05]);  % initial covariance
init_state = estimator.wrapState(0,[x0;0;0],P0);

%%
display('Pausing before experiment. Press key to start')
figure(100)
waitforbuttonpress
close(100)
display('Starting swingup demo commands now')
runLCM(sys,init_state,[]);