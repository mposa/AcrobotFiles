checkDependency('lcm');
javaaddpath('LCMTypes/acrobot_types.jar')

q0 = [pi;0];
u0 = 0;

x0 = [q0;0;0];
% Calibrate first
x_offset = calibrateAcrobot(q0);
q_offset = x_offset(1:2);

display('Pausing until ready to begin test')
pause


%%
p = AcrobotPlantSmooth;

% swept sine. u = 3*sin(wt)
%             w = t
t_s = linspace(0,10,1e4);
u_s = 3*sin(t_s.*t_s);

u_traj = PPTrajectory(foh(t_s,u_s));

%%
uc = AcrobotFeedbackController(p,u_traj,zeros(1,4),zeros(4,1));
R_ekf = diag([1e-4;3e-4]); % measurement covariance, from tick resolution
Q_ekf = diag([1e-7;1e-7;.01;.01]); %process noise covariance
estimator = AcrobotBEKFEstimator(p,R_ekf,Q_ekf,q_offset);
sys = estimator.cascade(uc);
P0 = diag([1;1;1;1]);  % initial covariance
init_state = estimator.wrapState(0,x0,P0);
display('Starting sine demo commands now')
runLCM(sys,init_state,[]);