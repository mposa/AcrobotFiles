checkDependency('lcm');
javaaddpath('LCMTypes/acrobot_types.jar')

q0 = [pi;0];
u0 = 0;

%%
p = AcrobotPlantSmooth;


x0 = [q0;0;0];
[A,B] = p.linearize(0,x0,u0);
% xdot ~ A*(x-x0) + B*(u-u0)

Q = diag([10;10;10;10]);
R = 5;
[K,S] = lqr(A,B,Q,R);
% K = [0; -1; 0; -.1]';
K
u0_traj = ConstantTrajectory(u0);
x0_traj = ConstantTrajectory(x0);
K_traj = ConstantTrajectory(K);


R_ekf = diag([1e-4;3e-4]); % measurement covariance, from tick resolution
Q_ekf = diag([1e-7;1e-7;.01;.01;1e-4;1e-4]); %process noise covariance
estimator = AcrobotBEKFEstimator(p,R_ekf,Q_ekf);
uc = AcrobotFeedbackController(p,u0_traj,K_traj,x0_traj);
sys = estimator.cascade(c);

P0 = diag([1;1;1;1;.05;.05]);  % initial covariance
init_state = estimator.wrapState(0,[x0;0;0],P0);
disp(sprintf('Starting feedback controller about point (%f,%f). Sending commands now...',q0(1),q0(2)))
runLCM(sys,init_state,[]);