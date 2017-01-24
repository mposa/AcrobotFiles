function [p,v,xtraj,utraj,z,F,info,traj_opt] = smoothSwingUp(xtraj,utraj,useAltFormulation)
if nargin < 3
  useAltFormulation = false;
end

p = AcrobotPlantSmooth;
[q0,qf] = p.calculateEquilibria;
p=p.setInputLimits(-9,9);
v = AcrobotVisualizer(p);
N = 50;
T = 10;
T0 = T/2;


x0 = [q0;0;0];
xf = [qf;0;0];

t_init = linspace(0,T0,N);

if nargin < 2
  traj_init.x = PPTrajectory(foh(t_init,linspacevec(x0,xf,N)));% + randn(4,N)*1));
  traj_init.x = traj_init.x.setOutputFrame(p.getStateFrame);
  traj_init.u = PPTrajectory(foh(t_init,randn(1,N)));
  traj_init.u = traj_init.u.setOutputFrame(p.getInputFrame);
else
  traj_init.x = xtraj;
  traj_init.u = utraj;
  t_init = linspace(0,xtraj.tspan(2),N);
end
T_span = [0 T];

% options = struct();
% options.integration_method = DirtranTrajectoryOptimization.FORWARD_EULER;
% options.integration_method = DirtranTrajectoryOptimization.BACKWARD_EULER;
% traj_opt = DirtranTrajectoryOptimization(p,N,T_span,options);
traj_opt = DircolTrajectoryOptimization(p,N,T_span,struct());
% traj_opt = traj_opt.setCheckGrad(true);
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',400);
% traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
% if ~useAltFormulation
  traj_opt = traj_opt.addRunningCost(@running_cost_fun);
  traj_opt = traj_opt.addFinalCost(@final_cost_fun);
% end
traj_opt = traj_opt.addStateConstraint(ConstantConstraint(x0),1);
traj_opt = traj_opt.addStateConstraint(ConstantConstraint(xf),N);
traj_opt = traj_opt.addInputConstraint(BoundingBoxConstraint(-4,4),1:N);
traj_opt = traj_opt.addInputConstraint(BoundingBoxConstraint(0,0),1);
% traj_opt = traj_opt.addLinearStateConstraint(LinearConstraint(xf,xf,eye(4)),N);


if useAltFormulation
%   [traj_opt,du_ind] = traj_opt.addDecisionVariable(1);
%   traj_opt = traj_opt.addQuadraticCost(1,0,du_ind);
  
  du_max = 20/N*T;
  for i=2:N,
      traj_opt = traj_opt.addConstraint(LinearConstraint(-du_max,du_max,[1 -1]),traj_opt.u_inds(i-1:i));
%     traj_opt = traj_opt.addConstraint(LinearConstraint(0,inf,[1 -1 1]),[traj_opt.u_inds(i-1:i) du_ind]);
%     traj_opt = traj_opt.addConstraint(LinearConstraint(-inf,0,[1 -1 -1]),[traj_opt.u_inds(i-1:i) du_ind]);
  end
end


traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(-2*pi,2*pi),1:N,2);

% max_vel = 5*pi;
% traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(-max_vel,max_vel),1:N,3);
% traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(-max_vel,max_vel),1:N,4);

tic
[xtraj,utraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
display(sprintf('Completed with info %d and cost %f',info,F));
toc
end

function [f,df] = running_cost_fun(h,x,u)
  R = 1;
  Q = 0*diag([0;0;1;1]);
  f = h*(u'*R*u + x'*Q*x);
  df = [(u'*R*u + x'*Q*x) 2*h*x'*Q 2*h*u'*R];
end

function [f,df] = final_cost_fun(T,x)
  K = 1;
  f = K*T;
  df = [K zeros(1,4)];
end