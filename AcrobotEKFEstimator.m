classdef AcrobotEKFEstimator < DrakeSystem
  properties
    plant
    C = [eye(2) zeros(2)]; % measurement model    R
    Q
    R
    lcm = [];
    lcm_x_coder
    offset
  end
  
  methods
    function obj = AcrobotEKFEstimator(plant, R, Q, offset, outputLCM)
      % initialize as DrakeSystem with 4^2+4+1 discrete states, 3 inputs and
      % 4 outputs
      obj = obj@DrakeSystem(0, 4*4+4+1, 3, 4);
      
      obj.plant = plant;
      obj.R = R;
      obj.Q = Q;
      obj.offset = offset;
      
      if nargin < 5
        outputLCM = true;
      end
      if outputLCM
        obj.lcm = lcm.lcm.LCM.getSingleton();
        obj.lcm_x_coder = AcrobotStateCoder();
      end
      
      % setup LCM frames
      lcmInFrame = LCMCoordinateFrameWCoder('acrobot_y', 3, 'y', AcrobotYCoder);
      obj = obj.setInputFrame(lcmInFrame);
      
      obj = obj.setOutputFrame(plant.getStateFrame);
      
      obj = obj.setSampleTime([1/200; 0]);
    end
    
    function state_n = update(obj,t,state,y)
      [q_meas,u] = unwrapY(obj,y);   
      q_meas = q_meas - obj.offset;
      [t_last,x,P] = unwrapState(obj,state);
      
      dt = t - t_last;      
      dt = min(max(dt,0),.1); % limit to [0,.1]
      
      [xdot,dxdot] = obj.plant.dynamics(0,x,u);
      F = eye(4) + dt*dxdot(:,2:5);
      
      % predict step
      x_pred = x + dt*xdot;
      P_pred = F*P*F' + obj.Q;
      
      %update step
      y_resid = q_meas - obj.C*x_pred;
      S = obj.C*P_pred*obj.C' + obj.R;
      K = P_pred*obj.C'/S;
      
      x_n = x_pred + K*y_resid;
      P = (eye(4) - K*obj.C)*P_pred;
      
      state_n = wrapState(obj,t,x_n,P);
    end
    
    function x = output(obj,t,state,y)
      [t,x,P] = unwrapState(obj,state);
      
      if ~isempty(obj.lcm)
        statemsg = obj.lcm_x_coder.encode(t,x);
        obj.lcm.publish('acrobot_xhat',statemsg);      
      end
    end
    
    function [t,x,P] = unwrapState(obj,state)
      t = state(1);
      x = state(2:5);
      P = reshape(state(6:end),4,4);
    end
    
    function [q,u] = unwrapY(obj,y)
      q = y(1:2);
      u = y(3);
    end
    
    function state = wrapState(obj,t,x,P)
      state = [t;x;P(:)];
    end
    
    
  end
  
end

