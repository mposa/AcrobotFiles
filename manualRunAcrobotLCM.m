function manualRunAcrobotLCM(estimator,controller,est_state,dt)
  checkDependency('lcm');
  javaaddpath('LCMTypes/acrobot_types.jar')
  
  lcm_y_coder = AcrobotYCoder();
  lcm_u_coder = AcrobotInputCoder();
  
  lc = lcm.lcm.LCM.getSingleton();
  aggregator = lcm.lcm.MessageAggregator();
  aggregator.setMaxMessages(1);  % make it a last-message-only queue

  lc.subscribe('acrobot_y',aggregator);
  
  msg = aggregator.getNextMessage();  
  
  isController = ~isempty(controller);
  
  tic;
  tnext = dt;
 
  while true
    while toc < tnext
    end
    tnext = tnext + dt;
    % get most recent message
    nmsg = aggregator.getNextMessage(0);
    if ~isempty(nmsg)
      msg = nmsg;
    end
    
    y=lcm_y_coder.decode(msg);
    t = toc;
    est_state = estimator.update(t,est_state,y);    
    xhat = estimator.output(t,est_state,y);
    if isController      
      u = controller.output(t,[],xhat);
      control_msg = lcm_u_coder.encode(t,u);
      lc.publish('acrobot_u',control_msg);      
    end  
    
    if t > tnext + 10*dt
%       error('Behind by more than 10xdt!')
    end
  end

end