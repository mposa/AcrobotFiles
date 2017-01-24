xtraj_list = {};
utraj_list = {};
F_list = [];

xtraj_list_alt = {};
utraj_list_alt = {};
F_list_alt = [];

%%
while length(xtraj_list) < 100
  [p,v,xtraj,utraj,z,F,info,traj_opt] = smoothSwingUp();
  if (info<10), 
%     v.playback(xtraj); 
    xtraj_list{end+1} = xtraj;
    utraj_list{end+1} = utraj;
    F_list = [F_list, F];
    display(sprintf('Found %d trajectories so far',length(xtraj_list)));
    
    [p,v,xtraj,utraj,z,F,info,traj_opt] = smoothSwingUp(xtraj,utraj,true);
    if info < 10
      xtraj_list_alt{length(xtraj_list)} = xtraj;
      utraj_list_alt{length(xtraj_list)} = utraj;
      F_list_alt(length(xtraj_list)) = F;
    end
  end
end

%%
for i = 1:length(xtraj_list)
  xtraj_list{i} = xtraj_list{i}.setOutputFrame(v.getInputFrame);
  if i <= length(xtraj_list_alt) && ~isempty(xtraj_list_alt{i})
    xtraj_list_alt{i} = xtraj_list_alt{i}.setOutputFrame(v.getInputFrame);
  end
end

%%
for i = 1:length(xtraj_list)
  ts = linspace(0,xtraj_list{i}.tspan(2),1e4);
  xdot = deriv(xtraj_list{I},ts);
  xdotmax(i) = max(abs(xdot(4,:)));
  xdotsq(i) = sqrt(sum(xdot(4,:).*xdot(4,:))/length(ts));
end
