function [sysdata,x0, data] = createIDDataFromLog(filename,start,finish)
  % python conversion
  setenv('PYTHONPATH',strcat(pwd,'/../LCMTypes/'));
  oldfile = strcat('../AcrobotLogs/',filename);
  newfile = strcat('../AcrobotLogs/',filename,'.mod');
  system(['python ../AcrobotLogs/lcm_log_timestamps.py ' oldfile ' ' newfile ' acrobot_types']);

  channels = {'acrobot_y','acrobot_xhat','acrobot_u','acrobot_out'};
  coders = {AcrobotYCoder(),AcrobotStateCoder(),AcrobotInputCoder(),AcrobotOutCoder()};
  data = readLog(newfile,channels,coders,start,finish);
  
  dt = .005;
  t = unique(data{1}.t);
  t = t(1):dt:t(end);
  
  [~,Ix] = unique(data{2}.t);
  x = data{2}.data(:,Ix);
  
  [~,Iu] = unique(data{3}.t);
  u = data{3}.data(:,Iu);
  
  [~,Iy] = unique(data{1}.t);
  y = data{1}.data(:,Iy);
  
  
  y = interp1(data{1}.t(Iy),y',t,'linear','extrap')';
  x = interp1(data{2}.t(Ix),x',t,'linear','extrap')';
  u = interp1(data{3}.t(Iu),u,t,'linear','extrap');
  
  
  y(1:2,:) = y(1:2,:) + repmat(x(1:2,1) - y(1:2,1),1,length(t));
  
  t = t-t(1);
  
  outputs = x(1:2,:);
  inputs = u;
  
  sysdata = iddata(outputs',inputs',dt);
  x0 = x(:,1);
end