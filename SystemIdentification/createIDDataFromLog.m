function [sysdata,x0] = createIDDataFromLog(filename,start,finish,trimStart,maxT)

if nargin < 4
  trimStart = false;
end
if nargin < 5
  maxT = inf;
end

% python conversion
setenv('PYTHONPATH',strcat(pwd,'/../LCMTypes/'));
oldfile = strcat('../AcrobotLogs/',filename);
newfile = strcat('../AcrobotLogs/',filename,'.mod');
system(['python ../lcm_log_timestamps.py ' oldfile ' ' newfile ' acrobot_types']);

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

if(trimStart)
  i0 = find(y(3,:),1);
  t = t(i0:end);
  y = y(:,i0:end);
  x = x(:,i0:end);
  u = u(:,i0:end);
end


y(1:2,:) = y(1:2,:) + repmat(x(1:2,1) - y(1:2,1),1,length(t));

t = t-t(1);

outputs = y(1:2,:);
%   inputs = u;
inputs = y(3,:);

if maxT < inf
  maxN = ceil(maxT/dt);
  i0 = 1;
  i1 = min(length(t),i0+maxN-1);
  
  sysdata = {};
  x0 = {};
  
  while(i0 < length(t))
    sysdata{end+1} = iddata(outputs(:,i0:i1)',inputs(:,i0:i1)',dt);
    x0{end+1} = x(:,i0);    
    i0 = i1+1;
    i1 = min(length(t),i0+maxN-1);
  end  
else
  sysdata = {iddata(outputs',inputs',dt)};
  x0 = {x(:,1)};
end
end