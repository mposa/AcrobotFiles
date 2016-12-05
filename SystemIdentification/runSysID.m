clear all
addpath('../')

filenames = {'12-02-2016/lcmlog-2016-12-02.00',...
  '12-02-2016/lcmlog-2016-12-02.01',...
  '12-02-2016/lcmlog-2016-12-02.02',...
  '12-02-2016/lcmlog-2016-12-02.03'};
start = [.4;.4;.1;.4];
% finish = [1;1;.3;1];
finish = [.5;1;.3;1];

% filenames = filenames(1);

sysiddata = {};
x0 = {};
data = {};
for i=1:length(filenames),
  file_times = start(i):.02:finish(i);
  for j=1:length(file_times)-1,
    [sysiddata{end+1},x0{end+1},data{end+1}] = createIDDataFromLog(filenames{i},file_times(j),file_times(j+1));
  end
end

data_bkp = sysiddata;
x0_bkp = x0;


%%
for i=1:length(sysiddata)
  sysiddata{i}.OutputData(:,1) = wrapToPi(sysiddata{i}.OutputData(:,1));
  sysiddata{i}.OutputData(:,2) = wrapToPi(sysiddata{i}.OutputData(:,2));
  
  x0{i}(1:2) = wrapToPi(x0{i}(1:2));
end

% remove 4,7,18,19,21,29,31,52,55,59,62,65,66,68,72
I = setdiff(1:length(sysiddata),[4,6,7,18,19,21,29,31,52,55,59,62,65,66,68,72]);
% I = [1, 3:7, 9:length(sysiddata)];
% I = [1, 3:7];

sysiddata=sysiddata(I);
x0=x0(I);

%%
z = merge(sysiddata{:});
FileName = 'AcrobotIDModel';
Order = [2, 1, 4];
% from Ani
% Parameters =     [2.1512;
%     0.9077;
%     0.5019;
%     0.4753;
%     0.5528;
%     0.2644;
%     0.0529;
%     0.5324;
%     0.2376];


InitParams(1).Name = 'm1';
InitParams(1).Minimum = .01;

InitParams(2).Name = 'm2';
InitParams(2).Minimum = .01;

InitParams(3).Name = 'l1';
InitParams(3).Minimum = .01;

InitParams(4).Name = 'lc1';
InitParams(4).Minimum = .01;

InitParams(5).Name = 'lc2';
InitParams(5).Minimum = .01;

InitParams(6).Name = 'b1';
InitParams(6).Minimum = 0;

InitParams(7).Name = 'b2';
InitParams(7).Minimum = 0;

InitParams(8).Name = 'I1';
InitParams(8).Minimum = .01;

InitParams(9).Name = 'I2';
InitParams(9).Minimum = .01;
  
% from 12/5
Parameters =     [    2.2244    0.5508    0.5134    0.8039    0.8362    0.2078    0.0390    0.8999    0.2293];
  
for i=1:9,
  InitParams(i).Value = Parameters(i);
  InitParams(i).Maximum = inf;
  InitParams(i).Unit = '';
  InitParams(i).Fixed = false;
end

% InitialStates = [0;0;0;0];
Ts = 0; % continuous model

clear InitialStates
InitialStates(1).Name = 'q1';
InitialStates(2).Name = 'q2';
InitialStates(3).Name = 'v1';
InitialStates(4).Name = 'v2';
for i=1:4,
  InitialStates(i).Unit = '';
  InitialStates(i).Minimum = -inf;
  InitialStates(i).Maximum = inf;
end
InitialStates(1).Fixed = true;
InitialStates(2).Fixed = true;
InitialStates(3).Fixed = false;
InitialStates(4).Fixed = false;

for i=1:length(sysiddata),
  InitialStates(1).Value(i) = x0{i}(1);
  InitialStates(2).Value(i) = x0{i}(2);
  InitialStates(3).Value(i) = x0{i}(3);
  InitialStates(4).Value(i) = x0{i}(4);
end

nlgr = idnlgrey(FileName, Order, InitParams, InitialStates, Ts);
%%
nlgr = pem(z, nlgr, 'Display', 'Full','MaxIter',10);

figure;
compare(getexp(z,1), nlgr);
% plot(sysiddata{56}.InputData)
% plot(sysiddata{56}.OutputData)

%%
p = nlgr.Report.Parameters;

% % Test on other data
% ztest = merge(z3,z4,z5); %iddata(outputs2,torque2,dt);
% figure;
% compare(ztest,nlgr);