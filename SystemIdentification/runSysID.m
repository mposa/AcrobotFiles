clear all
addpath('../')
checkDependency('lcm');
javaaddpath('../LCMTypes/acrobot_types.jar')

% filenames = {'12-02-2016/lcmlog-2016-12-02.00',...
%   '12-02-2016/lcmlog-2016-12-02.01',...
%   '12-02-2016/lcmlog-2016-12-02.02',...
%   '12-02-2016/lcmlog-2016-12-02.03'};
% start = [.4;.4;.1;.4];
% % finish = [1;1;.3;1];
% finish = [.5;1;.3;1];

% filenames = {'12-07-2016/sine_1Hz_6.log',...
%   '12-07-2016/sine_1Hz_3.log',...
%   '12-07-2016/sine_2Hz_6.log',...
%   '12-07-2016/sine_0_5Hz_6.log',...
%   '12-07-2016/sine_0_5Hz_3.log'};
%   %'12-07-2016/sine_2Hz_3.log',...

% filenames = {'12-12-2016/6_sin_2_pi',...
%   '12-12-2016/6_sin_4_pi',...
%   '12-12-2016/6_sin_pi',...
%   '12-12-2016/6_sin_1_5_pi',...
%   '12-12-2016/6_sin_0_5_pi'};

filenames = {'12-12-2016/6_sin_pi','12-14-2016/swingup2.log'};
start = [0;.57];
finish = [1;.8];

  %'12-07-2016/sine_2Hz_3.log',...

% start = [.33, .2, .45, .42, .36]; %.45,
% finish = [.7, .75, .9, .88, .85]; % .85, 


% filenames = {'12-07-2016/sine_1Hz_6.log',...
%   '12-07-2016/sine_2Hz_6.log',...
%   '12-07-2016/sine_0_5Hz_6.log'};
% 
% start = [.4, .5, .42] ; %.45,
% finish = [.6,  .6, .5]; % .85, 


% filenames = {'12-08-2016/swingup_closedloop.log'};
% start = .75;
% finish = .95;



% filenames = filenames(1);

sysiddata = {};
x0 = {};
data = {};
for i=1:length(filenames),
  [sysiddatai,x0i] = createIDDataFromLog(filenames{i},start(i),finish(i),true,1);
  sysiddata = [sysiddata,sysiddatai];
  x0 = [x0,x0i];
%   file_times = start(i):.02:finish(i);
%   file_times = [start(i):.02:finish(i)];
% file_times = [start(i) finish(i)];
%   for j=1:length(file_times)-1,
%     [sysiddata{end+1},x0{end+1},data{end+1}] = createIDDataFromLog(filenames{i},file_times(j),file_times(j+1));
%   end
end

data_bkp = sysiddata;
x0_bkp = x0;

%%
sysiddata = data_bkp;
x0 = x0_bkp;


% 
% % sysiddata = sysiddata(1);
% % x0 = x0(1);
% for i=1:length(sysiddata)
% %   maxv(i) = min(max(abs(data{i}{2}.data(3:4.,:))'));
% %   meanu(i) = mean(abs(data{i}{3}.data));
%   maxdt(i) = max(diff(data{i}{3}.t));
% end


%%
% for i=1:length(sysiddata)
%   sysiddata{i}.OutputData(:,1) = wrapToPi(sysiddata{i}.OutputData(:,1));
%   sysiddata{i}.OutputData(:,2) = wrapToPi(sysiddata{i}.OutputData(:,2));
%   
%   x0{i}(1:2) = wrapToPi(x0{i}(1:2));
% end

% cut segments with low velocity and low inputs
% I = find(maxv > .5 & meanu > .05);
% remove 4,7,18,19,21,29,31,52,55,59,62,65,66,68,72
% I = setdiff(I,[4,6,7,18,19,21,29,31,52,55,59,62,65,66,68,72]);
% I = [1, 3:7, 9:length(sysiddata)];
% I = [1, 3:7];

% I = 1:length(sysiddata);
% I = setdiff(I, [3 4 8 12 13 16 23]);
% I = find(maxdt < .01);
% sysiddata=sysiddata(I);
% x0=x0(I);

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

InitParams(10).Name = 'r1';
InitParams(10).Minimum = -inf;

% from 12/5
% Parameters =     [    2.2244    0.5508    0.5134    0.8039    0.8362    0.2078    0.0390    0.8999    0.2293 0];

% from 12/8
%  Parameters =  [2.3865    0.3743    0.5690    1.3408    2.0589    0.0547    0.0288    1.6038    0.4172   -0.0243*0];
 
 % From 12/12
 Parameters = [    2.4367    0.6178    0.5263    1.6738    1.5651    0.0320    0.0413    2.0824    0.5065    0.0011];
  
for i=1:length(InitParams),
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
nlgr = pem(z, nlgr, 'Display', 'Full','MaxIter',100);

% figure;
% compare(getexp(z,1), nlgr);
% plot(sysiddata{56}.InputData)
% plot(sysiddata{56}.OutputData)

%%
p = nlgr.Report.Parameters;
[p.InitialValues.ParVector, p.ParVector]
% % Test on other data
% ztest = merge(z3,z4,z5); %iddata(outputs2,torque2,dt);
% figure;
% compare(ztest,nlgr);

% 8,12, 39, 41, 44, 45, 49, 51, 53, 55, 80
%%
i = i+1;
t_offset = .0;
plant = AcrobotPlantSmooth(p.ParVector);
% plant = AcrobotPlantSmooth();
experiment = getexp(z,i);
t_exp = experiment.Ts*(0:length(experiment.InputData)-1);
utraj = PPTrajectory(foh([-t_offset t_exp]+t_offset,[0 experiment.InputData']));
utraj = utraj.setOutputFrame(plant.getInputFrame);
plant_ol = utraj.cascade(plant);
% traj = plant_ol.simulate([0 t_exp(end)],x0{i});
traj = plant_ol.simulate([0 t_exp(end)],nlgr.Report.Parameters.X0(:,i));

x_exp = traj.eval(t_exp);
figure(2)
subplot(2,1,1)
plot(t_exp,experiment.OutputData(:,1),t_exp,x_exp(1,:))
title(i)
subplot(2,1,2)
plot(t_exp,experiment.OutputData(:,2),t_exp,x_exp(2,:))
legend('exp','fit')