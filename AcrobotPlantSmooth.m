classdef AcrobotPlantSmooth < RigidBodyManipulator
  % Defines the dynamics for the real acrobot
  % Updated on Jan 20, 2014.
  %#ok<*PROPLC>
  properties
  end
  
  methods
    function obj = AcrobotPlantSmooth(params)
      obj = obj@RigidBodyManipulator('AcrobotParams.urdf');
      
%       obj = obj@Manipulator(2,1,2);
      obj = obj.setOutputFrame(obj.getStateFrame);
      
      if nargin < 1
        % July 26
%         params = [2.2244 0.5508 0.5134 0.8039 0.8362 0.2078 0.0390 0.8999 0.2293 0];
        % December 7, 2016 (mposa)
        % with masses
%         params =  [1.8719    0.1486    0.5131    1.0536    3.0806    0.2048    0.0391    0.9953    0.2282 0]';
        
%         params = [    2.6631    0.2732    0.5407    1.2449    2.9417    0.2770    0.1174    1.7551    0.4087    0.0101]';
        
        % December 12
        params = [    2.4367    0.6178    0.5263    1.6738    1.5651    0.0320    0.0413    2.0824    0.5065    0.0011]';
        
        % Dec 14
%         params =  [2.0253    0.3672    0.4410    1.8500    2.8088    0.0203    0.0499    1.8449    0.4790    0.0129]';

      end            
      
      % adjust inertias to match Ani's model
      params(8) = params(8) - params(1)*params(4)^2;
      params(9) = params(9) - params(2)*params(5)^2;
      obj = obj.setParams(params);
    end    
        
    function x = getInitialState(obj)
      x = [0 0 0 0]';
    end
    
    function [q0,q1] = calculateEquilibria(obj)
      function y = f0(q)
        qdd = obj.dynamics(0,[q;0;0],0);
        y = qdd(3)^2 + qdd(4)^2;
      end
      opt = optimset('Display','off');
      q0 = fminunc(@f0,[0;0],opt);      
      q1 = fminunc(@f0,[pi;0],opt);    
    end    
  end
  
end






