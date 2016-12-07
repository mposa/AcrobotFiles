classdef AcrobotVisualizer < Visualizer
% Implements the draw function for the Acrobot 

  methods
    function obj = AcrobotVisualizer(plant)
      % Construct visualizer
      %   AcrobotVisualizer(AcrobotPlant) will take the necessary
      %   parameters from the plant class
      
      typecheck(plant,'AcrobotPlantSmooth');
      obj = obj@Visualizer(plant.getOutputFrame);
%       obj.l1 = plant.l1;
%       obj.l2 = plant.lc2;
      obj.l1 = 1;
      obj.l2 = 1;
      
      
      av = pi/2*[1:.05:3];
      r = .04*min([obj.l1 obj.l2]);
      L1x = [r*cos(av) obj.l1+r*cos(av+pi)];
      L1y = [r*sin(av) r*sin(av+pi)];
      obj.L1r = (L1x.^2+L1y.^2).^.5;
      obj.L1a = atan2(L1y,L1x);
      L2x = [r*cos(av) obj.l2+r*cos(av+pi)];
      L2y = [r*sin(av) r*sin(av+pi)];
      obj.L2r = (L2x.^2+L2y.^2).^.5;
      obj.L2a = atan2(L2y,L2x);
    end
    
    function draw(obj,t,x)
      % draw the acrobot
     
      patch(obj.L1r.*sin(obj.L1a+x(1)),-obj.L1r.*cos(obj.L1a+x(1)),0*obj.L1a,'r');
      hold on
      patch(obj.l1*sin(x(1))+obj.L2r.*sin(obj.L2a+x(1)+x(2)),-obj.l1*cos(x(1))-obj.L2r.*cos(obj.L2a+x(1)+x(2)),1+0*obj.L2a,'b');
      plot3(0,0,2,'k+');
      axis image
      view(0,90)
      set(gca,'XTick',[],'YTick',[])
      axis((obj.l1+obj.l2)*1.1*[-1 1 -1 1 -1 1000]);
    end
  end

  properties
    l1=1;    % length of link 1
    l2=2;    % length of link 2
    L1r;
    L1a;
    L2r;
    L2a;
  end
  
end
