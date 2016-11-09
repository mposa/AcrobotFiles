classdef sosControlExp < DrakeSystem
    % Plays back a controller from sos for real time control
    
    properties 
        
        gainMatrixStorage; % stores the gain matrix messages
        % utape; % stores the tape to play back
        
        x0lcm; % lcm object for sending data about x0
        S;
        K;
        lqr;
        xtape;
        tvControl;
    end
    
  methods
    function obj = sosControlExp(tvControl, xtape)
        % constructor for sosControl
        %
        % @param u0spl: nominal input tape (as a spline)
        % @param Kspl: spline for gain matrices
        
        obj = obj@DrakeSystem(0, 0, 4, 1);

        obj.gainMatrixStorage = LCMStorage('wingeron_gains');
        
        obj.gainMatrixStorage.storage_struct.x0 = zeros(12,1);
        obj.gainMatrixStorage.storage_struct.was_live = false;
        obj.gainMatrixStorage.storage_struct.t0 = 0;
        obj.gainMatrixStorage.storage_struct.t0_utape = 0;
        
        % obj.utape = u0spl;
        obj.xtape = xtape;
                
        obj.x0lcm = lcm.lcm.LCM.getSingleton();
        obj.tvControl = tvControl;
        
        % obj = obj.setAngleFlags([1,1,0,0],[],[]);
        
        obj.S =    1.0e+05 * [ 1.803661667644989   0.495810841591424   0.489515529518338   0.156686638355850
            0.495810841591424   0.136899616073549   0.134611582572793   0.043146186219862
            0.489515529518338   0.134611582572793   0.132872525006751   0.042534246647896
            0.156686638355850   0.043146186219862   0.042534246647896   0.013627000147556];
        
        obj.K = -[-3975.65 1265.49 346.81 347.51 109.58];
        obj.lqr = false;
        
        % set up LCM stuff
        
        lcmInFrame = LCMCoordinateFrameWCoder('acrobot_xhat', 4, 'x', AcrobotStateCoder);
        obj = obj.setInputFrame(lcmInFrame);
        
        lcmOutFrame = LCMCoordinateFrameWCoder('acrobot_u',1,'u',AcrobotInputCoder);
        obj = obj.setOutputFrame(lcmOutFrame);
        
    end

    function u = output(obj,t,junk,x)
        % Implements control function.  
        
        if (obj.gainMatrixStorage.storage_struct.t0 <= 0)
            obj.gainMatrixStorage.storage_struct.t0 = t;
        end
                        
                       
        % V0 = 390.93*x(1)^2+75.547*x(2)^2+188.78*x(2)*x(1)+23.888*x(3)^2+14.666*x(3)*x(1)+5.0599*x(3)*x(2)+9.4736*x(4)^2-1.791*x(4)*x(1)-0.93173*x(4)*x(2)+16.093*x(4)*x(3);
        
%         V0 = 407.2*x(1)^2+67.905*x(2)^2+205.76*x(2)*x(1)+17.128*x(3)^2+23.579*x(3)*x(1)+7.7175*x(3)*x(2)+7.6101*x(4)^2+6.3658*x(4)*x(1)+1.5459*x(4)*x(2)+8.8151*x(4)*x(3);
%         if (obj.gainMatrixStorage.storage_struct.t0_utape <= 0 && V0 < 9.5533)
%             obj.gainMatrixStorage.storage_struct.t0_utape = t;
%             disp('u tape fire');
%         end        
        

        if (obj.gainMatrixStorage.storage_struct.t0_utape <= 0 && obj.gainMatrixStorage.storage_struct.t0 + 5 <= t)
            obj.gainMatrixStorage.storage_struct.t0_utape = t;
            disp('u tape fire');
        end
        
        
        % get the control values
        if (obj.gainMatrixStorage.storage_struct.t0_utape > 0)
            currentT = (t - obj.gainMatrixStorage.storage_struct.t0_utape);

            % u_from_tape = ppvalSafe(obj.utape,currentT,1,0) + ppvalSafe(obj.tvControl,currentT,1,0)'*x;
           % u_from_tape = ppvalSafe(obj.utape,currentT,1,0) - ppvalSafe(obj.tvControl,currentT,1,0)'*(x - ppvalSafe(obj.xtape,currentT,1,0));
            % u_from_tape = obj.utape.eval(currentT);
            
            u_from_tape = obj.tvControl.output(currentT,[],x);
            
        else
           u_from_tape =  0;
           currentT = -10^6;
        end
        
        
        
%         % Switch to Cubic Controller
%         beta = 5.5; % 0.0015
%         
%         x = x - [pi;0;0;0];
%         
%         if V < beta %abs(x(1)) < 0.01 && abs(x(2)) < 0.05 %V < beta && ~obj.lqr
%             disp('LQR!')
%             runLCM(LQRControl, AcrobotLCMCoder, 'xhat','u');
%         end

        if currentT > 4.1809 
            % disp('Balancing Controller!')
            % Cubic controller from July 31, 2012
            % u_from_tape = -999.5125619+4.991573472312866e+002*x(1)+33.56165087045649*x(2)+58.62875908355974*x(3)-31.12069036986469*x(4)-86.42238382932517*x(1)^2+14.78544245551002*x(1)*x(2)-2.210688305156353e+002*x(2)^2+3.00460961324647*x(1)*x(3)-5.57048657504347*x(2)*x(3)-10.92203798230073*x(3)^2+31.75127018124607*x(1)*x(4)-0.04907716302400*x(2)*x(4)-7.07378102126258*x(3)*x(4)-51.65038134762852*x(4)^2+9.16969972014812*x(1)^3-2.35317625259519*x(1)^2*x(2)+70.36839428021557*x(1)*x(2)^2+4.21551635184502*x(2)^3-0.47819847200960*x(1)^2*x(3)+1.77314094769042*x(1)*x(2)*x(3)+4.22797518089256*x(2)^2*x(3)+3.47659266704119*x(1)*x(3)^2+6.13814005660811*x(2)*x(3)^2+22.20068562209788*x(3)^3-5.05337159879161*x(1)^2*x(4)+0.01562174617639*x(1)*x(2)*x(4)+5.24453445942217*x(2)^2*x(4)+2.25165443176715*x(1)*x(3)*x(4)+8.13583594039934*x(2)*x(3)*x(4)-12.92913733395560*x(3)^2*x(4)+16.44082700811302*x(1)*x(4)^2+7.85470728479244*x(2)*x(4)^2-7.16507510724451*x(3)*x(4)^2-1.13063809360844*x(4)^3;

            
            % sdsos degree 5 controller, March 7 2014
            xs = x - [pi;0;0;0];
            % u_from_tape = 3.085241046193137e+02*xs(1)+78.894454857388766*xs(2)+85.063502865577433*xs(3)+25.493209316886816*xs(4)+7.166204707076140e+05*xs(1)^3-1.694092059522372e-08*xs(1)^4+3.124894952834363e+10*xs(1)^5+1.505572393697657e+04*xs(2)^3-1.356482070497458e-10*xs(2)^4+3.578964672999017e+07*xs(2)^5+1.616056475878293e+05*xs(1)*xs(2)^2-1.815888557952056e-09*xs(1)*xs(2)^3+6.954833169297311e+08*xs(1)*xs(2)^4+5.855561467098100e+05*xs(1)^2*xs(2)-9.109957111795188e-09*xs(1)^2*xs(2)^2+5.396336990895600e+09*xs(1)^2*xs(2)^3-2.029530923190065e-08*xs(1)^3*xs(2)+2.090340502214936e+10*xs(1)^3*xs(2)^2+4.043316822369830e+10*xs(1)^4*xs(2)+1.539846817147790e+04*xs(3)^3+4.931974309491873e+07*xs(3)^5+1.660917293823335e+05*xs(1)*xs(3)^2-1.351674133036618e-09*xs(1)*xs(3)^3+8.957987250617664e+08*xs(1)*xs(3)^4+4.514569256323729e+04*xs(2)*xs(3)^2-4.086869819503977e-10*xs(2)*xs(3)^3+2.317923237973038e+08*xs(2)*xs(3)^4+5.974262427957891e+05*xs(1)^2*xs(3)-7.481881601940568e-09*xs(1)^2*xs(3)^2+6.509249509894783e+09*xs(1)^2*xs(3)^3-1.839173672349449e-08*xs(1)^3*xs(3)+2.365337191590121e+10*xs(1)^3*xs(3)^2+4.298309132338514e+10*xs(1)^4*xs(3)+4.481235100731188e+04*xs(2)^2*xs(3)-6.784457212088000e-10*xs(2)^2*xs(3)^2+4.353599884705756e+08*xs(2)^2*xs(3)^3-4.968778863495200e-10*xs(2)^3*xs(3)+4.084082061409839e+08*xs(2)^3*xs(3)^2+1.913105007434349e+08*xs(2)^4*xs(3)+3.251057046938468e+05*xs(1)*xs(2)*xs(3)-4.509519883939057e-09*xs(1)*xs(2)*xs(3)^2+3.368672415768139e+09*xs(1)*xs(2)*xs(3)^3-4.973517804053412e-09*xs(1)*xs(2)^2*xs(3)+4.746253191143750e+09*xs(1)*xs(2)^2*xs(3)^2+2.968850341211510e+09*xs(1)*xs(2)^3*xs(3)-1.657530497798979e-08*xs(1)^2*xs(2)*xs(3)+1.836210889870000e+10*xs(1)^2*xs(2)*xs(3)^2+1.725070416494405e+10*xs(1)^2*xs(2)^2*xs(3)+4.449153364749788e+10*xs(1)^3*xs(2)*xs(3)+5.655234051351333e+02*xs(4)^3+1.395852254972572e+05*xs(4)^5+1.835091453980195e+04*xs(1)*xs(4)^2+8.206698775799997e+06*xs(1)*xs(4)^4+5.074243073505824e+03*xs(2)*xs(4)^2+2.112433327966461e+06*xs(2)*xs(4)^4+5.095547491536438e+03*xs(3)*xs(4)^2+2.258944781675657e+06*xs(3)*xs(4)^4+1.985698133403360e+05*xs(1)^2*xs(4)-7.617832257962487e-10*xs(1)^2*xs(4)^2+1.929075443854598e+08*xs(1)^2*xs(4)^3-5.868445149299703e-09*xs(1)^3*xs(4)+2.266319398154092e+09*xs(1)^3*xs(4)^2+1.330795762036796e+10*xs(1)^4*xs(4)+1.515605276857142e+04*xs(2)^2*xs(4)+1.279929381779647e+07*xs(2)^2*xs(4)^3-1.591314603266962e-10*xs(2)^3*xs(4)+3.881161409404323e+07*xs(2)^3*xs(4)^2+5.890032309414017e+07*xs(2)^4*xs(4)+1.090080701496446e+05*xs(1)*xs(2)*xs(4)-4.602904030895208e-10*xs(1)*xs(2)*xs(4)^2+9.946013420453101e+07*xs(1)*xs(2)*xs(4)^3-1.590735527283208e-09*xs(1)*xs(2)^2*xs(4)+4.524432548719930e+08*xs(1)*xs(2)^2*xs(4)^2+9.155940180366488e+08*xs(1)*xs(2)^3*xs(4)-5.295007023015220e-09*xs(1)^2*xs(2)*xs(4)+1.755147233293889e+09*xs(1)^2*xs(2)*xs(4)^2+5.327979267976303e+09*xs(1)^2*xs(2)^2*xs(4)+1.375926443573687e+10*xs(1)^3*xs(2)*xs(4)+1.532782459031791e+04*xs(3)^2*xs(4)+1.461408258508427e+07*xs(3)^2*xs(4)^3-1.168322305810151e-10*xs(3)^3*xs(4)+4.724748938260919e+07*xs(3)^3*xs(4)^2+7.634070527300289e+07*xs(3)^4*xs(4)+1.103148901539584e+05*xs(1)*xs(3)*xs(4)-4.129967361641235e-10*xs(1)*xs(3)*xs(4)^2+1.061828512070622e+08*xs(1)*xs(3)*xs(4)^3-1.294267293962778e-09*xs(1)*xs(3)^2*xs(4)+5.149229583230234e+08*xs(1)*xs(3)^2*xs(4)^2+1.109295765120790e+09*xs(1)*xs(3)^3*xs(4)+3.025108613212895e+04*xs(2)*xs(3)*xs(4)-1.251922672124624e-10*xs(2)*xs(3)*xs(4)^2+2.737190319178315e+07*xs(2)*xs(3)*xs(4)^3-3.918115633461741e-10*xs(2)*xs(3)^2*xs(4)+1.329171204181210e+08*xs(2)*xs(3)^2*xs(4)^2+2.867019692554435e+08*xs(2)*xs(3)^3*xs(4)-4.775350325878331e-09*xs(1)^2*xs(3)*xs(4)+1.870923748893684e+09*xs(1)^2*xs(3)*xs(4)^2+6.045622059706093e+09*xs(1)^2*xs(3)^2*xs(4)+1.464617243226823e+10*xs(1)^3*xs(3)*xs(4)-4.340799570947583e-10*xs(2)^2*xs(3)*xs(4)+1.244930099451733e+08*xs(2)^2*xs(3)*xs(4)^2+4.033546633830675e+08*xs(2)^2*xs(3)^2*xs(4)+2.518926931531205e+08*xs(2)^3*xs(3)*xs(4)-2.881670310566767e-09*xs(1)*xs(2)*xs(3)*xs(4)+9.659184533538469e+08*xs(1)*xs(2)*xs(3)*xs(4)^2+3.125120029795469e+09*xs(1)*xs(2)*xs(3)^2*xs(4)+2.931684396471087e+09*xs(1)*xs(2)^2*xs(3)*xs(4)+1.135676424205993e+10*xs(1)^2*xs(2)*xs(3)*xs(4);
           
            % sdsos, March 11 2014
            % u_from_tape = 3.092683662056400e+02*xs(1)+77.038111245761357*xs(2)+85.107145459054749*xs(3)+25.292034299170911*xs(4)-0.001850763537079*xs(1)^2+9.041559721533586e+05*xs(1)^3+67.556199586944174*xs(1)^4-5.717178501924879e+09*xs(1)^5-1.202800236465483e-04*xs(2)^2+2.013351598481470e+04*xs(2)^3+0.280419828189594*xs(2)^4-1.188147545735491e+07*xs(2)^5-9.439191050689328e-04*xs(1)*xs(2)+2.134500800643851e+05*xs(1)*xs(2)^2+4.416931273258479*xs(1)*xs(2)^3-2.092944138193729e+08*xs(1)*xs(2)^4+7.580767050533231e+05*xs(1)^2*xs(2)+26.097729384808158*xs(1)^2*xs(2)^2-1.460208909271125e+09*xs(1)^2*xs(2)^3+68.555945540268169*xs(1)^3*xs(2)-5.034819399629677e+09*xs(1)^3*xs(2)^2-8.558413816048624e+09*xs(1)^4*xs(2)-1.409164288510118e-04*xs(3)^2+1.917564953846053e+04*xs(3)^3+0.387742786292813*xs(3)^4-9.871420719966143e+06*xs(3)^5-0.001021432467650*xs(1)*xs(3)+2.077847417226995e+05*xs(1)*xs(3)^2+5.634614866522376*xs(1)*xs(3)^3-1.760553132483272e+08*xs(1)*xs(3)^4-2.604075312270015e-04*xs(2)*xs(3)+5.817994323988785e+04*xs(2)*xs(3)^2+1.429508277432348*xs(2)*xs(3)^3-5.215266525295986e+07*xs(2)*xs(3)^4+7.506634656845980e+05*xs(1)^2*xs(3)+30.706429868927419*xs(1)^2*xs(3)^2-1.256335784075036e+09*xs(1)^2*xs(3)^3+74.374677468280481*xs(1)^3*xs(3)-4.483972007098823e+09*xs(1)^3*xs(3)^2-8.004309708538097e+09*xs(1)^4*xs(3)+5.916946700917260e+04*xs(2)^2*xs(3)+1.977055933240751*xs(2)^2*xs(3)^2-1.090602662722582e+08*xs(2)^2*xs(3)^3+1.215690688584721*xs(2)^3*xs(3)-1.130325254405313e+08*xs(2)^3*xs(3)^2-5.813471819753476e+07*xs(2)^4*xs(3)+4.199854405258084e+05*xs(1)*xs(2)*xs(3)+15.580485695627608*xs(1)*xs(2)*xs(3)^2-7.462783502517269e+08*xs(1)*xs(2)*xs(3)^3+14.365950577437598*xs(1)*xs(2)^2*xs(3)-1.173305846391590e+09*xs(1)*xs(2)^2*xs(3)^2-8.123885572591611e+08*xs(1)*xs(2)^3*xs(3)+56.606614177349897*xs(1)^2*xs(2)*xs(3)-4.005880701748737e+09*xs(1)^2*xs(2)*xs(3)^2-4.209040758605489e+09*xs(1)^2*xs(2)^2*xs(3)-9.559975531726770e+09*xs(1)^3*xs(2)*xs(3)-1.222742020817235e-05*xs(4)^2+5.996581434473917e+02*xs(4)^3+0.003155195604349*xs(4)^4-3.243065184663817e+04*xs(4)^5-3.013605036134759e-04*xs(1)*xs(4)+2.064529564492092e+04*xs(1)*xs(4)^2+0.152674872486872*xs(1)*xs(4)^3-1.828310423249227e+06*xs(1)*xs(4)^4-7.695528564745358e-05*xs(2)*xs(4)+5.792555693034788e+03*xs(2)*xs(4)^2+0.038770534248848*xs(2)*xs(4)^3-5.316073121201801e+05*xs(2)*xs(4)^4-8.320918343630851e-05*xs(3)*xs(4)+5.720859315398493e+03*xs(3)*xs(4)^2+0.042028937889304*xs(3)*xs(4)^3-5.110653065217617e+05*xs(3)*xs(4)^4+2.367222477805019e+05*xs(1)^2*xs(4)+2.770274086372007*xs(1)^2*xs(4)^2-4.111657631117783e+07*xs(1)^2*xs(4)^3+22.339982774888387*xs(1)^3*xs(4)-4.608254981140610e+08*xs(1)^3*xs(4)^2-2.572391288219291e+09*xs(1)^4*xs(4)+1.868750235542021e+04*xs(2)^2*xs(4)+0.178587601017605*xs(2)^2*xs(4)^2-3.472421842102054e+06*xs(2)^2*xs(4)^3+0.365491997729735*xs(2)^3*xs(4)-1.130365953099621e+07*xs(2)^3*xs(4)^2-1.834616664735277e+07*xs(2)^4*xs(4)+1.325994092223995e+05*xs(1)*xs(2)*xs(4)+1.406511117080552*xs(1)*xs(2)*xs(4)^2-2.405658212796424e+07*xs(1)*xs(2)*xs(4)^3+4.317736983830417*xs(1)*xs(2)^2*xs(4)-1.182095814650981e+08*xs(1)*xs(2)^2*xs(4)^2-2.572470542539929e+08*xs(1)*xs(2)^3*xs(4)+17.008128264844348*xs(1)^2*xs(2)*xs(4)-4.071910077464384e+08*xs(1)^2*xs(2)*xs(4)^2-1.338136166408522e+09*xs(1)^2*xs(2)^2*xs(4)-3.053945811465348e+09*xs(1)^3*xs(2)*xs(4)+1.815770875177208e+04*xs(3)^2*xs(4)+0.209914383690574*xs(3)^2*xs(4)^2-3.215908408381813e+06*xs(3)^2*xs(4)^3+0.465908835874129*xs(3)^3*xs(4)-1.009671279518563e+07*xs(3)^3*xs(4)^2-1.580939011571343e+07*xs(3)^4*xs(4)+1.311102403038426e+05*xs(1)*xs(3)*xs(4)+1.525127880890666*xs(1)*xs(3)*xs(4)^2-2.299448262602218e+07*xs(1)*xs(3)*xs(4)^3+5.077734255065498*xs(1)*xs(3)^2*xs(4)-1.082150386222300e+08*xs(1)*xs(3)^2*xs(4)^2-2.257528529807490e+08*xs(1)*xs(3)^3*xs(4)+3.675184228004211e+04*xs(2)*xs(3)*xs(4)+0.387166342620572*xs(2)*xs(3)*xs(4)^2-6.712107091387619e+06*xs(2)*xs(3)*xs(4)^3+1.288618105643322*xs(2)*xs(3)^2*xs(4)-3.172566065920073e+07*xs(2)*xs(3)^2*xs(4)^2-6.650776784304526e+07*xs(2)*xs(3)^3*xs(4)+18.447208025659904*xs(1)^2*xs(3)*xs(4)-3.867277711447638e+08*xs(1)^2*xs(3)*xs(4)^2-1.209238588352289e+09*xs(1)^2*xs(3)^2*xs(4)-2.879658152321668e+09*xs(1)^3*xs(3)*xs(4)+1.188459937317190*xs(2)^2*xs(3)*xs(4)-3.292768996002408e+07*xs(2)^2*xs(3)*xs(4)^2-1.039020127344144e+08*xs(2)^2*xs(3)^2*xs(4)-7.155065513040408e+07*xs(2)^3*xs(3)*xs(4)+9.362978186993132*xs(1)*xs(2)*xs(3)*xs(4)-2.272809978982730e+08*xs(1)*xs(2)*xs(3)*xs(4)^2-7.142448248346558e+08*xs(1)*xs(2)*xs(3)^2*xs(4)-7.456209041185703e+08*xs(1)*xs(2)^2*xs(3)*xs(4)-2.557658730413766e+09*xs(1)^2*xs(2)*xs(3)*xs(4);

            u_from_tape = 3.369166582238232e+02*xs(1)+84.860800662773329*xs(2)+92.809675590270459*xs(3)+27.610312604815654*xs(4)-5.962880638746313e-05*xs(1)^2+1.493835848111554e+06*xs(1)^3-4.011731367875232e-06*xs(2)^2+3.006934739964588e+04*xs(2)^3-3.093498347072163e-05*xs(1)*xs(2)+3.298893228236598e+05*xs(1)*xs(2)^2+1.212199821626445e+06*xs(1)^2*xs(2)-4.587595114594396e-06*xs(3)^2+3.105544711706477e+04*xs(3)^3-3.307997719805348e-05*xs(1)*xs(3)+3.387258005463635e+05*xs(1)*xs(3)^2-8.579765055802269e-06*xs(2)*xs(3)+9.173352707283004e+04*xs(2)*xs(3)^2+1.231888056583903e+06*xs(1)^2*xs(3)+9.078588280033885e+04*xs(2)^2*xs(3)+6.668368442984559e+05*xs(1)*xs(2)*xs(3)-3.997755309070514e-07*xs(4)^2+1.040476068365597e+03*xs(4)^3-9.766251100695169e-06*xs(1)*xs(4)+3.527180856345079e+04*xs(1)*xs(4)^2-2.532189461277414e-06*xs(2)*xs(4)+9.557481893000000e+03*xs(2)*xs(4)^2-2.709134624826603e-06*xs(3)*xs(4)+9.709161509958816e+03*xs(3)*xs(4)^2+3.979080719520360e+05*xs(1)^2*xs(4)+2.932786738866842e+04*xs(2)^2*xs(4)+2.154701600781898e+05*xs(1)*xs(2)*xs(4)+3.011950178882090e+04*xs(3)^2*xs(4)+2.189193238530271e+05*xs(1)*xs(3)*xs(4)+5.930682315167788e+04*xs(2)*xs(3)*xs(4);
            
            % % LQR controller
            % u_from_tape = -obj.K*(x-[pi;0;0;0]); 
            % runLCM(LQRControl, AcrobotLCMCoder, 'xhat','u');
        end
        
        
        
        u = u_from_tape; % add in the calibration/set point values
        
        

        % ---- do NOT add code past this line -----
        
        umax = 9.0;
        if u > umax
            u = umax;
        elseif u < -umax
            u = -umax;
        end
        
        % nump = 10000;
        % scope('acrobot','torque',t,u,struct('scope_id',1,'num_points',nump));
        
    end
    
  end
  
  
  
end