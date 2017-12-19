% see rd3_vfa_sim_script.m for description
classdef SimVFA < handle
    properties
        simOpt    % struct based on opt
        simInObj  % object to pass to simulink
        simOutObj % object returned from simulink
        trimPts   % trim points
        pltOpt    % struct for plotting options
    end
    methods
        function vfa = SimVFA(opt) % constructor
            
            vfa.setSimOpts(opt);
            vfa.setTrimOpts();
            vfa.setPlotOpts();
            
            vfa.simInObj = Simulink.SimulationInput('rd3_mimo_sim_max');
            
            vfa.trimAC(~opt.reTrim, opt.reTrim, false);
            vfa.genLookupTables(~opt.reTrim, opt.reTrim);
            vfa.genController();
            vfa.populateSimIn();
            
        end
        
        function setSimOpts(vfa, opt)
            
            SO = vfa.simOpt; % shorthand
            
            % data save/load path
            if (isfield(opt, 'dataPath') && ~isempty(opt.dataPath))
                SO.dataPath = opt.dataPath;
            else
                SO.dataPath = [];
            end
            
            % flag for adaptive control on/off
            if (isfield(opt, 'adaFlag') && ~isempty(opt.adaFlag))
                SO.adaFlag = logical(opt.adaFlag);
            else
                SO.adaFlag = true;
            end
            
            % flag for uncertainty on/off
            if (isfield(opt, 'uncertFlag') && ~isempty(opt.uncertFlag))
                SO.uncertFlag = logical(opt.uncertFlag);
            else
                SO.uncertFlag = true;
            end
            
            % specification for actuator mode (slow/fast/nominal)
            if (isfield(opt, 'actMode') && ~isempty(opt.actMode))
                SO.actMode = opt.actMode;
            else
                SO.actMode = 'Slow';
            end
            
            % actuator parameters
            if SO.uncertFlag
                if strfind(SO.actMode, 'Fast')
                    SO.w_act_a    = 1.5;
                    SO.zeta_act_a = 0.7; % best for fast act
                    SO.lambda_s   = 0.1;
                    SO.Psi1_scale = 0.005;
                    SO.Psi2_scale = 0.0001;
                    SO.Psi3_scale = 0.005;
                elseif strfind(SO.actMode, 'Slow')
                    SO.w_act_a    = 0.5;
                    SO.zeta_act_a = 2;
                    SO.lambda_s   = 0.2; % good for intp
                    SO.Psi1_scale = 0.005;
                    SO.Psi2_scale = 0.0001;
                    SO.Psi3_scale = 0.005;   
                else
                    SO.w_act_a    = 1;
                    SO.zeta_act_a = 0.7;
                    SO.lambda_s   = 0.2;
                    SO.Psi1_scale = 0.005;
                    SO.Psi2_scale = 0.0001;
                    SO.Psi3_scale = 0.005;  
                end
            else
                SO.w_act_a    = 1;
                SO.zeta_act_a = 0.7;
                SO.lambda_s   = 1;
                SO.Psi1_scale = 0;
                SO.Psi2_scale = 0;
                SO.Psi3_scale = 0;    
            end

            SO.i_state_sel = [1:2,4:7];
            SO.i_dihedral  = 6;     % dihedral angle
            SO.i_input_sel = [2,3]; % outer aileron, center elevator
            SO.i_output    = 5;     % pitch rate

            SO.postcomp = 1; % post-compensator version (1, 2, 3)
            SO.q0 = 1.0; SO.epsilon = 30; %for post-compensator_v3 %best design
            % q0 used in P0 = lyap(NAM', q0*eye[])
            SO.s0 = []; % s0 used in A_eta = A+s0*eye[]

            SO.a22 = 1; SO.a21 = 2; SO.a20 = 1; % second-order filter coefficients
            SO.d22 = 1; SO.d21 = 2; SO.d20 = 1; % derivative coefficieints
            SO.d11 = 1; SO.d10 = 1; % derivative coefficients
            SO.d00 = 1; % derivative coefficients

            SO.Psi1 = [0,10,0,0,-5,-10,0,0,0,0,0,0;
                    0,-10,0,0,-10,50,0,0,0,0,0,0]; %excellent
            SO.Psi2 = [0,10,0,0,-5,-10,0,0,0,0,0,0;
                    0,-10,0,0,-10,50,0,0,0,0,0,0]; %excellent
            SO.Psi3 = [0,10,0,0,-5,-10,2.2,5,0.7,0,0,0;
                    0,10,0,0,2,30,10,0.8,4,0,0,0]; %excellent

            % scale uncertainty matrices
            SO.Psi1_s = SO.Psi1_scale * SO.Psi1;
            SO.Psi2_s = SO.Psi2_scale * SO.Psi2;
            SO.Psi3_s = SO.Psi3_scale * SO.Psi3;

            SO.rho_Ls = 5;

            % for basic LQR
            SO.rk = 50*eye(length(SO.i_input_sel));
            SO.qk = diag([0.001,0.001,0.001,0.001,0.001,0.001,0.0001,0.0001,0.0001,0.0001,1,0.01]);

            % second order actuator dynamics (nominal)
            SO.w_act    = 1;   % natural frequency
            SO.zeta_act = 0.7; % damping

            % command filter coefficients
            SO.zeta_cmd = 2; SO.w_cmd = 0.5;

            SO.data_deci = 1; % decimation for saved data from Simulink
            
            SO.eta_nom = 10; % select dihedral angle (deg) [== ind-1] from linearized tables

            % command
            r_step = [-0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5];
            r_step_scaled = [-2*pi/180; 2] * r_step;

            % simulation times for command steps
            t_step = 70;
            tsim = t_step*(length(r_step)-1)+1;
            r_timeseries = timeseries(r_step_scaled, 1:t_step:tsim);
            SO.r_timeseries = setinterpmethod(r_timeseries,'zoh');
            SO.tsim = tsim;
            
            vfa.simOpt = SO;  % save updated struct
        end
        
        function setTrimOpts(vfa)
        % trimming options
            % data: [w; x; y; z] (used for inertial properties of VFA)
            vfa.trimPts.data = [300; 30; 20/10; 18];
            vfa.trimPts.Vinitial = 30;     % airspeed (ft/s)
            vfa.trimPts.hinitial = 40000;  % altitude (ft)
            vfa.trimPts.alphainitial = 7.5*pi/180; % angle of attack (rad)
            vfa.trimPts.thetainitial = 7.5*pi/180; % pitch angle guess (rad)
            vfa.trimPts.etasweep = (0:45)'; % dihedral angles to linearize/trim (deg)
        end
        
        function setPlotOpts(vfa)
        % plotting options
            vfa.pltOpt.legfontsize=12;
            vfa.pltOpt.fontsize=14;
            vfa.pltOpt.weight='n';
            vfa.pltOpt.fontname='Times New Roman';
        end
        
        function trimAC(vfa, loadTrim, saveTrim, pltTrim)
        % find trim with zero dihedral rate (eta_dot=0), or load from file

            saveFile = [vfa.simOpt.dataPath, 'VFA_trim_data'];
            
            if loadTrim
            	vfa.trimPts = load(saveFile);                
            else
                % 7 states: V, alpha, h, theta, q, eta, etadot
                % 9 inputs: thrust, delta2, delta1, deltat2, deltat1, V2dis_X, ...
                %           V2dis_Z, V3dis_X, V3dis_Z
                
                TP = vfa.trimPts; % just for shorthand - will save struct at end of function
                
                assignin('base', 'stateinit', zeros(7,1));
                assignin('base', 'data', TP.data);
                oper_spec = operspec('VFA_lin');

                for i=min(TP.etasweep):max(TP.etasweep)

                    etainitial = i*pi/180;

                    stateguess  = [TP.Vinitial TP.alphainitial+i/600 TP.hinitial TP.thetainitial 0 etainitial 0]';
                    inputguess  = [2000 0.1 (15+i/3)*pi/180 (5-i/10)*pi/180 -0.1 0 0 0 0]';
                    statesknown = [1 1 1 0 1 1 1]';
                    inputknown  = [0 0 0 1 0 1 1 1 1]';

                    %%%%%% setting parameters %%%%%%%%%%%%%
                    set(oper_spec.States,'known',statesknown,'x',stateguess);
                    set(oper_spec.States,'Min',[10 0 0 0 -2 0 -2]');
                    set(oper_spec.States,'Max',[80 10*pi/180 100000 30*pi/180 2 60*pi/180 2]');
                    set(oper_spec.Inputs,'known',inputknown,'u',inputguess,'Max',[2000 1 1 0 0  0 0 0 0]');
                    set(oper_spec.Inputs,'u',inputguess,'Min',[0 -1 -1 -1 -1 0 0 0 0]');
                    %set(oper_spec.Inputs(2),'u',inputguess(2),'Max',1);
                    %set(oper_spec.Inputs(3),'u',inputguess(3),'Max',1);

                    [op_point,~] = findop('VFA_lin',oper_spec);
                    linsys = linearize('VFA_lin',op_point);

                    TP.A_hold(1:7,1:7,i+1) = linsys.A; % linearized A matrices
                    TP.B_hold(1:7,1:9,i+1) = linsys.B; % linearized B matrices

                    % real part of linearized system poles
                    TP.trim_eig_real(i+1,:) = real(eig(linsys.A)); 
                    % imaginary part of linearized system poles
                    TP.trim_eig_imag(i+1,:) = imag(eig(linsys.A));
                    % inputs
                    TP.trim_inputs(i+1,:)   = op_point.Inputs.u;
                    % states
                    TP.trim_states(i+1,:)   = op_point.States.x;

                    % rank of controllability matrix w/3 inputs
                    TP.rankhold(i+1) = rank(ctrb(linsys.A,linsys.B(1:7,[1 2 4])));

                    % test different subsets of B for ctrb, obsv:
                    Co = ctrb(linsys.A,linsys.B(1:7, [1 2]));
                    % ratio of the smallest singular value of Co to the largest
                    TP.sv_hold_1(i+1) = 1/cond(Co);

                    Co = ctrb(linsys.A,linsys.B(1:7,[1 2 4]));
                    % ratio of the smallest singular value of Co to the largest
                    TP.sv_hold_2(i+1) = 1/cond(Co);

                    Co = ctrb(linsys.A,linsys.B(1:7, [1 2 3 4]));
                    % ratio of the smallest singular value of Co to the largest
                    TP.sv_hold_3(i+1) = 1/cond(Co);

                    Co = obsv(linsys.A,[1 zeros(1,6); 0 0 1 0 0 0 0; 0 0 0 0 1 0 0]);
                    % ratio of the smallest singular value of Co to the largest
                    TP.sv_hold_4(i+1) = 1/cond(Co);

                    Co = ctrb(linsys.A,linsys.B(1:7, 1:5));
                    % ratio of the smallest singular value of Co to the largest
                    TP.sv_hold_5(i+1) = 1/cond(Co);

                    Co = obsv(linsys.A,eye(7,7));
                    % ratio of the smallest singular value of Co to the largest
                    TP.sv_hold_6(i+1) = 1/cond(Co);
                    
                    % update the saved struct
                    vfa.trimPts = TP;
                end

                
                if saveTrim
                    % save fields of tp_ret struct in .mat file
                    save(saveFile,'-struct','tp_ret');
                end
                
                % clear data and stateinit from base workspace
                evalin( 'base', 'clear data stateinit');

            end
            
            if pltTrim
                vfa.plotTrim();
            end
        end
                
        function populateSimIn(vfa)
        % collect everything which needs to be passed to simulink and
        % populate the Simulink.SimulinkInput object with these
            SI = vfa.simInObj;
            SO = vfa.simOpt;
            TP = vfa.trimPts;
            
%             arr_so = {'Aa', 'Aact', 'Acmd', 'Ada_Flag', 'Apsim', 'Ba', 'Bact', ...
%                       'Bact_x', 'Bcmd', 'Bm', 'Bp', 'Ca', 'Cact', 'Cact_dot', ...
%                       'Caz', 'Ccmd', 'Ccmd_dot', 'Cm', 'Cov', 'Cz', 'Dz', ...
%                       'Gamma_l', 'Gamma_p1', 'Gamma_p2', 'Gamma_p3', ...
%                       'Gamma_p31', 'Gamma_p31xm', 'Gamma_p32', 'Gamma_vl', ...
%                       'Lambda_s', 'Las', 'Si1', 'Si3', 'Ts', 'a20', 'a21', ...
%                       'a22', 'd00', 'd10', 'd11', 'd20', 'd21', 'd22', ...
%                       'd_mean', 'd_sam', 'd_seed', 'data_deci', ...
%                       'i_input_sel', 'i_state_sel', 'inputselect', ...
%                       'lambda_0', 'mu_lambda',  'mu_psi1', 'mu_psi2', ...
%                       'mu_psi3', 'mu_psi31', 'mu_psi32', 'n_mean', 'n_sam', ...
%                       'n_seed', 'psi1_0', 'psi2_0', 'psi31_0', 'psi31xm_0', ...
%                       'psi32_0', 'psi3_0', 'r_timeseries', 'samplet', 'seed', ...
%                       'tpower', 'tsim', 'vlambda_0', 'x_red_est_0', ...
%                       'input_hold', 'state_hold'};
            
            SI = SI.setVariable('Aa', SO.Aa);
            SI = SI.setVariable('Aact', SO.Aact);
            SI = SI.setVariable('Acmd', SO.Acmd);
            SI = SI.setVariable('Ada_Flag', int8(SO.adaFlag));
            SI = SI.setVariable('Apsim', SO.Apsim);
            SI = SI.setVariable('Ba', SO.Ba);
            SI = SI.setVariable('Bact', SO.Bact);
            SI = SI.setVariable('Bact_x', SO.Bact_x);
            SI = SI.setVariable('Bcmd', SO.Bcmd);
            SI = SI.setVariable('Bm', SO.Bm);
            SI = SI.setVariable('Bp', SO.Bp);
            SI = SI.setVariable('Ca', SO.Ca);
            SI = SI.setVariable('Cact', SO.Cact);
            SI = SI.setVariable('Cact_dot', SO.Cact_dot);
            SI = SI.setVariable('Caz', SO.Caz);
            SI = SI.setVariable('Ccmd', SO.Ccmd);
            SI = SI.setVariable('Ccmd_dot', SO.Ccmd_dot);
            SI = SI.setVariable('Cm', SO.Cm);
            SI = SI.setVariable('Cz', SO.Cz);
            SI = SI.setVariable('Dz', SO.Dz);
            SI = SI.setVariable('Gamma_l', SO.Gamma.l);
            SI = SI.setVariable('Gamma_p1', SO.Gamma.p1);
            SI = SI.setVariable('Gamma_p2', SO.Gamma.p2);
            SI = SI.setVariable('Gamma_p3', SO.Gamma.p3);
            SI = SI.setVariable('Gamma_p31', SO.Gamma.p31);
            SI = SI.setVariable('Gamma_p31xm', SO.Gamma.p31xm);
            SI = SI.setVariable('Gamma_p32', SO.Gamma.p32);
            SI = SI.setVariable('Gamma_vl', SO.Gamma.vl);
            SI = SI.setVariable('Lambda_s', SO.Lambda_s);
            SI = SI.setVariable('Las', SO.Las);
            SI = SI.setVariable('Si1', SO.Si1);
            SI = SI.setVariable('Si3', SO.Si3);

            SI = SI.setVariable('a20', SO.a20);
            SI = SI.setVariable('a21', SO.a21);
            SI = SI.setVariable('a22', SO.a22);
            SI = SI.setVariable('d00', SO.d00);
            SI = SI.setVariable('d10', SO.d10);
            SI = SI.setVariable('d11', SO.d11);
            SI = SI.setVariable('d20', SO.d20);
            SI = SI.setVariable('d21', SO.d21);
            SI = SI.setVariable('d22', SO.d22);
            SI = SI.setVariable('d_mean', SO.d_mean);
            SI = SI.setVariable('d_sam', SO.d_sam);
            SI = SI.setVariable('d_seed', SO.d_seed);
            SI = SI.setVariable('data_deci', SO.data_deci);
            SI = SI.setVariable('i_input_sel', SO.i_input_sel);
            SI = SI.setVariable('i_state_sel', SO.i_state_sel);
            SI = SI.setVariable('inputselect', SO.inputselect);
            SI = SI.setVariable('lambda_0', SO.lambda_0);
            SI = SI.setVariable('mu_lambda', SO.mu.lambda);
            SI = SI.setVariable('mu_psi1', SO.mu.psi1);
            SI = SI.setVariable('mu_psi2', SO.mu.psi2);
            SI = SI.setVariable('mu_psi3', SO.mu.psi3);
            SI = SI.setVariable('mu_psi31', SO.mu.psi31);
            SI = SI.setVariable('mu_psi32', SO.mu.psi32);
            SI = SI.setVariable('n_mean', SO.n_mean);
            SI = SI.setVariable('n_sam', SO.n_sam);
            SI = SI.setVariable('n_seed', SO.n_seed);
            SI = SI.setVariable('psi1_0', SO.psi1_0);
            SI = SI.setVariable('psi2_0', SO.psi2_0);
            SI = SI.setVariable('psi31_0', SO.psi31_0);
            SI = SI.setVariable('psi31xm_0', SO.psi31xm_0);
            SI = SI.setVariable('psi32_0', SO.psi32_0);
            SI = SI.setVariable('psi3_0', SO.psi3_0);
            SI = SI.setVariable('r_timeseries', SO.r_timeseries);
            SI = SI.setVariable('samplet', SO.samplet);
            SI = SI.setVariable('tpower', SO.tpower);
            SI = SI.setVariable('tsim', SO.tsim);
            SI = SI.setVariable('vlambda_0', SO.vlambda_0);
            SI = SI.setVariable('x_red_est_0', SO.x_red_est_0);
            SI = SI.setVariable('input_hold', SO.input_hold);
            SI = SI.setVariable('state_hold', SO.state_hold);
            SI = SI.setVariable('initstate', SO.state_hold);

%             arr_st = {'Aa_M', 'Ba3_aug_M','Ba32_aug_M','Ba3_M','Ba32_M', ...
%                       'Kr_M','Las_M','Ras_M','Sas_M','Sas1_M','data', ...
%                       'etasweep', 'initstate'};

            SI = SI.setVariable('Aa_M', TP.tables.Aa_M);
            SI = SI.setVariable('Ba3_aug_M', TP.tables.Ba3_aug_M);
            SI = SI.setVariable('Ba32_aug_M', TP.tables.Ba32_aug_M);
            SI = SI.setVariable('Ba3_M', TP.tables.Ba3_M);
            SI = SI.setVariable('Ba32_M', TP.tables.Ba32_M);
            SI = SI.setVariable('Kr_M', TP.tables.Kr_M);
            SI = SI.setVariable('Las_M', TP.tables.Las_M);
            SI = SI.setVariable('Ras_M', TP.tables.Ras_M);
            SI = SI.setVariable('Sas_M', TP.tables.Sas_M);
            SI = SI.setVariable('Sas1_M', TP.tables.Sas1_M);
            SI = SI.setVariable('data', TP.data);
            SI = SI.setVariable('etasweep', TP.etasweep);

            vfa.simInObj = SI; % save updated object
        end
        
        function genLookupTables(vfa, loadTables, saveTables)
        % compute and save linearized matrices for VFA sim
            TP = vfa.trimPts; % to be saved at end of function
            SO = vfa.simOpt;
            
            saveFile = [SO.dataPath, 'eta_swept_data_lookup'];

            if loadTables
            	tables = load(saveFile);                
            else
                % Lookup tables for matrices vs. dihedral angle 
                Aa_M = zeros([12,12,length(TP.etasweep)]);
                Ba_M = zeros([12,2,length(TP.etasweep)]);
                Ba3_aug_M  = zeros([12,3,length(TP.etasweep)]);
                Ba32_aug_M = zeros([12,3,length(TP.etasweep)]);
                Ba3_M  = zeros([12,2,length(TP.etasweep)]);
                Ba32_M = zeros([12,2,length(TP.etasweep)]);
                Kr_M   = zeros([2,12,length(TP.etasweep)]);
                Las_M  = zeros([12,3,length(TP.etasweep)]);
                Ras_M  = zeros([3,3,length(TP.etasweep)]);
                Sas_M  = zeros([3,3,length(TP.etasweep)]);
                Sas1_M = zeros([2,3,length(TP.etasweep)]);
                
                for eta_i = (min(TP.etasweep):max(TP.etasweep))
                    % Set plant parameters
                    eta_hold = eta_i; % select dihedral angle (deg) [== ind-1] from linearized tables

                    Ap = TP.A_hold(:,:,eta_hold+1);
                    B_c_hold = TP.B_hold(:,1:5,:);
                    Bp = B_c_hold(:,:,eta_hold+1);

                    % increase dimension (w/zeros) of controls fed to VFA sim model
                    Cp = [eye(6), zeros(6,1)]; % take first six of seven states

                    % augment plant with (nominal) actuator dynamics
                    A = [Ap(SO.i_state_sel,SO.i_state_sel), Bp(SO.i_state_sel,SO.i_input_sel), zeros(length(SO.i_state_sel),length(SO.i_input_sel));
                         zeros(length(SO.i_input_sel),length(SO.i_state_sel)+length(SO.i_input_sel)), eye(length(SO.i_input_sel));
                         zeros(length(SO.i_input_sel),length(SO.i_state_sel)), -SO.w_act^2*eye(length(SO.i_input_sel)), -2*SO.zeta_act*SO.w_act*eye(length(SO.i_input_sel))];
                    B = [0*Bp(SO.i_state_sel,SO.i_input_sel);
                         zeros(length(SO.i_input_sel));
                         SO.w_act^2*eye(length(SO.i_input_sel))];

                    C  = [Cp(SO.i_output,SO.i_state_sel), zeros(length(SO.i_output), 2*length(SO.i_input_sel))];
                    Cz = [0,0,0,0,1,0,0,0,0,0;
                          0,TP.Vinitial*Ap(2,2),0,0,0,0,TP.Vinitial*Bp(2,SO.i_input_sel),0,0];

                    num_input  = size(B,2);

                    % Introduce new coordinates
                    % integrate the tracking output to make the system strictly proper
                    index_output = [1,2];

                    % this sys used in RM: 
                    % Am (not in paper) = Aa - Ba*Kr,   Bm = Baz,   Cm (not in paper) = Ca
                    Aa = [A,zeros(length(A),length(index_output));
                          Cz,zeros(length(index_output))];           % "A" in paper
                    Ba = [B; zeros(length(index_output),num_input)]; % "B_3" in paper

                    Ca  = [C, zeros(size(C,1),length(index_output));
                           zeros(length(index_output),length(A)),eye(length(index_output))]; % "C" in paper?

                    Da  = zeros(size(Ca,1),size(Ba,2));

                    SimVFA.checkNegative(tzero(Aa,Ba,Ca,Da)); % check whether sys is min phase

                    Ba3  = Ba;
                    Ba2  = Aa*Ba3*SO.a22 + Ba3*SO.a21;

                    SimVFA.checkContObs(Aa,Ba,Ca);
                    SimVFA.checkRelDeg(Aa,Ba,Ca,Da,2); % make sure uniform relative degree three

                    % Add fictitious inputs (squaring up)
                    Ba_add_pool = null([Ca; (Ca*Aa)]);
                    Ba_aug = [Ba, 0.1*(6*Ba_add_pool(:,5)+6*Ba_add_pool(:,2)+0.4*Ba_add_pool(:,6))];
                    Da_aug = [Da, [0,0,0]']; % good design

                    [nrel_aug, vrel_aug, ~] = SimVFA.checkRelDeg(Aa,Ba_aug,Ca,Da_aug,2);

                    % this shouldn't change anything for uniform relative degree
                    [vrel_aug, Perm_aug] = SimVFA.sortPerm(vrel_aug);
                    Ba_aug = (Perm_aug*Ba_aug')';

                    index_m1 = []; 
                    index_m2 = 1:3; 

                    Ba3_aug  = Ba_aug;
                    Ba32_aug = Aa*Ba3_aug*SO.a22 + Ba3_aug*SO.a21;

                    % Find input normal form
                    [A_temp, B_temp, C_temp, U_temp, ~, ~]...
                            = SimVFA.findNormalForm(Aa',Ca',Ba_aug',vrel_aug,nrel_aug,1);
                        
                    % make a new system from dual
                    Uinv  = U_temp';
                    Atilt = A_temp'; % Atilt=U*A*Uinv
                    Btilt = C_temp'; % Btilt=U*B;
                    Ctilt = B_temp'; % Ctilt=C*Uinv;
                    Dtilt = zeros(size(Ctilt,1),size(Btilt,2));

                    % Cut some inputs out of above system
                    Ai    = Atilt;
                    Bi    = Btilt(:,index_m2);
                    Bi3   = Bi(:,1:num_input);
                    Bis1  = Btilt(:,index_m1);

                    Bi31  = SO.a22*Ai^2*Bi + SO.a21*Ai*Bi + SO.a20*Bi;

                    Ci    = Ctilt;
                    Bia1  = [Bi31,Bis1];
                    Dia1  = Dtilt;

                    test_MP = SimVFA.checkNegative(tzero(Ai,Bia1,Ci,Dia1)); % minimum phase flag

                    num_output_i = size(Ci,1);

                    if test_MP==1
                        Bi_aug = Bia1;

                        Rs = eye(num_output_i);

                        [S,C_bar,~] = SimVFA.findPostComp(Bi_aug,Ci,Rs,SO.postcomp==1);


                        [S1, ~] = SimVFA.findSquareDown(Bi3,Ci,S);

                        [F,~] = SimVFA.pickFSPR(Ai,Bi_aug,C_bar,SO.q0,SO.epsilon,SO.s0);
                        Lm = Bi_aug*F;

                        Lis = Lm*S;
                        Las = Uinv*Lis; % used in sim

                    end

                    % LQR design for u_bl
                    Kr = lqr(Aa, Ba, SO.qk, SO.rk);

                    % Add to lookup tables
                    Aa_M(:,:,eta_i+1) = Aa;
                    Ba_M(:,:,eta_i+1) = Ba;
                    Ba3_aug_M(:,:,eta_i+1)  = Ba3_aug;
                    Ba32_aug_M(:,:,eta_i+1) = Ba32_aug;
                    Ba3_M(:,:,eta_i+1)  = Ba3;
                    Ba32_M(:,:,eta_i+1) = Ba2;
                    Kr_M(:,:,eta_i+1)   = Kr;
                    Las_M(:,:,eta_i+1)  = Las;
                    Ras_M(:,:,eta_i+1)  = F;
                    Sas_M(:,:,eta_i+1)  = S;
                    Sas1_M(:,:,eta_i+1) = S1;  % to do

                end

                tables.Aa_M       = Aa_M;
                tables.Ba_M       = Ba_M;
                tables.Ba3_aug_M  = Ba3_aug_M;
                tables.Ba32_aug_M = Ba32_aug_M;
                tables.Ba3_M      = Ba3_M;
                tables.Ba32_M     = Ba32_M;
                tables.Kr_M       = Kr_M;
                tables.Las_M      = Las_M;
                tables.Ras_M      = Ras_M;
                tables.Sas_M      = Sas_M;
                tables.Sas1_M     = Sas1_M;
                
                if saveTables
                    save(saveFile, '-struct', 'tables');        
                end
            end
            TP.tables = tables;
            vfa.trimPts = TP; % save updated struct
        end
        
        function genController(vfa)
            SO = vfa.simOpt; % to be saved at end of function
            TP = vfa.trimPts;
            
            SO.input_hold = TP.trim_inputs(SO.eta_nom+1, :);
            SO.state_hold = TP.trim_states(SO.eta_nom+1, :);
            Ap = TP.A_hold(:,:,SO.eta_nom+1);
            B_c_hold = TP.B_hold(:,1:5,:);
            Bp = B_c_hold(:,:,SO.eta_nom+1);
            SO.Bp = Bp;
            
            % increase dimension (w/zeros) of controls fed to VFA sim model
            SO.inputselect = SimVFA.selectionMatrix(5, length(SO.i_input_sel), SO.i_input_sel);
            Cp = [eye(6), zeros(6,1)]; % take first six of seven states
            Dp = zeros(size(Cp,1), size(Bp,2));

            % augment plant with (nominal) actuator dynamics
            A = [Ap(SO.i_state_sel,SO.i_state_sel), Bp(SO.i_state_sel,SO.i_input_sel), zeros(length(SO.i_state_sel),length(SO.i_input_sel));
                 zeros(length(SO.i_input_sel),length(SO.i_state_sel)+length(SO.i_input_sel)), eye(length(SO.i_input_sel));
                 zeros(length(SO.i_input_sel),length(SO.i_state_sel)), -SO.w_act^2*eye(length(SO.i_input_sel)), -2*SO.zeta_act*SO.w_act*eye(length(SO.i_input_sel))];
            B = [0*Bp(SO.i_state_sel,SO.i_input_sel);
                 zeros(length(SO.i_input_sel));
                 SO.w_act^2*eye(length(SO.i_input_sel))];

            C  = [Cp(SO.i_output,SO.i_state_sel), zeros(length(SO.i_output), 2*length(SO.i_input_sel))];
            Cz = [0,0,0,0,1,0,0,0,0,0;
                  0,TP.Vinitial*Ap(2,2),0,0,0,0,TP.Vinitial*Bp(2,SO.i_input_sel),0,0];
            SO.Cz = Cz;
            
            D  = Dp(SO.i_output, SO.i_input_sel);
            SO.Dz = zeros(2, length(SO.i_input_sel));

            num_state  = length(A);
            num_input  = size(B,2);

            % Augment plant with integral error 
            % integrate the tracking output to make the system strictly proper
            index_output = [1,2]; % what's this one for?
            index_cmd    = [2,3]; % the output w

            % this sys used in RM: 
            % Am (not in paper) = Aa - Ba*Kr,   Bm = Baz,   Cm (not in paper) = Ca
            Aa = [A,zeros(length(A),length(index_output));
                  Cz,zeros(length(index_output))];           % "A" in paper
            SO.Aa = Aa;
            Ba = [B; zeros(length(index_output),num_input)]; % "B_3" in paper
            SO.Ba = Ba;
            Baz = [0*B; -eye(num_input)];                    % "B_z" in paper

            Ca  = [C, zeros(size(C,1),length(index_output));
                   zeros(length(index_output),length(A)),eye(length(index_output))]; % "C" in paper?
            SO.Ca = Ca;
            SO.Caz = [Cz, zeros(length(index_cmd))];

            Da  = zeros(size(Ca,1),size(Ba,2));

            SimVFA.checkNegative(tzero(Aa,Ba,Ca,Da)); % check whether sys is min phase

            B1   = [Bp(SO.i_state_sel,SO.i_input_sel);
                    zeros(2*length(SO.i_input_sel),length(SO.i_input_sel))];
            Daz1 = [D; TP.Vinitial*Bp(2,SO.i_input_sel)];

            Ba3  = Ba;
            Ba2  = Aa*Ba3*SO.a22 + Ba3*SO.a21;
            Ba1  = [B1; Daz1];

            SimVFA.checkContObs(Aa,Ba,Ca);
            SimVFA.checkRelDeg(Aa,Ba,Ca,Da,2); % make sure uniform relative degree three

            % Add fictitious inputs (squaring up)
            Ba_add_pool = null([Ca; (Ca*Aa)]);
            Ba_aug = [Ba, 0.1*(6*Ba_add_pool(:,5)+6*Ba_add_pool(:,2)+0.4*Ba_add_pool(:,6))];
            Da_aug = [Da, [0,0,0]']; % good design

            [nrel_aug, vrel_aug, ~] = SimVFA.checkRelDeg(Aa,Ba_aug,Ca,Da_aug,2);

            % this shouldn't change anything for uniform relative degree
            [vrel_aug, Perm_aug] = SimVFA.sortPerm(vrel_aug);
            Ba_aug = (Perm_aug*Ba_aug')';

            index_m1 = []; 
            index_m2 = 1:3; 

            % Find input normal form
            [A_temp, B_temp, C_temp, U_temp, ~, ~]...
                    = SimVFA.findNormalForm(Aa',Ca',Ba_aug',vrel_aug,nrel_aug,1);
            % make a new system from dual
            Uinv  = U_temp';
            Atilt = A_temp'; % Atilt=U*A*Uinv
            Btilt = C_temp'; % Btilt=U*B;
            Ctilt = B_temp'; % Ctilt=C*Uinv;
            Dtilt = zeros(size(Ctilt,1),size(Btilt,2));

            % Cut some inputs out of above system
            Ai    = Atilt;
            Bi    = Btilt(:,index_m2);
            Bi3   = Bi(:,1:num_input);
            Bis1  = Btilt(:,index_m1);

            Bi31  = SO.a22*Ai^2*Bi + SO.a21*Ai*Bi + SO.a20*Bi;

            Ci    = Ctilt;
            Bia1  = [Bi31,Bis1];
            Dia1  = Dtilt;

            test_MP = SimVFA.checkNegative(tzero(Ai,Bia1,Ci,Dia1)); % minimum phase flag
            % Defined here and also above

            num_state_i  = length(Ai);
            num_input_i  = size(Bi3,2);
            num_output_i = size(Ci,1);
            num_cmd = length(index_cmd);

            if test_MP==1
                Bi_aug = Bia1;

                Rs = eye(num_output_i);

                [S,C_bar,~] = SimVFA.findPostComp(Bi_aug,Ci,Rs,SO.postcomp);

                [F,~] = SimVFA.pickFSPR(Ai,Bi_aug,C_bar,SO.q0,SO.epsilon,SO.s0);
                Lm = Bi_aug*F;

                Lis = Lm*S;
                SO.Las = Uinv*Lis; % used in sim

            end

            % Reference Model Setup
            SO.Si1 = [eye(num_state-2*num_input),zeros(num_state-2*num_input,num_state_i-num_state+2*num_input)];
            SO.Si3 = [eye(num_state),zeros(num_state,num_state_i-num_state)];

            if SO.uncertFlag
                SO.Psi3_s(:,num_state+1:num_state_i) = zeros(num_input_i,num_cmd);
                SO.Psi2_s(:,num_state+1:num_state_i) = zeros(num_input_i,num_cmd);
                SO.Psi1_s(:,num_state+1:num_state_i) = zeros(num_input_i,num_cmd);
                SO.Lambda_s = SO.lambda_s*eye(num_input); % actuator effectiveness matrix
            else
                SO.Psi3_s = 0*SO.Psi3_s;
                SO.Psi2_s = 0*SO.Psi2_s;
                SO.Psi1_s = 0*SO.Psi1_s;
                SO.Lambda_s = eye(num_input);
            end
            
            Asim = Aa + Ba3*SO.Psi3_s + Ba1*SO.Psi1_s + Ba2*SO.Psi2_s; % do not introduce the uncertainty in Cp.
            Asim(7:10,7:10) = [zeros(2),eye(2); -SO.w_act_a^2*eye(2),-2*SO.zeta_act_a*SO.w_act_a*eye(2)];
            SO.Apsim = Ap;
            SO.Apsim(SO.i_state_sel,SO.i_state_sel) = Asim(1:(num_state-2*num_input),1:(num_state-2*num_input));
           
            SO.Bm = Baz; % reference model input matrix for r
            SO.Cm  = Ca;

            % Simulation parameters
            % high-order tuner gains
            SO.Gamma.l = 1000*eye(num_input_i);
            SO.Gamma.vl = 10000*eye(num_input_i);
            SO.Gamma.p1 = 10*eye(num_state-2*num_input);
            SO.Gamma.p2 = 10*eye(num_state-2*num_input);
            SO.Gamma.p3 = 10*eye(num_state);
            SO.Gamma.p31 = 1000*eye(num_output_i);
            SO.Gamma.p31xm = SO.Gamma.p31(1:num_input_i,1:num_input_i);
            SO.Gamma.p32 = 1000*eye(num_output_i);

            % intial conditions
            SO.lambda_0 = eye(num_input_i);
            SO.vlambda_0 = eye(num_input_i);
            SO.psi1_0 = zeros(num_state-2*num_input,num_input_i);
            SO.psi2_0 = zeros(num_state-2*num_input,num_input_i);
            SO.psi3_0 = zeros(num_state,num_input_i);
            SO.psi31_0 = zeros(num_output_i,num_output_i);
            SO.psi31xm_0 = zeros(num_input_i,num_input_i);
            SO.psi32_0 = zeros(num_output_i,num_output_i);

            % more high-order tuner gains
            SO.mu.lambda = 1; SO.mu.vlambda = 1; SO.mu.psi1 = 1; 
            SO.mu.psi2 = 1; SO.mu.psi3 = 1; SO.mu.psi31  = 1; 
            SO.mu.psi32 = 1;

            SO.x_red_est_0 = zeros(num_state_i,1);

            % command filter
            SO.Acmd = [zeros(num_cmd),eye(num_cmd);
                    -SO.w_cmd^2*eye(num_cmd),-2*SO.w_cmd*SO.zeta_cmd*eye(num_cmd)];
            SO.Bcmd = [zeros(num_cmd);SO.w_cmd^2*eye(num_cmd)];
            SO.Ccmd = [eye(num_cmd),zeros(num_cmd)];
            SO.Ccmd_dot = [zeros(num_cmd),eye(num_cmd)];

            % actuator dynamics
            SO.Aact = [zeros(num_input), eye(num_input);
                    -SO.w_act_a^2*eye(num_input), -2*SO.w_act_a*SO.zeta_act_a*eye(num_input)];
            SO.Bact = [zeros(num_input);SO.w_act_a^2*eye(num_input)];
            SO.Bact_x = Asim(num_state-2*num_input+1:num_state,1:num_state-2*num_input);
            SO.Cact = [eye(num_input),zeros(num_input)];
            SO.Cact_dot = [zeros(num_input),eye(num_input)];
            
            % process noise
            SO.d_seed = (1:num_input_i)';
            SO.d_sam  = 1;
            SO.d_mean = 0;

            % tracking feedthrough noise (Z)
            SO.n_seed = (1:num_input_i)';
            SO.n_sam  = 1;
            SO.n_mean = 0;    

            % gust model properties
            SO.samplet = 0.5;
            SO.tpower = 0;
            
            vfa.simOpt = SO; % save updated struct
        end
        
        function runSim(vfa)
            disp('Simulation will start');
            start = tic;
            vfa.simOutObj = sim(vfa.simInObj);
            run_time = toc(start);
            disp(['Simulation finished in ', num2str(run_time), ' seconds']);
        end
        
        function plotTrim(vfa)
            % plot characteristics of the trimmed aircraft

            TP = vfa.trimPts;
            
            % Figure 1: Zoomed-out imaginary plane
            figure; 
            hold on
            plot(TP.trim_eig_real,TP.trim_eig_imag,'.k','markersize',12);
            h2=plot(TP.trim_eig_real(1,:),TP.trim_eig_imag(1,:),'^m','markersize',8,'LineWidth',2);
            h3=plot(TP.trim_eig_real(12,:),TP.trim_eig_imag(12,:),'sr','markersize',8,'LineWidth',2);
            h4=plot(TP.trim_eig_real(46,:),TP.trim_eig_imag(46,:),'vb','markersize',8,'LineWidth',2);
            h=legend([h2 h3 h4],'$\eta=0$ deg','$\eta=11$ deg','$\eta=45$ deg');
            set(h,'fontsize',vfa.pltOpt.legfontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname,'Interpreter','Latex','Location','SouthWest')
            grid on;
            xlabel('Real','fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)
            ylabel('Imaginary','fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)
            box on;
            set(gca,'fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)

            % Figure 2: Zoomed-in imaginary plane
            figure;
            hold on;
            plot(TP.trim_eig_real,TP.trim_eig_imag,'.k','markersize',12);
            h2=plot(TP.trim_eig_real(1,:),TP.trim_eig_imag(1,:),'^m','markersize',8,'LineWidth',2);
            h3=plot(TP.trim_eig_real(12,:),TP.trim_eig_imag(16,:),'sr','markersize',8,'LineWidth',2);
            h4=plot(TP.trim_eig_real(46,:),TP.trim_eig_imag(46,:),'vb','markersize',8,'LineWidth',2);
            h=legend([h2 h3 h4],'$\eta=0$ deg','$\eta=11$ deg','$\eta=45$ deg');
            set(h,'fontsize',vfa.pltOpt.legfontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname,'Interpreter','Latex','Location','SouthWest')
            grid on;
            xlabel('Real','fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)
            ylabel('Imaginary','fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)
            box on;
            set(gca,'fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)
            axis([-0.2 0.5 -2 2])

            % Figure 3: Input trims vs. dihedral angle
            figure;
            hold on;
            box on;
            plot(0:45,180/pi*TP.trim_inputs(:,2),'-k','markersize',12)
            plot(0:45,180/pi*TP.trim_inputs(:,3),'--k','markersize',12)
            plot(0:45,180/pi*TP.trim_inputs(:,4),'.k','markersize',12)
            plot(0:45,180/pi*TP.trim_inputs(:,5),':k','markersize',9,'LineWidth',2)
            plot(0:45,TP.trim_inputs(:,1)/5,'ok','markersize',6)
            axis([0 45 -30 40])
            ylabel('Trim (Control Surface Inputs [deg], Thrust [5 lbf])','fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)
            xlabel('Dihedral angle [deg]','fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)
            h=legend('$\delta_{a_c}$','$\delta_{a_o}$','$\delta_{e_c}$','$\delta_{e_o}$','Thrust');
            set(h,'fontsize',vfa.pltOpt.legfontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname,'Interpreter','Latex','Location','SouthEast')
            set(gca,'fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)


            % Figure 4: Angle of attack trim
            figure;
            hold on;
            box on;
            plot(0:45,TP.trim_states(:,2)*180/pi,'k')
            ylabel('Trim angle of attack [deg])','fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)
            xlabel('Dihedral angle [deg]','fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)
            axis([0 45 7 12])
            set(gca,'fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)

            % Figure 5: S_min [-]
            figure;
            hold on; box on;
            plot(TP.trim_states(:,6)*180/pi,TP.sv_hold_1,'k-o','markersize',5)
            plot(TP.trim_states(:,6)*180/pi,TP.sv_hold_2,'k-^','markersize',5)
            plot(TP.trim_states(:,6)*180/pi,TP.sv_hold_3,'k.-','markersize',5)
            plot(TP.trim_states(:,6)*180/pi,TP.sv_hold_5,'k-','markersize',5)
            plot(TP.trim_states(:,6)*180/pi,TP.sv_hold_4,'r')
            plot(TP.trim_states(:,6)*180/pi,TP.sv_hold_6,'b')
            ylabel('$S_{\min}$ [-]','fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname,  'Interpreter','Latex')
            xlabel('Dihedral angle [deg]','fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)
            h=legend('$\bar B_1$','$\bar B_2$','$\bar B_3$','$\bar B_4$');
            set(h,'fontsize',vfa.pltOpt.legfontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname,'Interpreter','Latex','Location','NorthEast')
            %axis([0 45 0 0.08])
            set(gca,'fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)
        end
        
        function plotSim(vfa)

            SOO = vfa.simOutObj;
            
            tsim = vfa.simOpt.tsim;
            
            figure;
            subplot(2,2,1)
            plot(SOO.t_sim, SOO.r_cmd(1,:))
            hold on; plot(SOO.t_sim, SOO.z_red(1,:))
            xlim([0 tsim])
            title('Dihedral (rad rel. to nominal)')

            subplot(2,2,2)
            plot(SOO.t_sim, SOO.r_cmd(2,:))
            hold on; plot(SOO.t_sim, SOO.z_red(2,:))
            xlim([0 tsim])
            title('Vertical Accel (ft/s^2)')

            subplot(2,2,3)
            plot(SOO.t_sim, SOO.u_red(1,:))
            xlim([0 tsim])
            title('Aileron (rad)')

            subplot(2,2,4)
            plot(SOO.t_sim, SOO.u_red(2,:))
            xlim([0 tsim])
            title('Elevator (rad)')
            
        end
    end
    
    methods (Static)
        function [ctrbM, obsM] = checkContObs(A,B,C,mode)
        % Check controllability and observability of {A, B, C}
            if (nargin < 4)
                mode = 3;
            end
            num_state = size(A,1);
            ctrbM     = zeros(num_state,num_state);
            obsM      = zeros(num_state,num_state);

            name = [inputname(1),',',inputname(2),',',inputname(3)];

            if (mode >= 1)
                ctrbM=ctrb(A,B);
                if (rank(ctrbM) == num_state)
                    disp(strcat('The system {',name,'} is completely controllable'));
                else
                    disp(strcat('The system {',name,'} is NOT completely controllable'));
                end
            end

            if (mode >= 2)
                obsM=ctrb(A',C');
                if (rank(obsM) == num_state)
                    disp(strcat('The system {',name,'} is completely observable'));
                else
                    disp(strcat('The system {',name,'} is NOT completely observable'));
                end
            end
        end
        
        function test = checkNegative(A)
        % Check whether input is completely negative
            if ~isempty(inputname(1))
                name = inputname(1);
            else
                name = 'Tested system';
            end
            Pos_A  = A(real(A)>0);
            Zero_A = A(real(A)==0);

            test = 0;
            if ~isempty(Pos_A)
            %     disp(['the unstable pole of',' ',name,' ','is',' ',(num2str(uns_pole_A))']);
                 disp(strcat(['There are positive entries in ',name],' they are: ',num2str(Pos_A)));
            end



            if ~isempty(Zero_A)
            %     disp(['the imaginary pole of',' ',name,' ','is',' ',(num2str(int_pole_A))']);
                 disp(strcat(['There are imaginary entries in ',name],' they are: ',num2str(Zero_A)));
            end

            if ( (length(Zero_A)+length(Pos_A)) == 0 )
                disp(strcat(name,' is all negative'));
                test = 1;
            end
        end
        
        function test = checkPD(A)
        % Check whether sqaure matrix A is positive definite
            if ~isempty(inputname(1))
                name = inputname(1);
            else
                name = 'Tested system';
            end

            [~,pd_check] = chol((A+A')/2);
            if pd_check~=0
                test = 0;
                disp(strcat(name,' is not positive definite'));
            else
                test = 1;
                disp(strcat(name,' is positive definite'));
            end
        end
        
        function [nrel,vrel,CAB_stack_k1,CAB_stack_k2,k1_stack,k2_stack] = checkRelDeg(A,B,C,D,mode)
        % Check relative degree for D=0 square system
            if nargin<4
                D = zeros(size(C,1),size(B,2));
                mode = 1;
                varname = 'D';
            else
                varname = inputname(4);
            end

            if nargin<5
                mode = 1;
            end

            if mode==1
                modename = 'output';
            elseif mode==2
                modename = 'input';
                A = A'; Ctemp = C; C = B'; B = Ctemp'; D = D';
            end

            nrel = []; vrel = []; CAB_stack_k1 = []; CAB_stack_k2 = []; k1_stack = []; k2_stack = [];
            name = ['{',inputname(1),',',inputname(2),',',inputname(3),',',varname,'}'];

            % if size(B,2)==size(C,1)
            if size(C,1) <= size(B,2)
            %     CAB_stack_k2=[];
            %     vrel=[];
                for i=1:1:size(C,1)    
                    k1 = -1;
                    if norm(D(i,:))<1e-8
                        k1 = 0;
                        while norm(C(i,:)*(A^k1)*B)<1e-8
                            k1 = k1+1;
                        end
                        stack_piece = C(i,:)*(A^k1)*B;
                    end
                    k1_stack = [k1_stack; k1];
                    CAB_stack_k1 = [CAB_stack_k1; stack_piece];
                    k2 = k1;
                    if rank([CAB_stack_k2; D(i,:)])<i        
                        while rank([CAB_stack_k2; C(i,:)*(A^k2)*B])<i
                            k2 = k2+1;
                        end
                        stack_piece = C(i,:)*(A^k2)*B;
                    else
                        stack_piece=D(i,:);
                    end
                    k2_stack = [k2_stack; k2];
                    vrel = [vrel; k2+1]; % k=nrel-1
                    CAB_stack_k2 = [CAB_stack_k2; stack_piece];    
                end

                if mode==2
                    CAB_stack_k2 = CAB_stack_k2';
                    CAB_stack_k1 = CAB_stack_k1';
                end
                nrel = sum(vrel);

                if sum(k1_stack==k2_stack)==size(C,1)
                    if ~isempty(vrel)
                        disp(['The system ',name,' has a ',modename,' relative degree vector ', num2str(vrel')]);    
                    end
                    disp(['The system ',name,' has a total ',modename,' relative degree of ',num2str(nrel)]);
                else
                    disp(['The system ',name,' does not have ',modename,' relative degree']);
                end
            elseif (mode==1 && size(C,1)>size(B,2))
                disp(['The system ',name,' is MOTI, cannot have output relative degree']);
            elseif (mode==2 && size(C,1)>size(B,2))
                disp(['The system ',name,' is MITO, cannot have input relative degree']);
            end
        end
        
        function test = checkSym(A)
        % Check whether matrix is symmetric
            row = size(A,1); col = size(A,2);
            test = 0;
            name = inputname(1);
            At = A';
            TestM = 0*A;
            if row==col
                for i=1:1:row
                    for j=1:1:col
                        if abs(A(i,j)-At(i,j))<1e-6
                            TestM(i,j) = 1;
                        end
                    end
                end
                TestN = sum(sum(TestM));

                if TestN==row*col
                    disp(strcat(name,' is symmetric'));
                    test=1;
                else
                    disp(strcat(name,' is NOT symmetric'));
                end
            else    
                disp(strcat(name,' is NOT square'));
            end
        end
        
        function [tzeros,N,M] = checkSysZero(A,B,C,D)
        % Finds the invariant zeros for both square and non-square system.
        % 
        % This function does not provide any multiplicity information
        % of the zeros. For multiplicity information, using smform.

            if (nargin == 3)
                D=zeros(size(C,1),size(B,2));
                name=['{',inputname(1),',',inputname(2),',',inputname(3),'}'];
            else
                name=['{',inputname(1),',',inputname(2),',',inputname(3),',',inputname(4),'}'];    
            end


            %find the system zeros
            % num_state_cmd=num_state+num_cmd;
            % num_input_cmd=num_input+num_cmd;
            % num_output_cmd=num_output+num_cmd;
            num_state  = length(A);
            num_input  = size(B,2);
            num_output = size(C,1);

            condCB = cond(C*B); % 2-norm condition number
            if (condCB < 1e8)
                N_prime = SimVFA.findLeftAnni(B);
                M = SimVFA.findRightAnni(C);
                N = pinv(N_prime*M) * N_prime;
                % condNM=cond(N*M);
            end
            %%%%%%%%%%%%%%%%%%
            if ((num_input==num_output)&&(num_input<num_state)&&(condCB<1e8)&&rank(D)==0)

                if (num_input < num_state)

                    NAM=N*A*M;
                    % tzeros=Check_UnstaPole_v1(NAM);
                    tzeros=eig(NAM);
                    tzeros=sort(tzeros);
                    if ~isempty(tzeros)
                         disp(strcat(['There are tzeros in',' ',name],[' ','they are',': '],num2str(tzeros)));
                    else
                        disp(strcat('The system {',name,'} does not have tzeros'));
                    end

                else
                    if rank(B*C)==num_state
                        disp(strcat('The system {',name,'} does not have tzeros'));
                    end
                end
            else
                if (num_input~=num_output)
                    disp(strcat('The system_',name,' is not square'));
                end
                if (num_input==num_state)
                    disp(strcat('The system_',name,' is first-order type'));
                end
                if (condCB>1e6)
                    disp('Warning: CB is poor conditioned');
                end
                tzeros=tzero(A,B,C,D);
                tzeros=sort(tzeros);
                if ~isempty(tzeros)
                    disp(strcat(['There are invariant zeros in',' ',name],[' ','they are',': '],num2str(tzeros)));
                else
                    disp(strcat('The system_',name,' does not have tzeros'));
                end
                N = SimVFA.findLeftAnni(B);
                M = SimVFA.findRightAnni(C);
            end            
        end
        
        function N = findLeftAnni(B)
        % Find left annihilator
            name = inputname(1);
            Nt   = null(B');
            N    = Nt';
            NB   = N*B;

            if norm(norm(NB))>1e-8
                disp(['Warning, the left annihilator of ',name,' might not be accurate']);
            end
        end
        
        function M = findRightAnni(C)
        % Find right annihilator
            name = inputname(1);
            M    = null(C);
            CM   = C*M;

            if norm(norm(CM))>1e-8
                disp(['Warning, the right annihilator of ',name,' might not be accurate']);
            end
        end
        
        function [Atilt,Btilt,Ctilt,U,Uinv,Tau,Cscr,Bscr,Nscr,Mscr] = findNormalForm(A,B,C,vrel,nrel,sisolike)
        % Find normal form of system
        % 
        % only works for square system with ordered vector relative degree
        % v2 separate the case of SISO and MIMO, but ask for uniform relative degree
        % for siso-like
        % v3 improves the siso-like representation, 
        % and be applicable to non-uniform relative degree system, but ask for the same relative
        % degree to be grouped together

            if nargin<6
                sisolike=0;
            end

            m    = size(B,2);
            n    = length(A);
            Tau  = [];
            Cscr = [];
            Bscr = [];

            if size(B,2)==size(C,1)

                if (size(B,2)==1 || (sisolike==1 && (all(diff(vrel)>=0) || all(diff(vrel)<=0))==1) )
                    %has to be monotonic

                    disp('SISO or square + uniform relative degree system');
                    for i=1:1:length(vrel)
                        Cseg = [];
                        Bseg = [];
                        for j=1:1:vrel(i)
                            Cseg = [Cseg; C(i,:)*(A^(j-1))];
                            Bseg = [Bseg, A^(j-1)*B(:,i)];
                        end
                        Cscr = [Cscr; Cseg];
                        Bscr = [Bscr, Bseg];
                        Tau  = [Tau; C(i,:)*A^(vrel(i)-1)*B];
                    end

                    Mscr = SimVFA.findRightAnni(Cscr);
                    Nscr = inv(Mscr'*Mscr)*Mscr'*(eye(n)-Bscr*inv(Cscr*Bscr)*Cscr);
                    U = [Cscr; Nscr];
                else
                    disp('MIMO system');

                    for i=1:1:m
                       Tau(i,:) = C(i,:)*(A^(vrel(i)-1))*B;
                       Cscr_p   = zeros(vrel(i),n);

                       for j=1:1:vrel(i)
                           Cscr_p(j,:) = C(i,:)*(A^(j-1));
                       end
                       Cscr = [Cscr;Cscr_p];

                    end

                    Bscr = [];
                    % for i=1:1:vrel(1) %1 to r1
                    for i=1:1:max(vrel)
                       mi   = length(find(vrel>=i));
                       em   = [eye(mi); zeros(m-mi,mi)];
                       Bscr = [Bscr, A^(i-1)*B*inv(Tau)*em];
                    end

                    Mscr = SimVFA.findRightAnni(Cscr);
                    Nscr = inv(Mscr'*Mscr)*Mscr'*(eye(n)-Bscr*inv(Cscr*Bscr)*Cscr);
                    Uhat = [Cscr; Nscr];

                    U = Uhat;
                    for i=1:1:m
                       Ti = [zeros(vrel(i)+n-nrel, sum(vrel(1:i-1))), ...
                                 [eye(vrel(i)); zeros(n-nrel,vrel(i))], ...
                                 zeros(vrel(i)+n-nrel,sum(vrel((i+1):m))), ...
                                 [zeros(vrel(i),n-nrel);eye(n-nrel)]];
                       Cihat = [eye(vrel(i)), zeros(vrel(i),n-nrel)];
                       eri   = zeros(vrel(i)+n-nrel,1);
                       eri(vrel(i)) = 1;
                       Bihat = [];
                       for j=1:1:vrel(i)
                           Bihat = [Bihat, (Ti*Uhat*A*inv(Uhat)*Ti')^(j-1)*eri];
                       end
                       Nihat = [zeros(n-nrel,vrel(i)),eye(n-nrel)]*...
                                (eye(vrel(i)+n-nrel)-Bihat*inv(Cihat*Bihat)*Cihat);
                       Uihat = [eye(nrel),zeros(nrel,n-nrel);...
                                zeros(n-nrel,sum(vrel(1:(i-1)))), Nihat*[eye(vrel(i)); zeros(n-nrel,vrel(i))], ...
                                zeros(n-nrel,sum(vrel((i+1):m))),eye(n-nrel)];
                       U = Uihat * U; 
                    end
                end

            Uinv  = inv(U);
            Btilt = U*B;
            Ctilt = C*Uinv;
            Atilt = U*A*Uinv;

            end            
        end
        
        function [S,C_bar,CB_bar] = findPostComp(B,C_aug,R0,ver)
        % Post-compensator
            if ver == 1
                R0_invsqrt = inv(sqrtm(R0));
                [U,~,V] = svd(B'*C_aug'*R0_invsqrt);
                W = (U*V')';
                S = W'*R0_invsqrt;

                C_bar  = S*C_aug;
                CB_bar = C_bar*B;

                SimVFA.checkPD(CB_bar);
                SimVFA.checkSym(CB_bar);
            elseif ver == 2
                S = R0*inv(C_aug*B);

                C_bar  = S*C_aug;
                CB_bar = C_bar*B;

                SimVFA.checkPD(CB_bar);
                SimVFA.checkSym(CB_bar);
            else
                S = R0*(C_aug*B)';

                C_bar  = S*C_aug;
                CB_bar = C_bar*B;

                SimVFA.checkPD(CB_bar);
                SimVFA.checkSym(CB_bar);
            end
        end
        
        function [S1,Cs] = findSquareDown(B,C,S)
        % Slice S and return new S and SC
            num_input=size(B,2);
            S1=S(1:num_input,:);
            Cs=S1*C;
        end
        
        function [F,P] = pickFSPR(A,B,C,q0,epsilon,s0)
        % Pick F which is strictly positive real

            if (nargin==5 || isempty(s0))
                s0 = 0.2;
            end

            name = [inputname(1),',',inputname(2),',',inputname(3)];

            Aname  = inputname(1);
            Bname  = inputname(2);
            Cname  = inputname(3);
            CLname = ['(',inputname(1),'-',inputname(2),'F',inputname(3),')',',',inputname(2),',',inputname(3)];

            A_eta = A+s0*eye(length(A));
            n = length(A);
            m = size(B,2);
            p = size(C,1);
            r = rank(B*C);

            F = zeros(n,m);
            P = zeros(n,n);


            CB = C*B;
            PosDef_test = SimVFA.checkPD(CB);
            Sym_test    = SimVFA.checkSym(CB);
            [tzeros,N,M]  = SimVFA.checkSysZero(A,B,C);
            MinPhase_test = SimVFA.checkNegative(tzeros);

            if (PosDef_test==1 && Sym_test==1 && MinPhase_test==1)
                n_zeros=length(tzeros);

                if n_zeros>0
                    NAM = N*A*M;
                    Q0  = q0*eye(length(NAM));
                    P0  = lyap(NAM',Q0);
                    P   = C'*inv(CB)*C+N'*P0*N;
                    Q   = -A'*P - P*A;
                    Q_test = SimVFA.checkPD(Q);
                    if Q_test==1
                        F = zeros(size(B,2),size(C,1));
                        disp(strcat('{',name,'} is already SPR'));
                        return

                    else

                        Rlinv = inv(CB)*(C*A_eta*B+(C*A_eta*B)')*inv(CB)+epsilon*eye(m);
                        SimVFA.checkPD(Rlinv);
                        Ql = -A'*P - P*A + P*B*Rlinv*B'*P;
                        Ql_test = SimVFA.checkPD(Ql);

                        F = Rlinv;
                        if Ql_test==1
                            disp(strcat('Found F s.t. {',CLname,'} is SPR'));
                        else
                            disp('Not able to find SPR F, try to increase q0 or epsilon');
                        end
                    end
                else
                    P = C'*inv(CB)*C;
                    Q = -A'*P - P*A;
                    Q_test = SimVFA.checkPD(Q);
                    if Q_test==1
                        disp(strcat('{',name,'} is already SPR'));
                        return

                    else
                        Rlinv = inv(CB)*(C*A_eta*B+(C*A_eta*B)')*inv(CB)+epsilon*eye(m);
                        SimVFA.checkPD(Rlinv);
                        Ql = -A'*P - P*A + C'*Rlinv*C;
                        Ql_test = SimVFA.checkPD(Ql);

                        F = Rlinv;
                        if Ql_test==1
                            disp(strcat('Found F s.t. {',CLname,'} is SPR'));
                        else
                            disp('Not able to find SPR F, try to increase epsilon');
                        end
                    end
                end
            else
                disp(strcat('(',Cname,'*',Bname,') is not SPD, or the system is not minimum phase'));
            end
        end
        
        function S_K = selectionMatrix(num_out, num_in, indices_in)
        % Takes a vector of length num_in and outputs a vector of length num_out
        % passing indices indices_in with unity gain

            S_K = zeros(num_out,num_in);
            for i=1:1:length(indices_in)
                j = indices_in(i);
                S_K(j,i) = 1;
            end
        end
        
        function [vsort, Perm] = sortPerm(v)
        % Takes in a vector and sort it and produce a permutation matrix
            [vsort,index] = sort(v,'descend');
            n = length(v);
            Perm = zeros(n,n);
            for i=1:1:n
                Perm(i,index(i)) = 1;
            end
        end
        
    end
end