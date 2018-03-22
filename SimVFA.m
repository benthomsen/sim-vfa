% see RunVFASim.m for description
classdef SimVFA < handle
    properties
        simOpt    % struct holding simulation options
        simInObj  % object to pass to Simulink
        simOutObj % object returned from Simulink
        trimPts   % trim points and linearizations
        pltOpt    % struct for plotting options
    end
    methods
        function vfa = SimVFA(opt) % constructor
            
            vfa.setSimOpts(opt); % set basic options for simulation
            vfa.setTrimOpts();   % set params for trimming routine
            vfa.setPlotOpts();   % set basic plotting options
            
            if (vfa.simOpt.mActOrder == 2) % second-order actuator model for control design
                vfa.simInObj = Simulink.SimulationInput('VFA_ActModel2');
            else % first-order actuator model for control design
                vfa.simInObj = Simulink.SimulationInput('VFA_ActModel1');
            end
            
            % trim aircraft
            vfa.trimAC(~vfa.simOpt.reComp, vfa.simOpt.reComp, false);   
            % compute system and controller matrices at each linearization
            vfa.genLookupTables(~vfa.simOpt.reComp, vfa.simOpt.reComp); 
            % compute everything required for sim at initial dihedral
            vfa.genController();
            % put required variables in a Simulink input object
            vfa.populateSimIn();
            
        end
        
        function setSimOpts(vfa, opt)
        % set basic simulation options: uncertainty, commands, indices, etc.
        
            SO = vfa.simOpt; % shorthand
            
            % data save/load path (relative to working dir.)
            if (isfield(opt, 'dataPath') && ~isempty(opt.dataPath))
                SO.dataPath = opt.dataPath;
            else
                SO.dataPath = [];
            end
            
            % flag for adaptive control on/off
            if (isfield(opt, 'adaFlag') && ~isempty(opt.adaFlag))
                SO.adaFlag = logical(opt.adaFlag);
            else
                SO.adaFlag = true; % default to adaptive control on
            end
            
            % flag for uncertainty on/off
            if (isfield(opt, 'uncertFlag') && ~isempty(opt.uncertFlag))
                SO.uncertFlag = logical(opt.uncertFlag);
            else
                SO.uncertFlag = true; % default to uncertainty on
            end
            
            % flag for recomputing trim and linearization
            if (isfield(opt, 'reComp') && ~isempty(opt.adaFlag))
                SO.reComp = logical(opt.reComp);
            else
                SO.reComp = true; % default to trim/linearization on
            end
            
            % specification for order of modeled actuator dynamics (first
            % or second) in control design
            if (isfield(opt, 'mActOrder') && (opt.mActOrder == 1 || opt.mActOrder == 2))
                SO.mActOrder = opt.mActOrder;
            else
                SO.mActOrder = 1; % default to first-order actuator model in control design
            end
            
            % specification for order of actuator dynamics in plant (true order)
            if (isfield(opt, 'pActOrder') && (opt.pActOrder == 1 || opt.pActOrder == 2))
                % don't allow plant with lower order than model
                SO.pActOrder = max(opt.pActOrder, SO.mActOrder);
            else
                SO.pActOrder = max(1, SO.mActOrder); % default to first-order actuators
            end
            
            % uncertainty in plant
            Theta_p = [0.6, -4.52, 0, 0.05, 0.41, 1.48;
                       0.1, 1.83, 0, -0.02, -0.35, -0.59]'; % from example in paper
            
            % index selections
            SO.i_state_sel = [1:2,4:7]; % select all states except altitude
            SO.i_dihedral  = 6;         % dihedral angle
            SO.i_input_sel = [3,4];     % outer aileron, center elevator
            SO.i_output    = 5;         % pitch rate

            % actuator parameters and uncertainty 
            % (and some parameter defs which are specific to actuator model)
            if (SO.pActOrder == 2) % second-order actuator model for control
                % second order actuator dynamics (nominal)
                SO.w_act    = 2;   % natural frequency
                SO.zeta_act = 0.7; % damping
              
                % matrix form of nominal actuator params
                SO.D_1 = (SO.w_act)^2 * eye(2);
                SO.D_2 = (2*SO.zeta_act*SO.w_act) * eye(2);
                
                if SO.uncertFlag
                    % coefficients to scale uncertainty matrices by
                    SO.Psi1_scale = 1;
                    SO.Psi3_scale = 1;
                    SO.w_act_a    = 0.5; % actuator natural frequency
                    SO.zeta_act_a = 0.8; % actuator damping ratio
                    SO.lambda_s   = 0.2; % actuator effectiveness
                else
                    SO.w_act_a    = SO.w_act;
                    SO.zeta_act_a = SO.zeta_act;
                    SO.lambda_s   = 1;
                    SO.Psi1_scale = 0;
                    SO.Psi3_scale = 0;    
                end
                
                if (SO.mActOrder == 1)
                    SO.eig_act = -SO.w_act_a; % actuator cutoff frequency
                    SO.a11 = 0.5; SO.a10 = 1; % first-order filter coefficients
                end
                    
                SO.a22 = 1; SO.a21 = 2/sqrt(2); SO.a20 = 2; % second-order filter coefficients
                SO.d22 = 1; SO.d21 = 2; SO.d20 = 1; % derivative coefficieints
                SO.d11 = 1; SO.d10 = 1; % derivative coefficients
                SO.d00 = 1; % derivative coefficients
                                        
                % actuator uncertainty matrices
                Theta_1 = ((SO.w_act_a)^2 - (SO.w_act)^2) * eye(2);
                Theta_2 = 2*(SO.zeta_act_a*SO.w_act_a - SO.zeta_act*SO.w_act) * eye(2);
                
                % overall system uncertainty matrices
                if (SO.mActOrder == 2)
                    SO.Psi1 = SO.Psi1_scale * ([Theta_p; zeros(6, 2)]'); % [\Theta_{p}^{*}; 0; 0]^{T}
                    SO.Psi3 = SO.Psi3_scale * (-inv(SO.D_1) * [zeros(6,2); Theta_1; Theta_2; zeros(2,2)]');
                else
                    SO.Psi1_act = SO.Psi1_scale * ([Theta_p; zeros(6, 2)]'); % [\Theta_{p}^{*}; 0; 0]^{T}
                    SO.Psi3_act = SO.Psi3_scale * (-inv(SO.D_1) * [zeros(6,2); Theta_1; Theta_2; zeros(2,2)]');
                end
                
                % for baseline LQR design
                SO.rk = 50*eye(length(SO.i_input_sel));
                if (SO.mActOrder == 2)
                    SO.qk = diag([0.001,2,0.001,2,200,50,0.001,0.001,0.0001,0.0001,2,0.001]);
                else
                    SO.qk = diag([0.001,2,0.001,2,200,50,0.001,0.001,4,0.01]);
                end
            else % model order = true order of linearized dynamics
                % first order actuator dynamics (nominal)
                SO.w_act = 2; % cutoff frequency
                
                % matrix form of nominal actuator params
                SO.D_1 = (SO.w_act) * eye(2);

                if SO.uncertFlag
                    % coefficients to scale uncertainty matrices by
                    SO.Psi1_scale = 1;
                    SO.Psi2_scale = 1;
                    SO.eig_act  = -0.5; % actuator cutoff frequency
                    SO.lambda_s = 0.2;  % actuator effectiveness
                else
                    SO.eig_act    = -SO.w_act;
                    SO.lambda_s   = 1;
                    SO.Psi1_scale = 0;
                    SO.Psi2_scale = 0;
                end
                
                SO.a11 = 0.5; SO.a10 = 1; % first-order filter coefficients

                % actuator uncertainty matrix
                Theta_1 = (-SO.eig_act - SO.w_act) * eye(2);
                
                % overal system uncertainty matrices
                SO.Psi1 = SO.Psi1_scale * ([Theta_p; zeros(4, 2)]'); % [\Theta_{p}^{*}; 0; 0]^{T}
                SO.Psi2 = SO.Psi2_scale * (-inv(SO.D_1) * [zeros(6,2); Theta_1; zeros(2,2)]');

                % for baseline LQR design
                SO.rk = 50*eye(length(SO.i_input_sel));
                SO.qk = diag([0.001,2,0.001,2,200,50,0.001,0.001,4,0.01]);
            end
            
            SO.Lambda_s = SO.lambda_s*eye(2); % actuator effectiveness matrix

            % post-compensator version (1, 2, or 3) - calculation of output mixing matrix S
            SO.postcomp = 1; 
            SO.q0 = 10; SO.epsilon = 20;
            % q0 used in P0 = lyap(NAM', q0*eye[]) to find Rinv
            SO.s0 = 0.25; % s0 used in A_eta = A+s0*eye[] to find Rinv
            
            % command filter coefficients
            SO.zeta_cmd = 2; SO.w_cmd = 0.5;

            SO.data_deci = 1; % decimation for saved data from Simulink
            
            SO.eta_nom = 11; % select dihedral angle (deg) [== ind-1] from linearized tables

            % commands
            r_step_eta = -2 * pi/180 * [-0.5, -1, -0.5, -1, -0.5, -1, 0.5, -0.5, -1, -0.5, -1, -0.5, -1, 0.5, 0, 0];
            r_step_Az  = 2 * [-0.5, -1, -0.5, -1, -0.5, -1, 0.5, -0.5, -1, -0.5, -1, -1, 0.5, 0.5, 0, 0];
            r_step_scaled = [r_step_eta; r_step_Az];

            % simulation times for command steps
            t_step = 70;
            SO.tsim = t_step*(length(r_step_scaled)-1)+1;
            r_timeseries = timeseries(r_step_scaled, 1:t_step:SO.tsim);
            SO.r_timeseries = setinterpmethod(r_timeseries,'zoh');
            
            vfa.simOpt = SO;  % save updated struct
        end
        
        function setTrimOpts(vfa)
        % set options for trimming nonlinear VFA model
            % data: [w; x; y; z] (used for inertial properties of VFA)
            vfa.trimPts.data = [300; 30; 2; 18];
            vfa.trimPts.Vinitial = 68;     % airspeed (ft/s)
            vfa.trimPts.hinitial = 40000;  % altitude (ft)
            vfa.trimPts.alphainitial = 2.8*pi/180; % angle of attack (rad)
            vfa.trimPts.thetainitial = 2.8*pi/180; % pitch angle guess (rad)
            vfa.trimPts.etasweep = (0:20)'; % dihedral angles to linearize/trim (deg)
        end
        
        function setPlotOpts(vfa)
        % set plotting options
            vfa.pltOpt.legfontsize = 12;
            vfa.pltOpt.fontsize = 14;
            vfa.pltOpt.weight = 'n';
            vfa.pltOpt.fontname = 'Times New Roman';
        end
        
        function trimAC(vfa, loadTrim, saveTrim, pltTrim)
        % find trim with zero dihedral rate (eta_dot=0), or load from file
        % also linearize system at each specified dihedral angle

            % file to save or load trim data from
            saveFile = [vfa.simOpt.dataPath, 'VFA_trimData'];
            
            if loadTrim
            	vfa.trimPts = load(saveFile);                
            else
                % 7 states: V (ft/s), alpha (rad), h (ft), theta (rad), 
                %           q (rad/s), eta (rad), etadot (rad/s)
                % 9 inputs: thrust, center aileron, outer aileron, center 
                %           elevator, outer elevator, [4 wind inputs]
                
                TP = vfa.trimPts; % just for shorthand - will save struct at end of function
                
                assignin('base', 'stateinit', zeros(7,1));
                assignin('base', 'data', TP.data);
                oper_spec = operspec('VFA_lin');
                
                % iterate through desired dihedral angles to trim/linearize
                for i=min(TP.etasweep):max(TP.etasweep) 

                    etainitial = i*pi/180; % dihedral angle in radians

                    stateguess  = [TP.Vinitial TP.alphainitial TP.hinitial TP.thetainitial 0 etainitial 0]';
                    inputguess  = [100 0 (15-i/10)*pi/180 0 -0.1 0 0 0 0]';
                    statesknown = [1 1 1 1 1 1 1]';
                    inputknown  = [0 1 0 0 0 1 1 1 1]';

                    %%%%%%%%% setting parameters %%%%%%%%%%%%%
                    set(oper_spec.States,'known',statesknown,'x',stateguess);
                    set(oper_spec.States,'Min',[10 0 0 0 -2 0 -2]');
                    set(oper_spec.States,'Max',[80 10*pi/180 100000 30*pi/180 2 60*pi/180 2]');
                    set(oper_spec.Inputs,'known',inputknown,'u',inputguess,'Min',[0 -0.2 -0.2 -1 -1 0 0 0 0]','Max',[2000 0.2 0.2 1 1  0 0 0 0]');

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
                    TP.rank_ctrb(i+1) = rank(ctrb(linsys.A,linsys.B(1:7,[1 2 4])));

                    % update the saved struct
                    vfa.trimPts = TP;
                end

                if saveTrim
                    % save fields of TP struct in .mat file
                    save(saveFile,'-struct','TP');
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
            
            SI = SI.setVariable('Aact', SO.Aact);
            SI = SI.setVariable('Acmd', SO.Acmd);
            SI = SI.setVariable('Ada_Flag', int8(SO.adaFlag));
            SI = SI.setVariable('Bact', SO.Bact);
            SI = SI.setVariable('Bact_x', SO.Bact_x);
            SI = SI.setVariable('Bcmd', SO.Bcmd);
            SI = SI.setVariable('Baz', SO.Baz);
            SI = SI.setVariable('Bp', SO.Bp);
            SI = SI.setVariable('Ca', SO.Ca);
            SI = SI.setVariable('Cact', SO.Cact);
            SI = SI.setVariable('Ccmd', SO.Ccmd);
            SI = SI.setVariable('Cz', SO.Cz);
            SI = SI.setVariable('d_mean', SO.d_mean);
            SI = SI.setVariable('d_sam', SO.d_sam);
            SI = SI.setVariable('d_seed', SO.d_seed);
            SI = SI.setVariable('data_deci', SO.data_deci);
            SI = SI.setVariable('i_input_sel', SO.i_input_sel);
            SI = SI.setVariable('i_state_sel', SO.i_state_sel);
            SI = SI.setVariable('inputselect', SO.inputselect);
            SI = SI.setVariable('r_timeseries', SO.r_timeseries);
            SI = SI.setVariable('samplet', SO.samplet);
            SI = SI.setVariable('tpower', SO.tpower);
            SI = SI.setVariable('tsim', SO.tsim);
            SI = SI.setVariable('xm_0', SO.xm_0);
            SI = SI.setVariable('input_hold', SO.input_hold);
            SI = SI.setVariable('state_hold', SO.state_hold);
            SI = SI.setVariable('initstate', SO.state_hold);
            
            SI = SI.setVariable('Aa_M', TP.tables.Aa_M);
            SI = SI.setVariable('Ba_M', TP.tables.Ba_M);
            SI = SI.setVariable('Caz_M', TP.tables.Caz_M);
            SI = SI.setVariable('K_M', TP.tables.K_M);
            SI = SI.setVariable('L_M', TP.tables.L_M);
            SI = SI.setVariable('Rinv_M', TP.tables.Rinv_M);
            SI = SI.setVariable('S_M', TP.tables.S_M);
            SI = SI.setVariable('S1_M', TP.tables.S1_M);
            SI = SI.setVariable('data', TP.data);
            SI = SI.setVariable('etasweep', TP.etasweep);

            if (SO.mActOrder == 2) % second-order actuator model for control

                SI = SI.setVariable('Cact_dot', SO.Cact_dot);
                SI = SI.setVariable('Ccmd_dot', SO.Ccmd_dot);
                SI = SI.setVariable('Gamma_l', SO.Gamma.l);
                SI = SI.setVariable('Gamma_p1', SO.Gamma.p1);
                SI = SI.setVariable('Gamma_p2', SO.Gamma.p2);
                SI = SI.setVariable('Gamma_p3', SO.Gamma.p3);
                SI = SI.setVariable('Gamma_p31', SO.Gamma.p31);
                SI = SI.setVariable('Gamma_p31xm', SO.Gamma.p31xm);
                SI = SI.setVariable('Gamma_p32', SO.Gamma.p32);
                SI = SI.setVariable('Gamma_vl', SO.Gamma.vl);
                SI = SI.setVariable('Lambda_s', SO.Lambda_s);
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
                SI = SI.setVariable('vlambda_0', SO.vlambda_0);

                SI = SI.setVariable('Ba3_aug_M', TP.tables.Ba3_aug_M);
                SI = SI.setVariable('Ba32_aug_M', TP.tables.Ba32_aug_M);
                SI = SI.setVariable('Ba3_M', TP.tables.Ba3_M);
                SI = SI.setVariable('Ba32_M', TP.tables.Ba32_M);

            else % first-order actuator model

                SI = SI.setVariable('Gamma_l', SO.Gamma.l);
                SI = SI.setVariable('Gamma_p1', SO.Gamma.p1);
                SI = SI.setVariable('Gamma_p2', SO.Gamma.p2);
                SI = SI.setVariable('Gamma_p21', SO.Gamma.p21);
                SI = SI.setVariable('Lambda_s', SO.Lambda_s);
                SI = SI.setVariable('Si1', SO.Si1);
                SI = SI.setVariable('Si2', SO.Si2);

                SI = SI.setVariable('a10', SO.a10);
                SI = SI.setVariable('a11', SO.a11);
                SI = SI.setVariable('lambda_0', SO.lambda_0);
                SI = SI.setVariable('n_mean', SO.n_mean);
                SI = SI.setVariable('n_sam', SO.n_sam);
                SI = SI.setVariable('n_seed', SO.n_seed);
                SI = SI.setVariable('psi1_0', SO.psi1_0);
                SI = SI.setVariable('psi2_0', SO.psi2_0);
                SI = SI.setVariable('psi21_0', SO.psi21_0);

                SI = SI.setVariable('Ba_aug_M', TP.tables.Ba_aug_M);

            end
            
            vfa.simInObj = SI; % save updated object
        end
        
        function genLookupTables(vfa, loadTables, saveTables)
        % compute and save linearized matrices for VFA sim
        % including L, K, S and transformed coordinates
            TP = vfa.trimPts; % to be saved at end of function
            SO = vfa.simOpt;
            
            saveFile = [SO.dataPath, 'VFA_linSys_actModel', num2str(SO.mActOrder, '%d')];

            if loadTables
            	load(saveFile, 'tables', 'SO');
            else
                if (SO.mActOrder == 2) % second-order actuator model for control

                    % Lookup tables for matrices vs. dihedral angle 
                    Aa_M = zeros([12,12,length(TP.etasweep)]);
                    Ba_M = zeros([12,2,length(TP.etasweep)]);
                    Ba3_aug_M  = zeros([12,3,length(TP.etasweep)]);
                    Ba32_aug_M = zeros([12,3,length(TP.etasweep)]);
                    Ba3_M  = zeros([12,2,length(TP.etasweep)]);
                    Ba32_M = zeros([12,2,length(TP.etasweep)]);
                    Caz_M = zeros([2,12,length(TP.etasweep)]);
                    K_M    = zeros([2,12,length(TP.etasweep)]);
                    L_M    = zeros([12,3,length(TP.etasweep)]);
                    Rinv_M = zeros([3,3,length(TP.etasweep)]);
                    S_M    = zeros([3,3,length(TP.etasweep)]);
                    S1_M   = zeros([2,3,length(TP.etasweep)]);
                    
                    % iterate through systems linearized at fixed dihedrals
                    for eta_i = (min(TP.etasweep):max(TP.etasweep))
                        % Set plant parameters

                        Ap = TP.A_hold(:,:,eta_i+1);   % linearized state matrix (at selected dihedral)
                        Bp = TP.B_hold(:,1:5,eta_i+1); % linearized input matrix (at selected dihedral)
                        Cp = [eye(6), zeros(6,1)]; % all states measured in Cp except dihedral rate
                        Dp = zeros(size(Cp,1), size(Bp,2)); % no direct feedthrough in plant

                        % augment plant with (nominal) second-order actuator dynamics
                        % and remove altitude from states
                        A = [Ap(SO.i_state_sel,SO.i_state_sel), Bp(SO.i_state_sel,SO.i_input_sel), zeros(length(SO.i_state_sel),length(SO.i_input_sel));
                             zeros(length(SO.i_input_sel),length(SO.i_state_sel)+length(SO.i_input_sel)), eye(length(SO.i_input_sel));
                             zeros(length(SO.i_input_sel),length(SO.i_state_sel)), -SO.D_1, -SO.D_2];
                        B = [0*Bp(SO.i_state_sel,SO.i_input_sel);
                             zeros(length(SO.i_input_sel));
                             SO.D_1];
                        % C selects only pitch rate (q) as measurement
                        C  = [Cp(SO.i_output,SO.i_state_sel), zeros(length(SO.i_output), 2*length(SO.i_input_sel))];
                        % Cz selects dihedral angle and vertical acceleration as measurements
                        Cz = [0,0,0,0,1,0,0,0,0,0;
                              0,TP.Vinitial*Ap(2,2),0,0,0,0,TP.Vinitial*Bp(2,SO.i_input_sel),0,0];

                        num_input = size(B,2);
                        num_cmd = 2;

                        % Augment plant with integral error (for two tracked states)
                        index_output = [1,2]; % outputs to track (from Cz)

                        % augmented system is used in reference model and control in sim.
                        % the integral error states are appended after actuator dynamics.
                        % Am = Aa - Ba*K,   Bm = Baz,   Cm = Ca
                        Aa = [A,zeros(length(A),length(index_output));
                              Cz,zeros(length(index_output))];           % "A" in paper
                        Ba = [B; zeros(length(index_output),num_input)]; % "B_3" in paper

                        % Ca selects pitch rate and integral errors as measurements
                        Ca  = [C, zeros(size(C,1),length(index_output));
                               zeros(length(index_output),length(A)),eye(length(index_output))]; % "C" in paper
                        Caz = [Cz, zeros(num_cmd)]; % Caz selects dihedral angle and vertical acceleration as measurements
                        Da  = zeros(size(Ca,1),size(Ba,2)); % no direct feedthrough

                        num_output_i = size(Ca,1);

                        if (eta_i == SO.eta_nom)
                            D = Dp(SO.i_output, SO.i_input_sel);     % no direct feedthrough

                            SO.Cz = Cz;
                            SO.Baz = [0*B; -eye(num_input)]; % "B_z" in paper (reference model input for r)
                            SO.Ca = Ca;

                            SO.num_state = length(A);
                            SO.num_input = num_input;
                            SO.num_state_i  = length(Aa); % states in augmented system
                            SO.num_output_i = num_output_i;

                            SimVFA.checkNegative(tzero(Aa,Ba,Ca,Da)); % check whether sys is min phase
                            SimVFA.checkCtrbObsv(Aa,Ba,Ca);    % check that augmented system is ctrb and obsv
                            SimVFA.checkRelDeg(Aa,Ba,Ca,Da,2); % make sure uniform relative degree three

                            % B_1 in paper
                            B1 = [Bp(SO.i_state_sel,SO.i_input_sel);
                                  zeros(2*length(SO.i_input_sel),length(SO.i_input_sel));
                                  D; TP.Vinitial*Bp(2,SO.i_input_sel)];
                                 
                            % A^{*} in paper
                            SO.Asim = Aa + Ba*SO.Psi3 + B1*SO.Psi1;
                        end
                        
                        % coordinate change
                        Ba3  = SO.a22*Ba; % full relative-degree 3 input matrix
                        Ba2  = Aa*Ba*SO.a22 + Ba*SO.a21; % RD2 input path

%                         % Add fictitious inputs (squaring up): 
                        [Ba_aug, ~] = SimVFA.squareUpB(Aa, Ba, Ca, 1, 500);
                        Da_aug = [Da, [0,0,0]']; % no direct feedthrough

                        [nrel_aug, vrel_aug, ~] = SimVFA.checkRelDeg(Aa,Ba_aug,Ca,Da_aug,2);

                        % this shouldn't change anything for uniform relative degree
                        [vrel_aug, Perm_aug] = SimVFA.sortPerm(vrel_aug);
                        Ba_aug = (Perm_aug*Ba_aug')';

                        Ba3_aug  = SO.a22*Ba_aug;
                        Ba32_aug = Aa*Ba_aug*SO.a22 + Ba_aug*SO.a21;
                        
                        % Find input normal form of augmented square system
                        [A_temp, B_temp, C_temp, U_temp, ~, ~]...
                                = SimVFA.findNormalForm(Aa',Ca',Ba_aug',vrel_aug,nrel_aug,1);
                        % make a new system from dual - the real input normal form
                        Uinv  = U_temp'; % eigs at origin in Aa moved to LHP
                        Atilt = A_temp'; % Atilt=U*A*Uinv
                        Btilt = C_temp'; % Btilt=U*B;
                        Ctilt = B_temp'; % Ctilt=C*Uinv;
                        Dtilt = zeros(size(Ctilt,1),size(Btilt,2));
                        clear A_temp B_temp C_temp U_temp;

                        % Coordinate transform for relative degree 1 input path
                        Bi31  = SO.a22*Atilt^2*Btilt + SO.a21*Atilt*Btilt + SO.a20*Btilt;
                        Bi3   = SO.a22*Btilt(:,1:num_input);
                        tzero(Atilt,Bi31,Ctilt,Dtilt)
                        
                        % make sure transformed transformed squared-up system is minimum phase
                        if SimVFA.checkNegative(tzero(Atilt,Bi31,Ctilt,Dtilt))
                            Rs = eye(num_output_i);

                            % postcomp==1 gives unitary S (different than paper)
                            [S,C_bar,~] = SimVFA.findPostComp(Bi31,Ctilt,Rs,SO.postcomp);
                            [S1, ~] = SimVFA.findSquareDown(Bi3,Ctilt,S);

                            % F is R^(-1) in paper
                            [F,~] = SimVFA.pickFSPR(Atilt,Bi31,C_bar,SO.q0,SO.epsilon,SO.s0);
                            Lis = Bi31*F*S;
                            L = Uinv*Lis; % used in sim -- transformed back into real coordinates

                        end

                        % baseline LQR design
                        K = lqr(Aa, Ba, SO.qk, SO.rk);

                        % Add to lookup tables
                        Aa_M(:,:,eta_i+1) = Aa;
                        Ba_M(:,:,eta_i+1) = Ba;
                        Ba3_aug_M(:,:,eta_i+1)  = Ba3_aug;
                        Ba32_aug_M(:,:,eta_i+1) = Ba32_aug;
                        Ba3_M(:,:,eta_i+1)  = Ba3;
                        Ba32_M(:,:,eta_i+1) = Ba2;
                        Caz_M(:,:,eta_i+1)  = Caz;
                        K_M(:,:,eta_i+1)    = K;
                        L_M(:,:,eta_i+1)    = L;
                        Rinv_M(:,:,eta_i+1) = F;
                        S_M(:,:,eta_i+1)    = S;
                        S1_M(:,:,eta_i+1)   = S1;

                    end

                    tables.Aa_M       = Aa_M;
                    tables.Ba_M       = Ba_M;
                    tables.Ba3_aug_M  = Ba3_aug_M;
                    tables.Ba32_aug_M = Ba32_aug_M;
                    tables.Ba3_M      = Ba3_M;
                    tables.Ba32_M     = Ba32_M;
                    tables.Caz_M      = Caz_M;
                    tables.K_M        = K_M;
                    tables.L_M        = L_M;
                    tables.Rinv_M     = Rinv_M;
                    tables.S_M        = S_M;
                    tables.S1_M       = S1_M;

                else % first-order actuator model
                    % Lookup tables for matrices vs. dihedral angle 
                    Aa_M   = zeros([10,10,length(TP.etasweep)]);
                    Ba_M   = zeros([10,2,length(TP.etasweep)]);
                    Ba_aug_M = zeros([10,3,length(TP.etasweep)]);
                    Caz_M  = zeros([2,10,length(TP.etasweep)]);
                    K_M    = zeros([2,10,length(TP.etasweep)]);
                    L_M    = zeros([10,3,length(TP.etasweep)]);
                    Rinv_M = zeros([3,3,length(TP.etasweep)]);
                    S_M    = zeros([3,3,length(TP.etasweep)]);
                    S1_M   = zeros([2,3,length(TP.etasweep)]);
                    
                    % iterate through systems linearized at fixed dihedrals
                    for eta_i = (min(TP.etasweep):max(TP.etasweep))
                        % Set plant parameters

                        Ap = TP.A_hold(:,:,eta_i+1);   % linearized state matrix (at selected dihedral)
                        Bp = TP.B_hold(:,1:5,eta_i+1); % linearized input matrix (at selected dihedral)
                        Cp = [eye(6), zeros(6,1)]; % all states measured in Cp except dihedral rate

                        % augment plant with (nominal) first-order actuator dynamics
                        % and remove altitude from states
                        A = [Ap(SO.i_state_sel,SO.i_state_sel), Bp(SO.i_state_sel,SO.i_input_sel);
                             zeros(length(SO.i_input_sel),length(SO.i_state_sel)), -SO.D_1];
                        B = [0*Bp(SO.i_state_sel,SO.i_input_sel);
                             SO.D_1]; % input matrix completely changed with actuator dynamics
                        % C selects only pitch rate (q) as measurement
                        C  = [Cp(SO.i_output,SO.i_state_sel), zeros(length(SO.i_output), length(SO.i_input_sel))];
                        % Cz selects dihedral angle and vertical acceleration as measurements              
                        Cz = [0,0,0,0,1,0,0,0;
                              0,TP.Vinitial*Ap(2,2),0,0,0,0,TP.Vinitial*Bp(2,SO.i_input_sel)];

                        num_input = size(B,2);
                        num_cmd = 2;

                        % Augment plant with integral error (for two tracked states)
                        index_output = [1,2]; % outputs to track (from Cz)

                        % augmented system is used in reference model and control in sim.
                        % the integral error states are appended after actuator dynamics.
                        % Am = Aa - Ba*K,   Bm = Baz,   Cm = Ca
                        Aa = [A,zeros(length(A),length(index_output));
                              Cz,zeros(length(index_output))];           % "A" in paper
                        Ba = [B; zeros(length(index_output),num_input)]; % "B_3" in paper
                        % Ca selects pitch rate and integral errors as measurements
                        Ca  = [C, zeros(size(C,1),length(index_output));
                               zeros(length(index_output),length(A)),eye(length(index_output))]; % "C" in paper
                        Caz = [Cz, zeros(num_cmd)]; % Caz selects dihedral angle and vertical acceleration as measurements
                        Da  = zeros(size(Ca,1),size(Ba,2)); % no direct feedthrough

                        num_output_i = size(Ca,1);

                        if (eta_i == SO.eta_nom)
                            SO.Cz = Cz;

                            SO.Baz = [0*B; -eye(num_input)]; % "B_z" in paper (reference model input for r)
                            SO.Ca = Ca;
                            
                            SO.num_state = length(A);
                            SO.num_input = num_input;
                            SO.num_state_i  = length(Aa); % states in augmented system
                            SO.num_output_i = num_output_i;

                            SimVFA.checkNegative(tzero(Aa,Ba,Ca,Da)); % check whether sys is min phase
                            SimVFA.checkCtrbObsv(Aa,Ba,Ca);    % check that augmented system is ctrb and obsv
                            SimVFA.checkRelDeg(Aa,Ba,Ca,Da,2); % make sure uniform relative degree three
                                                            
                            if (SO.mActOrder == SO.pActOrder) % for mismatch, this is computed later
                                % B1 takes the two desired input paths from Bp (w/o actuators),
                                % removes altitude as state, and augments states with actuator states
                                B1   = [Bp(SO.i_state_sel,SO.i_input_sel);
                                        zeros(length(SO.i_input_sel),length(SO.i_input_sel));
                                        inv(SO.D_1)*Cz*B];
                                % A^{*} in paper
                                SO.Asim = Aa + Ba * SO.Psi2 + B1 * SO.Psi1;
                            end
                        end
                        
                        % Add fictitious inputs (squaring up): 
                        [Ba_aug, ~] = SimVFA.squareUpB(Aa, Ba, Ca, 1, 500);
                        Da_aug = [Da, [0,0,0]']; % no direct feedthrough

                        [nrel_aug, vrel_aug, ~] = SimVFA.checkRelDeg(Aa,Ba_aug,Ca,Da_aug,2);

                        % this shouldn't change anything for uniform relative degree
                        [vrel_aug, Perm_aug] = SimVFA.sortPerm(vrel_aug);
                        Ba_aug = (Perm_aug*Ba_aug')';

                        % Find input normal form of augmented square system
                        [A_temp, B_temp, C_temp, U_temp, ~, ~]...
                                = SimVFA.findNormalForm(Aa',Ca',Ba_aug',vrel_aug,nrel_aug,1);
                        % make a new system from dual - the real input normal form
                        Uinv  = U_temp'; % eigs at origin in Aa moved to LHP
                        Atilt = A_temp'; % Atilt=U*A*Uinv
                        Btilt = C_temp'; % Btilt=U*B;
                        Ctilt = B_temp'; % Ctilt=C*Uinv;
                        Dtilt = zeros(size(Ctilt,1),size(Btilt,2));
                        clear A_temp B_temp C_temp U_temp;

                        % Coordinate transform for relative degree 1 input path
                        Bi21 = SO.a11*Atilt*Btilt + SO.a10*Btilt;
                        Bi2  = SO.a11*Btilt(:,1:num_input);
                        
                        % make sure transformed transformed squared-up system is minimum phase
                        if SimVFA.checkNegative(tzero(Atilt,Bi21,Ctilt,Dtilt))
                            Rs = eye(num_output_i);

                            % postcomp==1 gives unitary S (different than paper)
                            [S,C_bar,~] = SimVFA.findPostComp(Bi21,Ctilt,Rs,SO.postcomp);
                            [S1, ~] = SimVFA.findSquareDown(Bi2,Ctilt,S);

                            % F is R^(-1) in paper
                            [F,~] = SimVFA.pickFSPR(Atilt,Bi21,C_bar,SO.q0,SO.epsilon,SO.s0);
                            Lis = Bi21*F*S;
                            L = Uinv*Lis; % used in sim -- transformed back into real coordinates

                        end

                        % baseline LQR design
                        K = lqr(Aa, Ba, SO.qk, SO.rk);

                        % Add to lookup tables
                        Aa_M(:,:,eta_i+1)   = Aa;
                        Ba_M(:,:,eta_i+1)   = Ba;
                        Ba_aug_M(:,:,eta_i+1) = Ba_aug;
                        Caz_M(:,:,eta_i+1)  = Caz;
                        K_M(:,:,eta_i+1)    = K;
                        L_M(:,:,eta_i+1)    = L;
                        Rinv_M(:,:,eta_i+1) = F;
                        S_M(:,:,eta_i+1)    = S;
                        S1_M(:,:,eta_i+1)   = S1;

                    end

                    tables.Aa_M      = Aa_M;
                    tables.Ba_M      = Ba_M;
                    tables.Ba_aug_M  = Ba_aug_M;
                    tables.Caz_M     = Caz_M;
                    tables.K_M       = K_M;
                    tables.L_M       = L_M;
                    tables.Rinv_M    = Rinv_M;
                    tables.S_M       = S_M;
                    tables.S1_M      = S1_M;

                end
                
                if saveTables
                    save(saveFile, 'tables', 'SO');        
                end
            end
            TP.tables = tables;
            vfa.trimPts = TP; % save updated struct
            vfa.simOpt = SO;
        end
        
        function genController(vfa)
            SO = vfa.simOpt; % to be saved at end of function
            TP = vfa.trimPts;
            
            % no actuator dynamics or integral error in these
            SO.input_hold = TP.trim_inputs(SO.eta_nom+1, :); % initial inputs (at nominal dihedral)
            SO.state_hold = TP.trim_states(SO.eta_nom+1, :); % initial states (at nominal dihedral)
            Ap = TP.A_hold(:,:,SO.eta_nom+1);   % linearized state matrix (at nominal dihedral)
            Bp = TP.B_hold(:,1:5,SO.eta_nom+1); % linearized input matrix (at nominal dihedral)
            SO.Bp = Bp;
            Cp = [eye(6), zeros(6,1)];          % all states measured in Cp except dihedral rate
            Dp = zeros(size(Cp,1), size(Bp,2)); % no direct feedthrough in plant

            % use this matrix in sim to make control actions go through correct input paths
            SO.inputselect = SimVFA.selectionMatrix(5, length(SO.i_input_sel), SO.i_input_sel);
            
            % Augment plant with integral error (for two tracked states)
            index_output = [1,2]; % outputs to track (from Cz)
            num_cmd = 2;
            num_state = SO.num_state;
            num_input = SO.num_input;
            num_state_i = SO.num_state_i;
            num_output_i = SO.num_output_i;

            if (SO.mActOrder == 2) % second-order actuator model for control
                
                % Reference Model Setup
                SO.Si1 = [eye(num_state-2*num_input),zeros(num_state-2*num_input,num_state_i-num_state+2*num_input)];
                SO.Si3 = [eye(num_state),zeros(num_state,num_state_i-num_state)];

                % Simulation parameters
                % high-order tuner gains
                SO.Gamma.l = 5000*eye(num_input);
                SO.Gamma.vl = 5000*eye(num_input);
                SO.Gamma.p1 = 0.2*eye(num_state-2*num_input);
                SO.Gamma.p2 = 0.2*eye(num_state-2*num_input);
                SO.Gamma.p3 = 0.2*eye(num_state);
                SO.Gamma.p31 = 1000*eye(num_output_i);
                SO.Gamma.p31xm = SO.Gamma.p31(1:num_input,1:num_input);
                SO.Gamma.p32 = 1000*eye(num_output_i);

                % intial conditions
                SO.lambda_0 = eye(num_input);
                SO.vlambda_0 = eye(num_input);
                SO.psi1_0 = zeros(num_state-2*num_input,num_input);
                SO.psi2_0 = zeros(num_state-2*num_input,num_input);
                SO.psi3_0 = zeros(num_state,num_input);
                SO.psi31_0 = zeros(num_output_i,num_output_i);
                SO.psi31xm_0 = zeros(num_input,num_input);
                SO.psi32_0 = zeros(num_output_i,num_output_i);
                SO.xm_0 = zeros(num_state_i,1);

                % more high-order tuner gains
                mu = 0.02;
                SO.mu.lambda = mu; SO.mu.vlambda = mu; SO.mu.psi1 = mu; 
                SO.mu.psi2 = mu; SO.mu.psi3 = mu; SO.mu.psi31  = mu; 
                SO.mu.psi32 = mu;

                % actuator dynamics (real dynamics, not nominal)
                SO.Aact = [zeros(num_input), eye(num_input);
                        -SO.w_act_a^2*eye(num_input), -2*SO.w_act_a*SO.zeta_act_a*eye(num_input)];
                SO.Bact = [zeros(num_input);SO.w_act_a^2*eye(num_input)];
                SO.Bact_x = SO.Asim(num_state-2*num_input+1:num_state,1:num_state-2*num_input);
                SO.Cact = [eye(num_input),zeros(num_input)];
                SO.Cact_dot = [zeros(num_input),eye(num_input)];

            elseif (SO.pActOrder == 2) % true second-orde actuators, first-order model

                % Reference Model Setup
                SO.Si1 = [eye(num_state-num_input),zeros(num_state-num_input,num_state_i-num_state+num_input)];
                SO.Si2 = [eye(num_state),zeros(num_state,num_state_i-num_state)];

                % Simulation parameters
                % tuner gains
                SO.Gamma.l = 2000*eye(num_input);
                SO.Gamma.p1 = 0.2*eye(num_state-num_input);
                SO.Gamma.p2 = 0.2*eye(num_state);
                SO.Gamma.p21 = 400*eye(num_output_i);

                % intial conditions
                SO.lambda_0 = eye(num_input);
                SO.psi1_0 = zeros(num_state-num_input,num_input);
                SO.psi2_0 = zeros(num_state,num_input);
                SO.psi21_0 = zeros(num_output_i,num_output_i);
                SO.xm_0 = zeros(num_state_i,1);

                % for second-order actuators with first-order model
                % this is all here to calculate Asim used in Bact_x
                % i.e. actuator coupling with nonlinear VFA model
                act.A = [Ap(SO.i_state_sel,SO.i_state_sel), Bp(SO.i_state_sel,SO.i_input_sel), zeros(length(SO.i_state_sel),length(SO.i_input_sel));
                     zeros(length(SO.i_input_sel),length(SO.i_state_sel)+length(SO.i_input_sel)), eye(length(SO.i_input_sel));
                     zeros(length(SO.i_input_sel),length(SO.i_state_sel)), -SO.D_1, -SO.D_2];
                act.B = [0*Bp(SO.i_state_sel,SO.i_input_sel);
                     zeros(length(SO.i_input_sel));
                     SO.D_1]; % input matrix completely changed with actuator dynamics
                act.Cz = [0,0,0,0,1,0,0,0,0,0;
                      0,TP.Vinitial*Ap(2,2),0,0,0,0,TP.Vinitial*Bp(2,SO.i_input_sel),0,0];
                act.D  = Dp(SO.i_output, SO.i_input_sel);     % no direct feedthrough
                act.num_state  = length(act.A); 
                act.num_input  = size(act.B,2); % two inputs
                act.Aa = [act.A,zeros(length(act.A),length(index_output));
                      act.Cz,zeros(length(index_output))];           % "A" in paper
                act.Ba = [act.B; zeros(length(index_output),act.num_input)]; % "B_3" in paper
                act.B1 = [Bp(SO.i_state_sel,SO.i_input_sel);
                        zeros(2*length(SO.i_input_sel),length(SO.i_input_sel));
                        act.D; TP.Vinitial*Bp(2,SO.i_input_sel)];
                act.Asim = act.Aa + act.Ba*SO.Psi3_act + act.B1*SO.Psi1_act; % A^{*} in paper

                % actuator dynamics (real dynamics, not nominal)
                SO.Aact = [zeros(act.num_input), eye(act.num_input);
                        -SO.w_act_a^2*eye(act.num_input), -2*SO.w_act_a*SO.zeta_act_a*eye(act.num_input)];
                SO.Bact = [zeros(act.num_input);SO.w_act_a^2*eye(act.num_input)];
                SO.Bact_x = act.Asim(act.num_state-2*act.num_input+1:act.num_state,1:act.num_state-2*act.num_input);
                SO.Cact = [eye(act.num_input),zeros(act.num_input)];

            else

                % Reference Model Setup
                SO.Si1 = [eye(num_state-num_input),zeros(num_state-num_input,num_state_i-num_state+num_input)];
                SO.Si2 = [eye(num_state),zeros(num_state,num_state_i-num_state)];

                % Simulation parameters
                % tuner gains
                SO.Gamma.l = 2000*eye(num_input);
                SO.Gamma.p1 = 0.2*eye(num_state-num_input);
                SO.Gamma.p2 = 0.2*eye(num_state);
                SO.Gamma.p21 = 400*eye(num_output_i);

                % intial conditions
                SO.lambda_0 = eye(num_input);
                SO.psi1_0 = zeros(num_state-num_input,num_input);
                SO.psi2_0 = zeros(num_state,num_input);
                SO.psi21_0 = zeros(num_output_i,num_output_i);

                SO.xm_0 = zeros(num_state_i,1);

                % actuator dynamics (real dynamics, not nominal)
                SO.Aact = SO.eig_act*eye(2);
                SO.Bact = -SO.eig_act*eye(2);
                SO.Bact_x = SO.Asim(num_state-num_input+1:num_state,1:num_state-num_input);
                SO.Cact = eye(num_input);

            end
                        
            % command filter
            SO.Acmd = [zeros(num_cmd),eye(num_cmd);
                    -SO.w_cmd^2*eye(num_cmd),-2*SO.w_cmd*SO.zeta_cmd*eye(num_cmd)];
            SO.Bcmd = [zeros(num_cmd);SO.w_cmd^2*eye(num_cmd)];
            SO.Ccmd = [eye(num_cmd),zeros(num_cmd)];
            SO.Ccmd_dot = [zeros(num_cmd),eye(num_cmd)];

            % process noise
            SO.d_seed = (1:num_input)';
            SO.d_sam  = 1;
            SO.d_mean = 0;

            % tracking feedthrough noise (Z)
            SO.n_seed = (1:num_input)';
            SO.n_sam  = 1;
            SO.n_mean = 0;    

            % gust model properties
            SO.samplet = 0.5;
            SO.tpower = 0;
            
            vfa.simOpt = SO; % save updated struct
        end
        
        function runSim(vfa)
        % run the simulation
            disp('Simulation will start');
            start = tic;
            vfa.simOutObj = sim(vfa.simInObj);
            run_time = toc(start);
            disp(['Simulation finished in ', num2str(run_time), ' seconds']);
            if (vfa.simOutObj.t_sim(end)/vfa.simOpt.tsim < 0.99)
                disp('Simulation exited early due to error/signal divergence');
            end
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
            h4=plot(TP.trim_eig_real(21,:),TP.trim_eig_imag(21,:),'vb','markersize',8,'LineWidth',2);
            h=legend([h2 h3 h4],'$\eta=0$ deg','$\eta=11$ deg','$\eta=20$ deg');
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
            h3=plot(TP.trim_eig_real(12,:),TP.trim_eig_imag(12,:),'sr','markersize',8,'LineWidth',2);
            h4=plot(TP.trim_eig_real(21,:),TP.trim_eig_imag(21,:),'vb','markersize',8,'LineWidth',2);
            h=legend([h2 h3 h4],'$\eta=0$ deg','$\eta=11$ deg','$\eta=20$ deg');
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
            plot(TP.etasweep,180/pi*TP.trim_inputs(:,2),'-k','markersize',12)
            plot(TP.etasweep,180/pi*TP.trim_inputs(:,3),'--k','markersize',12)
            plot(TP.etasweep,180/pi*TP.trim_inputs(:,4),'.k','markersize',12)
            plot(TP.etasweep,180/pi*TP.trim_inputs(:,5),':k','markersize',9,'LineWidth',2)
            plot(TP.etasweep,TP.trim_inputs(:,1)/5,'ok','markersize',6)
            axis([0 max(TP.etasweep) -30 40])
            ylabel('Trim (Control Surface Inputs [deg], Thrust [5 lbf])','fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)
            xlabel('Dihedral angle [deg]','fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)
            h=legend('$\delta_{a_c}$','$\delta_{a_o}$','$\delta_{e_c}$','$\delta_{e_o}$','Thrust');
            set(h,'fontsize',vfa.pltOpt.legfontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname,'Interpreter','Latex','Location','SouthEast')
            set(gca,'fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)


            % Figure 4: Angle of attack trim
            figure;
            hold on;
            box on;
            plot(TP.etasweep,TP.trim_states(:,2)*180/pi,'k')
            ylabel('Trim angle of attack [deg])','fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)
            xlabel('Dihedral angle [deg]','fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)
            axis([0 max(TP.etasweep) 0 10])
            set(gca,'fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)

        end
        
        function plotSim(vfa)
        % plot output of simulation
            SOO = vfa.simOutObj;
            
            tsim = vfa.simOpt.tsim;
            
            c1 = [0, 0.4470, 0.7410];
            c2 = [0.466, 0.674, 0.188];

            figure('Position',[1,1, 800, 640]);
            subplot(4,1,1)
            plot(SOO.t_sim, SOO.r_cmd(1,:)*180/pi + vfa.simOpt.eta_nom, 'LineWidth', 1.5, 'Color', c1)
            hold on; grid on; 
            plot(SOO.t_sim, SOO.z(1,:)*180/pi + vfa.simOpt.eta_nom, 'LineWidth', 1.5, 'LineStyle', '-', 'Color', c2)
            xlim([0 tsim])
            ylim([min(SOO.r_cmd(1,:)*180/pi)+vfa.simOpt.eta_nom-1 max(SOO.r_cmd(1,:)*180/pi)+vfa.simOpt.eta_nom+1])
            if (vfa.simOutObj.t_sim(end)/vfa.simOpt.tsim < 0.99)
                line([SOO.t_sim(end) SOO.t_sim(end)],ylim,'Color',[0.6 0.6 0.6],'LineStyle','--', 'LineWidth', 1, 'HandleVisibility', 'off');
            end
            title('Dihedral (deg)','Interpreter','Latex')
            h=legend('Command', 'Output');
            set(h,'fontsize',vfa.pltOpt.legfontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname,'Interpreter','Latex','Location','SouthWest'); legend('boxoff')
            set(gca,'fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)

            subplot(4,1,2)
            plot(SOO.t_sim, SOO.r_cmd(2,:), 'LineWidth', 1.5, 'Color', c1)
            hold on; grid on; 
            plot(SOO.t_sim, SOO.z(2,:), 'LineWidth', 1.5, 'LineStyle', '-', 'Color', c2)
            xlim([0 tsim])
            ylim([min(SOO.r_cmd(2,:))-1 max(SOO.r_cmd(2,:))+1])
            if (vfa.simOutObj.t_sim(end)/vfa.simOpt.tsim < 0.99)
                line([SOO.t_sim(end) SOO.t_sim(end)],ylim,'Color',[0.6 0.6 0.6],'LineStyle','--', 'LineWidth', 1, 'HandleVisibility', 'off');
            end
            title('Vertical Accel (ft/s$^2$)','Interpreter','Latex')
            set(gca,'fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)

            subplot(4,1,3)
            plot(SOO.t_sim, SOO.u_p(1,:)*180/pi, 'LineWidth', 1.5, 'Color', c1); grid on; hold on;
            xlim([0 tsim])
            ylim([0 3])
            if (vfa.simOutObj.t_sim(end)/vfa.simOpt.tsim < 0.99)
                line([SOO.t_sim(end) SOO.t_sim(end)],ylim,'Color',[0.6 0.6 0.6],'LineStyle','--', 'LineWidth', 1, 'HandleVisibility', 'off');
            end
            title('Outer Aileron (deg)','Interpreter','Latex')
            set(gca,'fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)

            subplot(4,1,4)
            plot(SOO.t_sim, SOO.u_p(2,:)*180/pi, 'LineWidth', 1.5, 'Color', c1); grid on; hold on;
            xlim([0 tsim])
            ylim([-3 0])
            if (vfa.simOutObj.t_sim(end)/vfa.simOpt.tsim < 0.99)
                line([SOO.t_sim(end) SOO.t_sim(end)],ylim,'Color',[0.6 0.6 0.6],'LineStyle','--', 'LineWidth', 1, 'HandleVisibility', 'off');
            end
            title('Center Elevator (deg)','Interpreter','Latex')
            xlabel('Time (s)')
            set(gca,'fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)

            % plot adaptive parameters (matrix norms to reduce dimensionality)
            if (vfa.simOpt.adaFlag)
                steps = length(SOO.t_sim);

                if (vfa.simOpt.mActOrder == 2) % second-order actuator dynamics for control
                    norm_lambda_ada = zeros(steps, 1);
                    norm_psi1_ada   = zeros(steps, 1);
                    norm_psi2_ada   = zeros(steps, 1); 
                    norm_psi31_ada  = zeros(steps, 1);
                    norm_psi32_ada  = zeros(steps, 1);
                    norm_psi3_ada   = zeros(steps, 1);

                    for i=1:steps
                        norm_lambda_ada(i) = norm(SOO.lambda_ada(:,:,i));
                        norm_psi1_ada(i) = norm(SOO.psi1_ada(:,:,i));
                        norm_psi2_ada(i) = norm(SOO.psi2_ada(:,:,i));
                        norm_psi31_ada(i) = norm(SOO.psi31_ada(:,:,i));
                        norm_psi32_ada(i) = norm(SOO.psi32_ada(:,:,i));
                        norm_psi3_ada(i) = norm(SOO.psi3_ada(:,:,i));
                    end

                    norm_lambda_ada = norm_lambda_ada/norm_lambda_ada(end);
                    norm_psi1_ada   = norm_psi1_ada/norm_psi1_ada(end);
                    norm_psi2_ada   = norm_psi2_ada/norm_psi2_ada(end);
                    norm_psi31_ada  = norm_psi31_ada/norm_psi31_ada(end);
                    norm_psi32_ada  = norm_psi32_ada/norm_psi32_ada(end);
                    norm_psi3_ada   = norm_psi3_ada/norm_psi3_ada(end);

                    norms = [norm_lambda_ada, norm_psi1_ada, norm_psi2_ada, ...
                            norm_psi31_ada, norm_psi32_ada, norm_psi3_ada];

                    figure('Position',[100,100, 800, 400]);
                    plot(SOO.t_sim, norms, 'LineWidth', 1);
                    xlim([0 tsim]); grid on;
                    title('Normalized Learned Parameters')
                    xlabel('Time (s)')
                    h=legend('$\|\underline{\it{\Lambda}}\|$', '$\|\underline{\Psi}_1\|$', '$\|\underline{\Psi}_2\|$', '$\|\psi_3^1\|$', '$\|\psi_3^2\|$', '$\|\underline{\Psi}_3\|$');
                    set(h,'fontsize',vfa.pltOpt.legfontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname,'Interpreter','Latex','Location','SouthEast'); legend('boxoff')
                    set(gca,'fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)
                else % first-order actuator dynamics
                    norm_lambda_ada = zeros(steps, 1);
                    norm_psi1_ada   = zeros(steps, 1);
                    norm_psi2_ada   = zeros(steps, 1); 
                    norm_psi21_ada = zeros(steps, 1);

                    for i=1:steps
                        norm_lambda_ada(i) = norm(SOO.lambda_ada(:,:,i));
                        norm_psi1_ada(i) = norm(SOO.psi1_ada(:,:,i));
                        norm_psi2_ada(i) = norm(SOO.psi2_ada(:,:,i));
                        norm_psi21_ada(i) = norm(SOO.psi21_ada(:,:,i));
                    end

                    norm_lambda_ada = norm_lambda_ada/norm_lambda_ada(end);
                    norm_psi1_ada   = norm_psi1_ada/norm_psi1_ada(end);
                    norm_psi2_ada   = norm_psi2_ada/norm_psi2_ada(end);
                    norm_psi21_ada = norm_psi21_ada/norm_psi21_ada(end);

                    norms = [norm_lambda_ada, norm_psi1_ada, norm_psi2_ada, norm_psi21_ada];

                    figure('Position',[100,100, 800, 400]);
                    plot(SOO.t_sim, norms, 'LineWidth', 1);
                    xlim([0 tsim]); grid on;
                    title('Normalized Learned Parameters')
                    xlabel('Time (s)')
                    h=legend('$\|\underline{\it{\Lambda}}\|$', '$\|\underline{\Psi}_1\|$', '$\|\underline{\Psi}_2\|$', '$\|\psi_{2}^1\|$');
                    set(h,'fontsize',vfa.pltOpt.legfontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname,'Interpreter','Latex','Location','SouthEast'); legend('boxoff')
                    set(gca,'fontsize',vfa.pltOpt.fontsize,'fontweight',vfa.pltOpt.weight,'fontname',vfa.pltOpt.fontname)
                end
            end
        end
    end
    
    methods (Static) % these are all just helper functions
        function [ctrbM, obsM] = checkCtrbObsv(A,B,C,mode)
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
                N = SimVFA.findLeftAnni(B);  % NB = 0
                M = SimVFA.findRightAnni(C); % CM = 0
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
        
        function [S,C_bar,CB_bar] = findPostComp(B,C,R0,ver)
        % Post-compensator
            if ver == 1
                R0_invsqrt = inv(sqrtm(R0));
                [U,~,V] = svd(B'*C'*R0_invsqrt);
                W = (U*V')'; % directions from SVD of (CB)^T
                S = W'*R0_invsqrt;

                C_bar  = S*C;
                CB_bar = C_bar*B;

                SimVFA.checkPD(CB_bar);
                SimVFA.checkSym(CB_bar);
            elseif ver == 2
                S = R0*inv(C*B);

                C_bar  = S*C;
                CB_bar = C_bar*B;

                SimVFA.checkPD(CB_bar);
                SimVFA.checkSym(CB_bar);
            else
                S = R0*(C*B)';

                C_bar  = S*C;
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
        %   See paper "Synthesis of Static Output Feedback SPR Systems via
        %   LQR Weighting Matrix Design" for some insight
        
            if (nargin==5 || isempty(s0))
                s0 = 0.2;
            end

            name = [inputname(1),',',inputname(2),',',inputname(3)];

            CLname = ['(',inputname(1),'-',inputname(2),'F',inputname(3),')',',',inputname(2),',',inputname(3)];

            A_eta = A+s0*eye(length(A));
            n = length(A);
            m = size(B,2);

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
                    P   = C'*inv(CB)*C+N'*P0*N; % PB = C'
                    Q   = -A'*P - P*A;
                    Q_test = SimVFA.checkPD(Q);
                    if Q_test==1
                        F = zeros(size(B,2),size(C,1));
                        disp(strcat('{',name,'} is already SPR'));
                        return

                    else

                        Rlinv = inv(CB)*(C*A_eta*B+(C*A_eta*B)')*inv(CB)+epsilon*eye(m);
%                         RQ0 = (C*A*M+(N*A*B)'*P0)*inv(Q0)*(C*A*M+(N*A*B)'*P0)';
%                         Rlinv = inv(CB)*(C*A*B+(C*A*B)' + RQ0)*inv(CB)+epsilon*eye(m);
                        SimVFA.checkPD(Rlinv);
                        Ql = -A'*P - P*A + P*B*Rlinv*B'*P; % Algebraic Riccati Eqn
                        Ql_test = SimVFA.checkPD(Ql);

                        F = Rlinv;
                        if Ql_test==1
                            disp(strcat('Found F s.t. {',CLname,'} is SPR'));
                        else
                            disp('F(q0, epsilon) is not SPR');
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
                            disp('F(q0, epsilon) is not SPR');
                        end
                    end
                end
            else
                disp(strcat('(',inputname(3),'*',inputname(2),') is not SPD, or the system is not minimum phase'));
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

        function [B_fin, SU_zeros] = squareUpB(A_orig, B_orig, C_orig, q0, r0)
            % square up procedure for uniform relative degree 2 or 3 system to add
            % ficticious inputs (i.e. augment B_orig)

            % system transposed to follow procedure to augment C
            A = A_orig';
            B = C_orig';
            C = B_orig';

            n = length(A); m = size(B,2); p = size(C,1);
            [At, Bt, Ct, T] = SimVFA.sqUpTransform(A,B,C);

            A21 = At(m+1:n,1:m);
            A22 = At(m+1:n,m+1:n);

            C11 = Ct(:,1:m);
            C12 = Ct(:,m+1:n);

            if (rank(A21)==m && rank(C11)==0)
                [C2, C1, ~] = SimVFA.findSquareUpInner(A22,A21,C12,q0,r0);
                Ct_aug = [C1, C2];
            else
                error('Squaring-Up Error: Rank(A21) < m');
            end

            C_aug = Ct_aug * T;
            SU_zeros = tzero(A,B,C_aug, zeros(size(Ct_aug,1),size(Bt,2)));

            B_fin = C_aug';
        end

        function [At, Bt, Ct, T] = sqUpTransform(A,B,C)
            % Tranform to controller canonical form, as in the paper
            % "Squaring-Up Method in the Presence of Transmission Zeros"
            B_perp = (null(B'));
            B_pinv = pinv(B);
            T = [B_pinv;B_perp'];

            At = T*A*inv(T);
            Bt = T*B;
            Ct = C*inv(T);

            T(abs(T)<1e-12) = 0;
            At(abs(At)<1e-9) = 0;
            Bt(abs(Bt)<1e-9) = 0;
            Ct(abs(Ct)<1e-9) = 0;
        end

        function [C_aug, D_aug, SU_zeros] = findSquareUpInner(A,B,C,q0,r0)
            % This is needed for relative degree 2 and 3 systems where CB is not full rank

            [At, Bt, Ct, T] = SimVFA.sqUpTransform(A,B,C);

            n=length(At); m=size(Bt,2); p=size(Ct,1);

            if (rank(Bt) ~= size(Bt,2))
                error('Squaring-Up Error: Simplified procedure requires B to have full rank');
            end


            A21 = At(m+1:n,1:m);
            A22 = At(m+1:n,m+1:n);

            C11 = Ct(:,1:m);
            C12 = Ct(:,m+1:n);

            if (rank(A21)==m && rank(C11)==0) % for relative degree 3 (recurse)
                [C2, C1, ~] = SimVFA.findSquareUpInner(A22,A21,C12,q0,r0);
                Ct_aug = [C1, C2];
                C_aug = Ct_aug * T;
                D_aug = zeros(m);
                SU_zeros = tzero(A,B,C_aug, zeros(size(Ct_aug,1),size(Bt,2)));
            else % for relative degree 2
                C21 = (null(C11))';
                C1 = [C11; C21];

                C2tilde = [C12; zeros(m-p,n-m)];

                A22tilde = A22-A21*inv(C1)*C2tilde;

                B_pseudo = A21*inv(C1);
                B_pseudo_2 = B_pseudo(:, end-(m-p-1):end);

                Qtilde = q0*eye(length(A22tilde));
                Rtilde = r0*eye(size(B_pseudo_2,2));
                C22 = lqr(A22tilde,B_pseudo_2,Qtilde,Rtilde);

                C2 = [C12; C22];

                Ct_aug = [C1, C2];
                C_aug = Ct_aug*T;
                C_aug(abs(C_aug)<1e-9) = 0; % round numerical noise to 0
                D_aug = zeros(m);

                SU_zeros = tzero(A,B,C_aug,D_aug);
            end
        end
    end
end
