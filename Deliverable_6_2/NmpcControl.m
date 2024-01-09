classdef NmpcControl < handle

%% Key Properties

% - **solver**: CasADi solver object for optimization.
% 
% - **nx, nu, N**: Sizes and horizon length for states, inputs, and the control horizon, respectively.
% 
% - **nlp_x0**: Initial guess for decision variables.
% 
% - **nlp_lbx, nlp_ubx**: Lower and upper bounds for decision variables.
% 
% - **nlp_lbg, nlp_ubg**: Lower and upper bounds for equality and inequality constraints.
% 
% - **nlp_p**: Parameters for the optimization problem.
% 
% - **T_opt**: Time grid for the optimization horizon.
% 
% - **sol**: Solution structure containing the optimization results.
% 
% - **idx**: Structure storing indices for extracting variables from the solution.
% 
% - **rocket**: Object containing rocket system parameters.
% 
% - **expected_delay**: Expected delay in the system.
% 
% - **mem_u**: Memory for storing past control inputs.
% 
% - **nlp_lam_x0, nlp_lam_g0**: Lagrange multipliers for warmstarting.

%% Example Usage

% ```matlab
% % Example instantiation of NmpcControl class
% rocket = RocketParameters(); % Replace with actual rocket parameter object
% tf = 10; % Final time for MPC horizon
% controller = NmpcControl(rocket, tf);


    properties
        solver
        nx, nu, N
        nlp_x0
        nlp_lbx, nlp_ubx
        nlp_lbg, nlp_ubg
        nlp_p
        
        T_opt
        sol
        idx
        
        % Delay compensation
        rocket
        expected_delay
        mem_u
        
        % Warmstart
        nlp_lam_x0
        nlp_lam_g0
    end
    
    methods

        function obj = NmpcControl(rocket, tf, expected_delay)
            
            if nargin < 3, expected_delay = 0; end
           
            import casadi.*
            
            N_segs = ceil(tf/rocket.Ts); % MPC horizon
            nx = 12; % Number of states
            nu = 4;  % Number of inputs
            
            % Decision variables (symbolic)
            N = N_segs + 1; % Index of last point
            X_sym = SX.sym('X_sym', nx, N); % state trajectory
            U_sym = SX.sym('U_sym', nu, N-1); % control trajectory)
            
            % Parameters (symbolic)
            x0_sym  = SX.sym('x0_sym', nx, 1);  % initial state
            ref_sym = SX.sym('ref_sym', 4, 1);  % target position
            
            % Default state and input constraints
            ubx = inf(nx, 1);       
            lbx = -inf(nx, 1);
            ubu = inf(nu, 1);
            lbu = -inf(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            M = zeros(nx, nu);
            M(10:12,1:3) = eye(3); %x y z 
            M(6, 4) = 1; %gamma
            
            %        [wx,wy,wz, alpha,beta,gamma, vx,vy,vz, x,y,z ]
            Q = diag([2 2 1      1   1  10        1 1  2  10 10 10]);
            %         d1     d2     pavg  pdiff
            R = diag([0.001 0.001 0.001 0.001]);

            cost = 0;
            

            % Equality constraints (Casadi SX), each entry == 0
            %eq_constr = [ ; ];
            eq_constr = X_sym(:,1) - x0_sym;
            % Inequality constraints (Casadi SX), each entry <= 0
            ineq_constr = [ ; ];
            
            h = rocket.Ts;

            for i = 1:N-1
                k1 = rocket.f(X_sym(:,i),        U_sym(:,i));
                k2 = rocket.f(X_sym(:,i)+h/2*k1, U_sym(:,i));
                k3 = rocket.f(X_sym(:,i)+h/2*k2, U_sym(:,i));
                k4 = rocket.f(X_sym(:,i)+h*k3,   U_sym(:,i));

                x_next = X_sym(:,i) + h/6*(k1+2*k2+2*k3+k4);

                con = X_sym (:,i+1) - x_next;
                eq_constr = [eq_constr,con];
                cost = cost + (X_sym(:, i) - M * ref_sym)'*Q*(X_sym(:, i) - M * ref_sym) + U_sym(:, i)'*R*U_sym(:, i);
            end
            
            
            total = size(eq_constr,1) * size(eq_constr,2);
            eq_constr = reshape(eq_constr, [total,1]);

            
            % For box constraints on state and input, overwrite entries of
            % lbx, ubx, lbu, ubu defined above
            % Input constraints
            % delta < +-0.26 rad (15°)
            % Pavg = [0.2,0.8] -> Pdiff = [-0.2,0.2]
            
            % State constraints
            % |beta| <= 75°

            %beta
            ubx(5,1) = deg2rad(75);         
            lbx(5,1) = -deg2rad(75);  
            
            %delta 1,2
            ubu(1:2,1) = deg2rad(15);
            lbu(1:2,1) = -deg2rad(15);
            
            %Pavg
            ubu(3) = 80;
            lbu(3) = 20;
            
            %Pdiff
            ubu(4) = 20;
            lbu(4) = -20;

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % ---- Assemble NLP ------
            nlp_x = [X_sym(:); U_sym(:)];
            nlp_p = [x0_sym; ref_sym];
            nlp_f = cost;
            nlp_g = [eq_constr; ineq_constr];
            
            nlp = struct('x', nlp_x, 'p', nlp_p, 'f', nlp_f, 'g', nlp_g);
            
            % ---- Setup solver ------
            opts = struct('ipopt', struct('print_level', 1), 'print_time', true);
            obj.solver = nlpsol('solver', 'ipopt', nlp, opts);
            
            % ---- Assemble NLP bounds ----
            obj.nlp_x0  = zeros(size(nlp_x));
            
            obj.nlp_ubx = [repmat(ubx, N, 1); repmat(ubu, (N-1), 1)];
            obj.nlp_lbx = [repmat(lbx, N, 1); repmat(lbu, (N-1), 1)];
            
            obj.nlp_ubg = [zeros(size(eq_constr)); zeros(size(ineq_constr))];
            obj.nlp_lbg = [zeros(size(eq_constr)); -inf(size(ineq_constr))];
            
            obj.nlp_p = [zeros(size(x0_sym)); zeros(size(ref_sym))];
            
            obj.nlp_lam_x0 = [];
            obj.nlp_lam_g0 = [];
            
            obj.nx = nx;
            obj.nu = nu;
            obj.N = N;
            obj.T_opt = linspace(0, N * rocket.Ts, N);
            
            obj.idx.X = [1, obj.N * obj.nx];
            obj.idx.U = obj.idx.X(2) + [1, (obj.N-1) * obj.nu];
            obj.idx.u0 = obj.idx.U(1) + [0, obj.nu-1];
            
            % Members for delay compensation
            obj.rocket = rocket;
            obj.expected_delay = expected_delay;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            u_init = [0,0, 61, 0]; % stay in playce init
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj.mem_u = repmat(u_init, 1, expected_delay);
        end
        
        function [u, T_opt, X_opt, U_opt] = get_u(obj, x0 , ref)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            delay = obj.expected_delay;
            u = obj.mem_u;
            % Delay compensation: Predict x0 delay timesteps later.
            % Simulate x_ for 'delay' timesteps
            x_ = x0;
            h = obj.rocket.Ts;
            for i = 1:delay
                u_segment = u((i-1)*4 + 1 : i*4);
                x_ = x_ + h*obj.rocket.f(x_,        u_segment);
            end
       
            x0 = x_;
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute solution from x0
            obj.solve(x0, ref);
            
            % Evaluate u0
            nlp_x = obj.sol.x;
            id = obj.idx.u0;
            u = full( nlp_x(id(1):id(2)) );      
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % Delay compensation: Save current u

            %FiFo memory last element is the most recent input, first
            %element will be applied next time step
            u = reshape(u,1,4);
            if obj.expected_delay > 0
               if delay > 1
                    % Shift all elements to the left
                    obj.mem_u = [obj.mem_u(5:end), u];
                else
                    % If the array has only one element, replace it with the new input
                    obj.mem_u = u;
                end
            end
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            if nargout > 1, T_opt = obj.get_T_opt(); end
            if nargout > 2, X_opt = obj.get_X_opt(); end
            if nargout > 3, U_opt = obj.get_U_opt(); end
            return
            
            % Additional evaluation
            % Complete trajectory
            % % X_opt = full(reshape(nlp_x(idx_X(1):idx_X(2)), obj.nx, obj.N));
            % % U_opt = full(reshape(nlp_x(idx_U(1):idx_U(2)), obj.nu, obj.N - 1));
            % %
            % % cost_opt = full(sol.f);
            % % constr_opt = full(sol.g);
            % %
            % % stats = obj.solver.stats;
        end
        
        function solve(obj, x0, ref)
            
            % ---- Set the initial state and reference ----
            obj.nlp_p = [x0; ref];     % Initial condition
            obj.nlp_x0(1:obj.nx) = x0; % Initial guess consistent
            
            % ---- Solve the optimization problem ----
            args = {'x0', obj.nlp_x0, ...
                'lbg', obj.nlp_lbg, ...
                'ubg', obj.nlp_ubg, ...
                'lbx', obj.nlp_lbx, ...
                'ubx', obj.nlp_ubx, ...
                'p', obj.nlp_p, ...
                %                 'lam_x0', obj.nlp_lam_x0, ...
                %                 'lam_g0', obj.nlp_lam_g0
                };
            
            obj.sol = obj.solver(args{:});
            if obj.solver.stats.success ~= true
                solve_status_str = obj.solver.stats.return_status;
                fprintf([' [' class(obj) ': ' solve_status_str '] ']);
                obj.sol.x(obj.idx.u0) = nan;
            end
            
            % Use the current solution to speed up the next optimization
            obj.nlp_x0 = obj.sol.x;
            obj.nlp_lam_x0 = obj.sol.lam_x;
            obj.nlp_lam_g0 = obj.sol.lam_g;
        end
        function T_opt = get_T_opt(obj)
            T_opt = obj.T_opt;
        end
        function X_opt = get_X_opt(obj)
            nlp_x = obj.sol.x;
            id = obj.idx.X;
            X_opt = full(reshape(nlp_x(id(1):id(2)), obj.nx, obj.N));
        end
        function U_opt = get_U_opt(obj)
            nlp_x = obj.sol.x;
            id = obj.idx.U;
            U_opt = full(reshape(nlp_x(id(1):id(2)), obj.nu, obj.N - 1));
        end
    end
end

