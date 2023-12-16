addpath(fullfile('..', 'src'));

close all
clear all
clc

%% TODO: This file should produce all the plots for the deliverable

% Parameters
CLOSELOOP = true;
MPC_X=1;
MPC_Y=1;
MPC_Z=1;
MPC_R=1;


% General initializations
Ts = 1/20; % Sample time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

H = 3;
Tf = 10;

%% MPC Controller X

if MPC_X
    x0 = [0; 0; 0; 3];
    mpc_x = MpcControl_x(sys_x, Ts, H);
    
    if CLOSELOOP
        [T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, 0);
        ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);
    else
        [u, T_opt, X_opt, U_opt] = mpc_x.get_u(x0);
        U_opt(:,end+1) = NaN;
        % Account for linearization point
        % X_opt = ...
        % U_opt = ...
        ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us); % Plot as usual
    end
end

%% MPC Controller Y

if MPC_Y
    x0 = [0; 0; 0; 3];
    mpc_y = MpcControl_y(sys_y, Ts, H);
    
    if CLOSELOOP
        [T, X_sub, U_sub] = rocket.simulate_f(sys_y, x0, Tf, @mpc_y.get_u, 0);
        ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);
    else
        [u, T_opt, X_opt, U_opt] = mpc_y.get_u(x0);
        U_opt(:,end+1) = NaN;
        % Account for linearization point
        % X_opt = ...
        % U_opt = ...
        ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us); % Plot as usual
    end
end

%% MPC Controller Z

if MPC_Z
    x0 = [0; 3];
    mpc_z = MpcControl_z(sys_z, Ts, H);
    
    if CLOSELOOP
        [T, X_sub, U_sub] = rocket.simulate_f(sys_z, x0, Tf, @mpc_z.get_u, 0);
        ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us);
    else
        [u, T_opt, X_opt, U_opt] = mpc_z.get_u(x0);
        U_opt(:,end+1) = NaN;
        % Account for linearization point
        % X_opt = ...
        % U_opt = ...
        ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us); % Plot as usual
    end
end

%% MPC Controller ROLL

if MPC_R
    x0 = [0; deg2rad(40)];
    mpc_roll = MpcControl_roll(sys_roll, Ts, H);
    
    if CLOSELOOP
        [T, X_sub, U_sub] = rocket.simulate_f(sys_roll, x0, Tf, @mpc_roll.get_u, 0);
        ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);
    else
        [u, T_opt, X_opt, U_opt] = mpc_roll.get_u(x0);
        U_opt(:,end+1) = NaN;
        % Account for linearization point
        % X_opt = ...
        % U_opt = ...
        ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us); % Plot as usual
    end
end
