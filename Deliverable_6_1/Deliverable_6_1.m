addpath(fullfile('..', 'src'));

close all;
clear all;
clc;

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Note that we choose a larger Ts here to speed up the simulation
rocket = Rocket(Ts);
H = 2;
nmpc = NmpcControl(rocket, H);
Tf = 30;
x0 = zeros(12,1);
rocket.anim_rate = 3;

%% Open loop
ref4 = [2 2 2 deg2rad(40)]';
[u, T_opt, X_opt, U_opt] = nmpc.get_u(x0, ref4);
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref4);
ph.fig.Name = 'Open loop NMPC';

%% NMPC with max roll of 15°

ref = @(t_, x_) ref_TVC(t_);
[T, X_1, U_1, Ref_1] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
 
ph = rocket.plotvis(T, X_1, U_1, Ref_1);
ph.fig.Name = 'NMPC_15';

%% NMPC with max roll of 50°
roll_max = deg2rad(50);
ref = @(t_, x_) ref_TVC(t_, roll_max);

[T, X_2, U_2, Ref_2] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);

ph = rocket.plotvis(T, X_2, U_2, Ref_2);
ph.fig.Name = 'NMPC_50';

