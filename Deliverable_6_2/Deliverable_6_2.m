addpath(fullfile('..', 'src'));

%close all
%clear all
%clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/40; % Note that we choose a larger Ts here to speed up the simulation
H = 2;
Tf = 3;
x0 = zeros(12,1);

%% Delay of 5
rocket = Rocket(Ts);
rocket.anim_rate = 5;
delay = 5;
rocket.delay = delay; % 0 if not specified
rocket.mass = 1.75;
nmpc = NmpcControl(rocket, H,delay);
ref = [0.5, 0, 1, deg2rad(65)]';
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = "Delay5_H2";

%% Delay of 10
rocket = Rocket(Ts);
rocket.anim_rate = 5;
rocket.mass = 1.75;
delay = 10;
rocket.delay = delay; % 0 if not specified
nmpc = NmpcControl(rocket, H,delay);
ref = [0.5, 0, 1, deg2rad(65)]';
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = "Delay10_H2";

%% Delay of 15
rocket = Rocket(Ts);
rocket.anim_rate = 5;
delay = 15;
rocket.delay = delay; % 0 if not specified
nmpc = NmpcControl(rocket, H,delay);
ref = [0.5, 0, 1, deg2rad(65)]';
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = "Delay15_H2";

%% Delay of 5 no compensation
rocket = Rocket(Ts);
rocket.anim_rate = 5;
rocket.mass = 1.75;
delay = 5;
rocket.delay = delay; % 0 if not specified
nmpc = NmpcControl(rocket, H);
ref = [0.5, 0, 1, deg2rad(65)]';
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = "Delay5_no_compensation_H2";

%% Delay of 5 with compensation of 3
rocket = Rocket(Ts);
rocket.anim_rate = 5;
rocket.mass = 1.75;
delay = 5;
rocket.delay = delay; % 0 if not specified
nmpc = NmpcControl(rocket, H,3);
ref = [0.5, 0, 1, deg2rad(65)]';
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = "Delay5_comp3_H2";


%% NMPC on TVC with delay correction (delay of 5)
rocket = Rocket(Ts);
rocket.anim_rate = 5;
rocket.mass = 1.75;
delay = 5;
rocket.delay = delay; % 0 if not specified
nmpc = NmpcControl(rocket, H,delay);
roll_max = deg2rad(50);
ref = @(t_, x_) ref_TVC(t_, roll_max);
Tf = 30;
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
ph = rocket.plotvis(T, X, U, Ref);
ph.fig.Name = "TVC_Delay5";


