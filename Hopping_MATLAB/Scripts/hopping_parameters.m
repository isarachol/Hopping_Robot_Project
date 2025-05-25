%------------------------------------------------------
% Using Mastric Units
%------------------------------------------------------
%% Robot dimension
global lower_l upper_l;
foot_dim = [0.10 0.07 0.02];
lower_r = 0.015;
lower_l = 0.15;
upper_r = lower_r;
upper_l = lower_l;
head_dim = [0.10 0.07 0.04];

global ankle_pos_1 head_pos_7; % picked at random
ankle_pos_1 = [-(foot_dim(1)-lower_r*2)/3 0 foot_dim(3)];
head_pos_7  = [(head_dim(1)-upper_r*2)/3  0 head_dim(3)];

%% Transformation
ankle_pos = [-(foot_dim(1)-lower_r)/3 0 0];

%% color
red = [0.6 0.1 0.1];
green = [0.1 0.6 0.1];
blue = [0.2 0.3 0.6];
yellow = [0.7 0.5 0];

%% material props
% rho
origin = [0 0 0 1]';

% mass (kg)
head_m = 0.5;
head_cm_8 = origin;
hip_m = 0.2;
hip_cm_6 = origin;
upper_m = 0.05;
upper_cm_5 = [0 0 upper_l/2 1]';
knee_m = 0.1;
knee_cm_4 = origin;
lower_m = 0.05;
lower_cm_3 = [0 0 lower_l/2 1]';
ankle_m = 0.2;
ankle_cm_2 = origin;
foot_m = 0.5;
foot_cm_1 = origin;

% Convert them to array
global m cm cm_frame;
m           = [foot_m ankle_m lower_m knee_m upper_m hip_m head_m];
cm          = [foot_cm_1 ankle_cm_2 lower_cm_3 knee_cm_4 upper_cm_5 hip_cm_6 head_cm_8];
cm_frame    = [1 2 3 4 5 6 8];

%% motor param

%% controller param