%% Startup hopping_v1 project
clear;
curr_proj = simulinkproject;
cd(curr_proj.RootFolder)

% Change to root folder
cd(curr_proj.RootFolder)

hopping_parameters;
hopping_v1;