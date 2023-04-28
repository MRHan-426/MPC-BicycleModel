% Creates a new simulation of car going around track with new random set of 35 obstacles
% Simulation is done via forwrad integrating (using Euler Integration) control inputs 
% acquired from the MPC controller every 0.5 seconds. 
%Inputs - None
%Outputs - Y_sim - 6 system states [longitudinal coordinate, longitudianal velocity, lateral coordinate, lateral velocity, heading, rotational velocity]
%          U_sim - 2 control inputs [wheel angle (steering), tire Force (Throttle)].
%          t_total - total simulated time to complete track
%          t_update - time to update per function call 
%          xobs - coordinates of random obstacles  
clc
clear
close all
% forward Integrates Simulation
for i = 1:1
    [Y_sim,U_sim,t_total,t_update,Xobs] = forwardIntegrate(i);

%     getTrajectoryInfo(Y_sim,U_sim,Xobs,t_update)
    %save('trajectories/sim_result2.mat');
end
