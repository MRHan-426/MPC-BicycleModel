function [Y,U,t_total,t_update,Xobs] = forwardIntegrate()
% [Y,U,t_total,t_update] = forwardIntegrate
% 
% This script returns the vehicle trajectory with control input being
% generated via the control input generation function:
%         ROB535_ControlProject_part2_Team<your team number>
% Obstacles are randomly generated along the test track. Notice that the
% vehicle can only sense (observe) the obstacles within 150m, therefore
% the control input generation function is called repeatedly. In this
% script, we assume the control input generation function is called every
% 'dt' second (see line 32). 
% 
% OUTPUTS:
%   Y         an N-by-6 vector where each column is the trajectory of the
%             state of the vehicle 
%   
%   U         an N-by-2 vector of inputs, where the first column is the
%             steering input in radians, and the second column is the
%             longitudinal force in Newtons
%   
%   t_total   a scalar that records the total computational time  
%   
%   t_update  a M-by-1 vector of time that records the time consumption
%             when the control input generation function is called
%             
% Written by: Jinsun Liu
% Created: 31 Oct 2021


    load('TestTrack.mat') % load test track

    dt = 0.5; 
    TOTAL_TIME = 20*60; % second
    
    % initialization
    t_total = 0;
    t_update = zeros(TOTAL_TIME/dt+1,1);
    Y = zeros(TOTAL_TIME/0.01+1,6);
    U = zeros(TOTAL_TIME/0.01,2);
    Y(1,:) = [287,5,-176,0,2,0];

    % generate obstacles along the track
    %Xobs = generateRandomObstacles(9 + randi(16),TestTrack);
    Xobs = generateRandomObstacles(35,TestTrack);
    %load('difficultobstacles2.mat');

    iteration = 1; % a counter that counts how many times the control input 
                   % generation function is called.

    TIMER = tic; % start the timer
    
    % you only have TOTAL_TIME seconds to sense the obstacles, update
    % control inputs, and simulate forward vehicle dynamcis.
    while t_total < TOTAL_TIME 
        curr_pos = Y( (iteration-1)*dt/0.01+1 , [1,3] ); % record current vehicle position
        Xobs_seen = senseObstacles(curr_pos, Xobs); % sense the obstacles within 150m
        curr_state = Y( (iteration-1)*dt/0.01+1 , : ); % record current vehicle states

        
        
        % compute control inputs, and record the time consumption
        t_temp = toc(TIMER);
        %%%%%%%%%%%%%%%% THIS IS WHERE YOUR FUNCTION IS CALLED (replace in your team number). %%%%%%%%%%%%%%%%%%%%%%%%%%%
        [Utemp, FLAG_terminate] = ROB535_ControlProject_part2_Team2(TestTrack,Xobs_seen,curr_state); %%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         FLAG_terminate = randi(2)-1                                 % GSIs: This line is just for us to debug. Feel free to play with it if you want
%         Utemp = rand(dt/0.01+1 + FLAG_terminate* (randi(10)-5),2);  % GSIs: This line is just for us to debug. Feel free to play with it if you want        
        t_update(iteration) = toc(TIMER)-t_temp;
        
        
        
        % Utemp must contain control inputs for at least dt second,
        % otherwise stop the whole computation.
        if size(Utemp,1)<dt/0.01+1 && FLAG_terminate == 0
            fprintf('When FLAG_terminate = 0, Utemp cannot contain control inputs for less than %f second. \n',dt);
            fprintf('Solving process is terminated.\n');
            t_total = toc(TIMER);
            break
        end

        
        
        if FLAG_terminate == 0
            % if FLAG_terminate == 0, simulate forward vehicle dynamics for
            % dt second.
            U( (iteration-1)*dt/0.01+1:iteration*dt/0.01 , : ) = Utemp(1:dt/0.01,:);
            Ytemp = forwardIntegrateControlInput( Utemp(1:dt/0.01+1,:) , curr_state );
            Y( (iteration-1)*dt/0.01+2:iteration*dt/0.01+1 , : ) = Ytemp(2:end,:);
        
            % update the counter
            iteration = iteration + 1;
        else
            % if FLAG_terminate == 1, simulate forward vehicle dynamics for
            % no more than dt second, and stop the solving process.
            simulate_length = min(dt/0.01+1, size(Utemp,1));
            U((iteration-1)*dt/0.01+1:(iteration-1)*dt/0.01+simulate_length-1, :) = Utemp(1:simulate_length-1,:);
            Ytemp = forwardIntegrateControlInput( Utemp(1:simulate_length,:) , curr_state );
            Y((iteration-1)*dt/0.01+2:(iteration-1)*dt/0.01+simulate_length, : ) = Ytemp(2:end,:);
        end
        
        
        % update t_total
        t_total = toc(TIMER);

        % stop the computation if FLAG_terminate == 1
        if FLAG_terminate == 1
            break
        end
    end

    % if reach the finish line before TOTAL_TIME, ignore any parts of the
    % trajectory after crossing the finish line.
    idx_temp = find(sum(abs(Y),2)==0,1);
    Y(idx_temp:end,:) = [];
    U(idx_temp:end,:) = [];
    t_update(iteration:end) = [];

end