function Xobs = generateRandomObstacles(Nobs,TestTrack)
% Xobs = generateRandomObstacles(Nobs)
%
% Given a number of obstacles Nobs and a track, place obstacles at random
% orientations with one corner of each obstacle pinned to the center line
% of the track
%
% INPUTS:
%   Nobs        an integer defining the number of obstacles to output
%
%   TestTrack   a TestTrack object for which TestTrack.cline is the
%               centerline
%
% OUTPUTS:
%   Xobs        a 1-by-Nobs cell array, where each cell contains a single
%               rectangular obstacle defined as a 4-by-2 matrix where the
%               first column is the x coordinates and the second column is
%               the y coordinates
%
% Written by: Shreyas Kousik
% Created: 12 Nov 2017
% Modified: 13 Nov 2017

    if nargin < 2
        loaded_file = load('TestTrack.mat') ;
        TestTrack = loaded_file.TestTrack ;
    end

    if Nobs > 100
    warning(['Number of obstacles is greater than 100! This is likely to ',...
             'make the resulting course infeasible.'])
    end

    % get the center line and boundaries, but exclude the parts of the
    % track that are close to the beginning and end
    c = TestTrack.cline(:,4:end-4) ;
    h = TestTrack.theta(:,4:end-4) ;

    % get the cumulative and total distance along the centerline
    dists_along_cline = cumsum([0, sqrt(diff(c(1,:)).^2 + diff(c(2,:)).^2)]) ;
    total_dist_along_cline = dists_along_cline(end) ;

    % create a vector of random distances between obstacles
    min_dist_btwn_obs = 10 ; % meters
    max_dist_btwn_obs = total_dist_along_cline / Nobs ; % also meters
    dists_btwn_obs = (max_dist_btwn_obs-min_dist_btwn_obs).*rand(1,Nobs) + min_dist_btwn_obs ;
    obs_start_dists = cumsum(dists_btwn_obs) ;

    % scale up the distances between the obstacles to be along the whole length
    % of the track (this means the min and max distanes between obstacles will
    % increase, but this is a hack anyways, so hah)
    end_pct = 0.1*rand(1) + 0.85 ;
    obs_start_dists = obs_start_dists.*(end_pct*total_dist_along_cline./obs_start_dists(end)) ;
    
    % NOTE: The lines above are meant to encourage the track to be
    % feasible, but there is never a 100% guarantee off feasibility

    % get the start point and orientation of each obstacle
    obs_start_x = interp1(dists_along_cline,c(1,:),obs_start_dists) ;
    obs_start_y = interp1(dists_along_cline,c(2,:),obs_start_dists) ;
    obs_heading = interp1(dists_along_cline,h,obs_start_dists) ;

    % generate a random size and random side of the road for each obstacle;
    % the parameters below work well for the COTA track segment that we are
    % using in ROB 599
    obs_min_length = 1 ;
    obs_max_length = 4 ;
    obs_min_width = 3 ; 
    obs_max_width = 7 ;
    obs_lengths = (obs_max_length - obs_min_length).*rand(1,Nobs) + obs_min_length ;
    obs_widths = (obs_max_width - obs_min_width).*rand(1,Nobs) + obs_min_width ;
    obs_sides = round(rand(1,Nobs)) ; % 0 is right, 1 is left

    % from each start point, create a CCW contour defining a box that is
    % pinned to the centerline at one corner
    Xobs = cell(1,Nobs) ;
    for idx = 1:Nobs
        % create box
        lidx = obs_lengths(idx) ;
        widx = obs_widths(idx) ;
        obs_box = [0, 0, lidx, lidx ;
                   0, -widx, -widx, 0] ;

        % if the box is on left side of track, shift it by +widx in its
        % local y-direction
        obs_box = obs_box + obs_sides(idx).*[zeros(1,4) ; widx.*ones(1,4)] ;

        % rotate box to track orientation
        hidx = obs_heading(idx) ;
        Ridx = [cos(hidx) -sin(hidx) ; sin(hidx) cos(hidx)] ;
        obs_box = Ridx*obs_box ;

        % shift box to start point
        xidx = obs_start_x(idx) ;
        yidx = obs_start_y(idx) ;
        obs_box = obs_box + repmat([xidx;yidx],1,4) ;

        % fill in Xobs
        Xobs{idx} = obs_box' ;
    end
end