function [A, b] = find_constraints(car, track, obstacles)

% Find the closest point along the lines to the car
[left_i, left_j] = nearest_points(track.bl, car);
[right_i, right_j] = nearest_points(track.br, car);
[center_i, center_j] = nearest_points(track.cline, car);

% Create constraints from the sides of the road
[A_left, b_left] = constraint(track.bl(:,left_j), track.bl(:,left_i));
[A_right, b_right] = constraint(track.br(:,right_i), track.br(:,right_j));
A = [A_left; A_right];
b = [b_left; b_right];

% Order the obstacles along the track
n_obs = size(obstacles, 2);
obstacle_order = zeros(1, n_obs);
for o = 1:n_obs
    obstacle_order(o) = nearest_points(track.cline, obstacles{o}(1, :)');
end

% Find the next obstacle in front of the car
inds = 1:n_obs;
in_front = inds(obstacle_order > center_i - 2);
if any(in_front)
    [~, i] = mink(obstacle_order(in_front), 2);
    next_obs = in_front(i);
    
    for o = next_obs
        % Only if we are within a certain distance of the obstacle
        if sum((obstacles{o}(1,:)' - car).^2)^0.5 < 30
            % Choose a side to pass the obstacle on
            [left_clearance, obs_left] = clearance(track.bl, obstacles{o});
            [right_clearance, obs_right] = clearance(track.br, obstacles{o});

            if left_clearance > right_clearance
                for ob_i = 1:size(obs_left, 2)
                    [A_obs, b_obs] = constraint(car, obs_left(:,ob_i));
                    A = [A; A_obs];
                    b = [b; b_obs];
                end
            else
                for ob_i = 1:size(obs_right, 2)
                    [A_obs, b_obs] = constraint(obs_right(:,ob_i), car);
                    A = [A; A_obs];
                    b = [b; b_obs];
                end
            end
        end
    end
end

end