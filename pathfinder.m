function [ desired_heading, map ] = pathfinder(robot_pos, target, map, slam_map)
% Finds the best path to the target based on the current SLAM map
%   Creates a grid map and calculates the distance from each map grid to
%   the target. Distance is used as a cost function to determine shortest
%   route. A* algorithm is used to find best route.
    
    % NOTE: Can't load variables from workspace into a function which uses
    %       nested functions. Avoid nested functions
    
    %pathfinder Macros
    ENVIRONMENT_SIZE    = 120;
    TILE_SIZE           = 1;
    MAP_SIZE            = ENVIRONMENT_SIZE / TILE_SIZE; % This should be an integer
    VISUALIZE_MAP       = 1;
    VISUALIZE_PATH      = 1;
    VIS_MAP_ALPHA       = 0.5; % Transparency percentage
    SLAM_THRESHOLD      = 127; % Threshold for determining if obstacle is present
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                       MAP BUILDING                        %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Initialize map (aligned with global environment origin)
    map_origin(1) = 0;
    map_origin(2) = 0; 
    redraw_map = size( find(map), 1 );  % If map is empty, need to redraw
    redraw_map = ~redraw_map;
    
    % Determine map indicies of robot position
    robot_pos_ind(1) = robot_pos(1) - map_origin(1);
    robot_pos_ind(2) = robot_pos(2) - map_origin(2);
    robot_pos_ind = ceil( robot_pos_ind ./ TILE_SIZE );
    
    % Determine if target position has changed since last iteration
%     [~, temp_index] = min( map(:) );
%     last_target(2) = ceil( temp_index/MAP_SIZE );
%     last_target(1) = mod( temp_index, MAP_SIZE );
%     if (last_target(1) == 0)
%         last_target(1) = MAP_SIZE;
%     end
%     target_ind(1) = target(1) - map_origin(1);
%     target_ind(2) = target(2) - map_origin(2);
%     target_ind = ceil( target_ind ./ TILE_SIZE );
%     if (target_ind ~= last_target)
%         redraw_map = 1;
%     end
    
    % If target has moved, need to re-weight map tiles
%      redraw_map = 1; % TEMP always redraw
    if (redraw_map)
        % Add gradient weighting to map which increases with distance from target
        dx2 = repmat( (map_origin(1)+TILE_SIZE/2) , 1, MAP_SIZE) + (TILE_SIZE * (0:MAP_SIZE-1));
        dy2 = repmat( (map_origin(2)+TILE_SIZE/2) , 1, MAP_SIZE) + (TILE_SIZE * (0:MAP_SIZE-1));
        dx2 = dx2 - repmat( target(1), 1, MAP_SIZE );
        dy2 = dy2 - repmat( target(2), 1, MAP_SIZE );
        dx2 = dx2.^2;
        dy2 = dy2.^2;
        dist_from_target = repmat( dx2, MAP_SIZE, 1)' + repmat( dy2, MAP_SIZE, 1);
        dist_from_target = dist_from_target.^(1/2);
        map = dist_from_target ./  TILE_SIZE;
    end
 
    % Give infinite weighting to map tiles with obstacles (walls)
    slam_map = slam_map < SLAM_THRESHOLD;
    slam_map = slam_map * inf;
    temp = find(isnan( slam_map ));
    slam_map(temp) = 0;
    slam_map = flip( slam_map );
    map = map + slam_map';
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                       PATHFINDING                         %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Find best path using a variant of A* algorithm
    % Website used for reference:
    % http://www.redblobgames.com/pathfinding/a-star/introduction.html
    
    % If target is in range, set as goal
    % Otherwise, set goal to lowest value point in map
    target_ind(1) = target(1) - map_origin(1);
    target_ind(2) = target(2) - map_origin(2);
    target_ind = ceil( target_ind ./ TILE_SIZE );
    if (target_ind(1) > 0) && (target_ind(1) <= MAP_SIZE) && (target_ind(2) > 0) && (target_ind(2) <= MAP_SIZE)
        goal_index = target_ind;
    else
        [~, temp_index] = min( map(:) );
        goal_index(2) = ceil( temp_index/MAP_SIZE );
        goal_index(1) = mod( temp_index, MAP_SIZE );
        if (goal_index(1) == 0)
            goal_index(1) = MAP_SIZE;
        end
    end
    
    num_added = 0;
    found_goal = 0;
    came_from = zeros(2, MAP_SIZE, MAP_SIZE);
    cost_so_far = zeros(MAP_SIZE, MAP_SIZE);
    map_frontier = zeros(3, 1000);
    % map_frontier stores x and y coords and a priority value of all points
    % in frontier. Frontier made large to prevent growth every loop iteration.
    % MATLAB does not have built in data queue structure, or any
    % particularly efficient way (known to me) to maintain a queue. Instead, 
    % maintain a count of empty cells, and append new entries to 
    % map_frontier when necessary
    map_frontier(3,:) = inf; % Minimum priority
    map_frontier(1,1) = robot_pos_ind(1);
    map_frontier(2,1) = robot_pos_ind(2);
    map_frontier(3,1) = 0; % Maximum priority
    size_frontier = 1;
    num_empty_cells = 0;
    while (size_frontier > 0) && (found_goal == 0) % Run till frontier is empty or goal is found
        [~, cur_frontier_index] = min( map_frontier(3, :) ); % Find highest priority
        cur_x_index = map_frontier(1, cur_frontier_index);
        cur_y_index = map_frontier(2, cur_frontier_index);
        
%         %%%%%%%%%%%%%%%%%%%%%%%%%%% DEBUG 
%         Visulaization of expanding frontier
%
%         temp_map = map;
%         temp_ind = find( map_frontier(3,:) < inf );
%         [~, sz] = size( temp_ind );
%         for i = 1:sz
%             temp_map( map_frontier(1,temp_ind(i)), map_frontier(2,temp_ind(i)) ) = 0;
%         end
%         im = imagesc( [0, MAP_SIZE], [0, MAP_SIZE], temp_map' );
%         pause(0.01);
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
        
        size_frontier = size_frontier - 1;
        for i = -1:1:1
            for j = -1:1:1
                % Nested for loops allow movement in 8 directions (Forward,
                % Back, Left, Right, and Diagonals)
                if (i == 0) && (j == 0) % Don't check current pos
                    continue;
                end
                if (i ~= 0) && (j ~= 0)
                    next_cost = 1.414; % Correct distance cost for diagonal steps
                else
                    next_cost = 1;
                end
                next_x_ind = cur_x_index + i;
                next_y_ind = cur_y_index + j;
                if (next_x_ind < 1) || (next_x_ind > MAP_SIZE) || (next_y_ind < 1) || (next_y_ind > MAP_SIZE)
                    continue; % Skip if out of map range
                end
                if ~(map( next_x_ind, next_y_ind ) < inf) 
                    continue; % Skip if obstacle in the way
                end
                new_cost = cost_so_far(cur_x_index, cur_y_index) + next_cost;
                % No 'came_from' data on tile means tile unvisited
                old_tile = came_from(1, next_x_ind, next_y_ind);
                if (~old_tile) || (new_cost < cost_so_far( next_x_ind, next_y_ind ))
                    % Tile not visited before, or found shorter route
                    if (num_added == 0)
                        temp_frontier_index = cur_frontier_index; % Replace current entry
                    else
                        temp_frontier_index = size_frontier + 1 + num_empty_cells; % append to end
                    end
                    cost_so_far( next_x_ind, next_y_ind ) = new_cost;
                    map_frontier(1, temp_frontier_index) = next_x_ind;
                    map_frontier(2, temp_frontier_index) = next_y_ind;
                    map_frontier(3, temp_frontier_index) = new_cost + map(next_x_ind, next_y_ind);
                    came_from(1, next_x_ind, next_y_ind) = cur_x_index;
                    came_from(2, next_x_ind, next_y_ind) = cur_y_index;
                    num_added = num_added + 1;
                    size_frontier = size_frontier + 1;
                end
                if (next_x_ind == goal_index(1)) && (next_y_ind == goal_index(2))
                    found_goal = 1;
                end
            end
        end
        if (num_added == 0) % No replacement found for current frontier. Incremement empty cell count
            num_empty_cells = num_empty_cells + 1;
            map_frontier(:, cur_frontier_index) = 0;
            map_frontier(3, cur_frontier_index) = inf;
        end
        num_added = 0;
    end
    
    if( found_goal == 0 )
        % If goal not found, find best obtainable point
        allowed_tile_mask(:,:) = came_from(1,:,:) > 0;
        allowed_map = map ./ allowed_tile_mask; % MATLAB allows divide by 0. Returns 'inf'
        [~, temp_index] = min( allowed_map(:) );
        goal_index(2) = ceil( temp_index/MAP_SIZE );
        goal_index(1) = mod( temp_index, MAP_SIZE );
        if (goal_index(1) == 0)
            goal_index(1) = MAP_SIZE;
        end
    end
    
    % Create path by backtracking steps from goal to start
    found_path = 0;
    step = 1;
    path(:, step) = goal_index(:);
    while( ~found_path );
        step = step + 1;
        if (path(1, step-1) ~= 0) && (path(2, step-1) ~= 0)
            path(:, step) = came_from(:, path(1, step-1), path(2, step-1));
        else
            path = zeros(2,2);
            path(1,:) = robot_pos_ind(1);
            path(2,:) = robot_pos_ind(2);
            break;
        end
        if (path(1, step) == robot_pos_ind(1)) && (path(2, step) == robot_pos_ind(2))
            path = fliplr( path ); % Reverse order so path(1) is current pos
            found_path = 1;
        end
    end
    
    % Determine heading to next point
    % Returned relative to global frame (NOT to robot current heading)
    % Returned in range (0, 360) in degrees
    temp = (path(:,2) * TILE_SIZE);
    temp = temp - map_origin';
    temp = temp - (TILE_SIZE/2);
    dx = temp(1) - robot_pos(1);
    dy = temp(2) - robot_pos(2);
    if (dx == 0) 
        dx = 0.0001;
    end
    desired_heading = atan2d(dy,dx);
    if (desired_heading < 0)
        desired_heading = desired_heading + 360;
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                       VISUALIZATION                       %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(VISUALIZE_MAP)
        vis_map = map;
        x_rng = [ map_origin(1) + TILE_SIZE/2, map_origin(1) + TILE_SIZE/2 + (TILE_SIZE * (MAP_SIZE-1))];
        y_rng = [ map_origin(2) + TILE_SIZE/2, map_origin(2) + TILE_SIZE/2 + (TILE_SIZE * (MAP_SIZE-1))];
        if(VISUALIZE_PATH)
            [~, sz] = size( path );
            for i = 1:sz
                vis_map( path(1,i), path(2,i) ) = 0;
            end
        end
        im = imagesc(x_rng, y_rng, vis_map');
        im.AlphaData = VIS_MAP_ALPHA;
    end
end

