% Clean workspace
clc
clf
clear
close all

% Add Breezy SLAM matlab folder to matlab path and recompile
BSLAM_path = [pwd '\BreezySLAM-master\matlab'];
path(path,BSLAM_path);
mex ./BreezySLAM-master/matlab/mex_breezyslam.c ./BreezySLAM-master/c/coreslam.c ./BreezySLAM-master/c/coreslam_sisd.c ./BreezySLAM-master/c/random.c ./BreezySLAM-master/c/ziggurat.c
% mex -g ./BreezySLAM-master/matlab/mex_breezyslam.c ./BreezySLAM-master/c/coreslam.c ./BreezySLAM-master/c/coreslam_sisd.c ./BreezySLAM-master/c/random.c ./BreezySLAM-master/c/ziggurat.c

% Global Macros
ENVIRONMENT_SIZE    = 120;
SAVE_FILE           = 'environment.mat';
VIS_LIDAR_RAYS      = 0;
GOAL_TOL            = 3;

% generateEnvironment Macros
MAX_WALL_LEN        = 80;
MIN_WALL_LEN        = 10;
NUM_WALLS           = 10;
NUM_WALL_POINTS     = 2;
MIN_TARGET_SEP      = 120;
WALL_EDGE_PAD       = 5;

%pathfinder Macros
TILE_SIZE           = 2; % Recommend even number
MAP_SIZE            = ENVIRONMENT_SIZE / TILE_SIZE; % This should be an integer
VISUALIZE_MAP       = 1;
VISUALIZE_PATH      = 1;
VIS_MAP_ALPHA       = 0.5; % Transparency percentage

% getLidar Macros
NUM_LIDAR_LINES     = 100;
LIDAR_RANGE         = 40;
LIDAR_STD_DEV       = 0.0;
LIDAR_BIAS          = 0;
DETECTION_MARGIN    = 10;

% Breezy SLAM Macros
MAP_SIZE_PIXELS          = ENVIRONMENT_SIZE * 1;
MAP_SIZE_METERS          = ENVIRONMENT_SIZE / 10;
ROBOT_SIZE_PIXELS        = 2;

% Save macros to file so all functions can access. Overwrite old file
% Generate random environment and initialize map
save( SAVE_FILE );
generateEnvironment( SAVE_FILE );
load( SAVE_FILE );
map = zeros(MAP_SIZE, MAP_SIZE);

% Initialize motion controller
priorValues = [0;0;0;0;0];
roboX = robot_start(1);
roboY = robot_start(2);
last_true_pos = robot_start;
last_true_pos(3) = 0;
last_odo_pos = robot_start;
last_odo_pos(3)= 0;
steering_angle = 0;
odo_roboX = roboX;
odo_roboY = roboY;

% Define Laser
laser.scan_size = NUM_LIDAR_LINES;
laser.scan_rate_hz = 10;
laser.detection_angle_degrees = 360;
laser.distance_no_detection_mm = LIDAR_RANGE * 100;
laser.detection_margin = DETECTION_MARGIN;
laser.offset_mm = 1;

% Initialize SLAM
start_pos(1) = robot_start(1) * 100;
start_pos(2) = (ENVIRONMENT_SIZE - robot_start(2)) * 100;
start_pos(3) = 0;
slam = Deterministic_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS, start_pos);
velocities = [0, 0, 0];
theta_degrees = 0;
slam_theta = 0;

f1 = figure();
f2 = figure();
colormap default;
robot_size = ROBOT_SIZE_PIXELS;

% Wait for user to begin main loop
disp(' ');
disp('Press any key to begin trial...');
pause;

while true
    % Verify Robot is within environment bounds and has not yet reached the
    % target
    if (roboX > ENVIRONMENT_SIZE) || (roboY > ENVIRONMENT_SIZE)
        break;
    end
    if (abs(roboX - target_pos(1)) < GOAL_TOL) && (abs(roboY - target_pos(2)) < GOAL_TOL)
        disp('AT GOAL');
        break;
    end
    lidarRays = getLidar( roboX, roboY, steering_angle, wall_map );
    
    % Configure plot
    figure(f1);
    clf;
    
    % Display Environment
    xlim([0, ENVIRONMENT_SIZE]);
    ylim([0, ENVIRONMENT_SIZE]);
    hold on
    for j=1:NUM_WALLS
       plot([wall_map(j,1), wall_map(j,3)], [wall_map(j,2), wall_map(j,4)], 'linewidth', 2);
    end
    plot(target_pos(1), target_pos(2), '*', 'MarkerSize', 20);
    
    % Convert LIDAR rays to Breezy SLAM compatable format
    % Breezy SLAM takes data from -180 to +180 degrees
    % getLidar returns data from 0 to 360 degrees
    temp = lidarRays < LIDAR_RANGE;
    visLidarRays = lidarRays .* temp;
    temp = temp * 100;
    temp = lidarRays .* temp;
    mid = round(NUM_LIDAR_LINES/2);
    slamLidarRays(1:mid,:) = flip( temp(1:mid,:) );
    slamLidarRays(mid+1:NUM_LIDAR_LINES,:) = flip( temp(mid+1:NUM_LIDAR_LINES,:) );

    % SLAM update position and map
    % velocities = [linear_speed_mm/s, angular_speed_deg/s, time_delta_s]
    odo_velocities = findVelocities( [odo_roboX, odo_roboY, steering_angle], last_odo_pos, 1);
    true_velocities = findVelocities( [roboX, roboY, steering_angle], last_true_pos, 1);
    slam = slam.update(slamLidarRays(:,1), odo_velocities);
    [x_mm, y_mm, theta_degrees] = slam.getpos();
    slam_map = slam.getmap();
    slam_theta = -theta_degrees;
    slam_roboX = (x_mm / 100);
    slam_roboY = ENVIRONMENT_SIZE - (y_mm / 100);
        
    % Pathfinding
    [ heading, map ] = pathfinder( [slam_roboX, slam_roboY], target_pos, map, slam_map );
    
    % Update steering controller
    last_odo_pos = [odo_roboX, odo_roboY, steering_angle];
    last_true_pos = [roboX, roboY, steering_angle];
    [steering_angle, priorValues] = steer(heading,priorValues,last_true_pos(3));
    
    % Update motion model
    [roboX,roboY,odo_roboX,odo_roboY] = motionModel(steering_angle, roboX, roboY);
    roboTheta = steering_angle;
    
    % Display Lidar rays
    if( VIS_LIDAR_RAYS )
        [numRays,~] = size(visLidarRays);
        for j = 1:numRays
            X = roboX + visLidarRays(j,1)*cos(visLidarRays(j,2));
            Y = roboY + visLidarRays(j,1)*sin(visLidarRays(j,2));
            line([roboX;X],[roboY;Y],'Color',[1 0 0])
        end
    end

    % Display map
    figure(f2)
    clf
    image(slam_map/4) % Keep bytes in [0,64] for colormap
    
    figure(f1);
    hold on;
    % Generate a polyline to represent the robot
    HEIGHT_RATIO = 1.25; 
    slam_x_pix = [-robot_size,              robot_size, -robot_size]; 
    slam_y_pix = [-robot_size/HEIGHT_RATIO, 0 ,          robot_size/HEIGHT_RATIO];
    true_x_pix = [-robot_size,              robot_size, -robot_size]; 
    true_y_pix = [-robot_size/HEIGHT_RATIO, 0 ,          robot_size/HEIGHT_RATIO];

    % Rotate the polyline based on the robot's angular rotation
    theta_radians = pi * slam_theta / 180;
    slam_x_pix_r = slam_x_pix * cos(theta_radians) - slam_y_pix * sin(theta_radians);
    slam_y_pix_r = slam_x_pix * sin(theta_radians) + slam_y_pix * cos(theta_radians);
    true_x_pix_r = true_x_pix * cos(theta_radians) - true_y_pix * sin(theta_radians);
    true_y_pix_r = true_x_pix * sin(theta_radians) + true_y_pix * cos(theta_radians);

    % Add the robot's position as offset to the polyline
    slam_x_pix = slam_x_pix_r + slam_roboX;
    slam_y_pix = slam_y_pix_r + slam_roboY;
    true_x_pix = true_x_pix_r + roboX;
    true_y_pix = true_y_pix_r + roboY;
    
    % Add robot image to map
    fill(slam_x_pix, slam_y_pix, 'b')
    fill(true_x_pix, true_y_pix, 'r')
    drawnow  
    
    pause(0.05)
end

close all;

