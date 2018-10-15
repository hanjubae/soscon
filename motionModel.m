function [true_roboX, true_roboY, noise_roboX, noise_roboY] = motionModel(steering_angle, in_roboX, in_roboY)
% Function implementing the motion model of the robot
%   Accepts a steering angle and true coordinate, returns both a true and
%   noisy coordinate pair. The true should be given to getLidar, the noisy
%   one to SLAM
%   Robot drives at a constant speed of ROBO_SPEED

    % Robot speed is in .... (pixels?) per tick
    ROBO_SPEED = 1;
    DISTANCE_STD_DEV = 0.1;
    STRAIGHT_STD_DEV = 0.1;
    
    true_roboX = in_roboX + ROBO_SPEED*cosd(steering_angle);
    
    true_roboY = in_roboY + ROBO_SPEED*sind(steering_angle);
    
    noise_roboX = in_roboX + ...
        (((DISTANCE_STD_DEV*(sum(rand(12,1))-6.0)) + ROBO_SPEED)...
         * cosd(((STRAIGHT_STD_DEV*steering_angle)*(sum(rand(12,1))-6.0)) + steering_angle));
     
    noise_roboY = in_roboY + ...
        (((DISTANCE_STD_DEV*(sum(rand(12,1))-6.0)) + ROBO_SPEED)...
         * sind(((STRAIGHT_STD_DEV*steering_angle)*(sum(rand(12,1))-6.0)) + steering_angle));
end