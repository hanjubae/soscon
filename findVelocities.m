function [ velocities ] = findVelocities( robot_pos, last_pos, dt )
% Finds linear and and angular speed in (mm/s) and (deg/s)
%   Used to convert robot odometry data into format suitable for use with
%   Breezy SLAM
%   output: velocities = [linear_speed_mm/s, angular_speed_deg/s, time_delta_s]

    dx = robot_pos(1) - last_pos(1);
    dy = robot_pos(2) - last_pos(2);
    dist = dx^2 + dy^2;
    dist = sqrt(dist);
    velocities(1) = (dist/dt) * 100;
    
    dtheta = robot_pos(3) - last_pos(3);
    if (dtheta < -180)
        dtheta = dtheta + 360;
    elseif (dtheta > 180)
        dtheta = dtheta - 360;
    else
    end
    dtheta = -dtheta;
    velocities(2) = dtheta / dt;
    
    velocities(3) = dt;
end

