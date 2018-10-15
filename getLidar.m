function [lidarRays] = getLidar(robotX,robotY,robotTheta,walls)
%Returns a "NUM_LIDAR_LINES"x2 array of lidar data
%   lidarRays = [[radius1, radius2...]',[theta1,theta2...]'];
%   0 <= radius <= LIDAR_RANGE
%   radius is in pixels from (robotX,robotY) to obstacle
%   0 <= theta <= 2*pi
%   theta is in radians from global positive x axis
%   
%   Each wall in 'walls' is a row vector:
%       [          ...            ]
%       [X1     Y1      X2      Y2]
%       [          ...            ]
%   WARNING!!! WALL CURRENTLY CANNOT GO THROUGH ORIGIN (0,0)
%
%   Requires isBetween.m

% getLidar Macros
NUM_LIDAR_LINES     = 100;
LIDAR_RANGE         = 40;
LIDAR_STD_DEV       = 0.0;
LIDAR_BIAS          = 0;

%Create rays matrix

robotTheta = deg2rad(robotTheta);

t = linspace(0,2*pi,NUM_LIDAR_LINES+1);
for i = 1:(length(t)-1);
    lidarRays(i,1) = LIDAR_RANGE;
    lidarRays(i,2) = t(i) + robotTheta;
end

[numWalls,~] = size(walls);
[numRays,~] = size(lidarRays);

%For each LIDAR ray...
for i = 1:numRays
    
    %Find its ending coordinate
    X = robotX + LIDAR_RANGE*cos( t(i) + robotTheta);
    Y = robotY + LIDAR_RANGE*sin( t(i) + robotTheta);
    
    %Find standard form of LIDAR ray
    R = [robotX robotY ; X Y];
    Q = inv(R)*[1;1];
    
    %For each wall...
    for j = 1:numWalls
        
        %Find standard form of wall
        P = [walls(j,1:2) ; walls(j,3:4)];
        O = inv(P)*[1;1];
        
        %Find intersection of wall and LIDAR ray
        INT = inv([Q';O'])*[1;1];
        intX = INT(1);  % x coordinate of intersection
        intY = INT(2);  % y coordinate of intersection
        
        %Check that intersection actually occurs within the
        %bounds of the LIDAR ray and wall
        if ((isBetween(intY,robotY,Y) == 1) && ...
            (isBetween(intX,robotX,X) == 1) && ...
            (isBetween(intX,walls(j,1),walls(j,3)) == 1) && ...
            (isBetween(intY,walls(j,2),walls(j,4)) == 1))
%             disp('INTERSECTION')
            temp = hypot((intX-robotX),(intY-robotY));
            
            %Clip ranges beyond max sensitivity
            if(temp > LIDAR_RANGE)
                temp = LIDAR_RANGE;
            end
            
            %Only store the minimum range for each wall
            if (lidarRays(i,1) > temp)
                lidarRays(i,1) = temp;
                
            end
        else
%             disp('NOPE')
        end
    end
    
end

for i = 1:numRays
    if (lidarRays(i,1) < 40)
        lidarRays(i,1) = lidarRays(i,1) + ...
            (LIDAR_STD_DEV*lidarRays(i,1))*(sum(rand(12,1))-6.0) + LIDAR_BIAS;
    end
end
for i = 1:numRays
    if (lidarRays(i,1) < 0)
        lidarRays(i,1) = 0;
    end
end

end

