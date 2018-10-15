%%
clf
clc
clear
figure(1)

numLines = 50;
lidarRange = 40;

w = [   1   1   101 1;
        1   1   1   101;
        101 1   101 101;
        1   101 101 101;
        5   85  25  55;
        20  20  40  41;
        60  50  80  40;
        50  90  50  70;
        60  20  60  40;
        30  70  50  70];
    
[numWalls,~] = size(w);

% getLidar Macros
    NUM_LIDAR_LINES     = 50;
    LIDAR_RANGE         = 40;
    LIDAR_STD_DEV       = 1;
    LIDAR_BIAS          = 0;
    

for i = 1:80
    pos(i,1) = 10+i;
    pos(i,2) = 10;
end
for i = 81:160
    pos(i,1) = 90;
    pos(i,2) = -70+i;
end
for i = 161:240
    pos(i,1) = 90-(i-161);
    pos(i,2) = 90-(2*(i-161)/3);
end

for i = 1:240
    
    roboX = pos(i,1);
    roboY = pos(i,2);

    x = [w(:,1)';w(:,3)'];
    y = [w(:,2)';w(:,4)'];

    lidar = getLidar(roboX,roboY,w);
    clf
    line(x,y,'Color',[0 0 0])
    axis([-10 110 -10 110])
    hold on

    [numRays,~] = size(lidar);
    for i = 1:numRays
        X = roboX + lidar(i,1)*cos(lidar(i,2));
        Y = roboY + lidar(i,1)*sin(lidar(i,2));
        line([roboX;X],[roboY;Y],'Color',[1 0 0])
    end
    
    pause(0.1)
end