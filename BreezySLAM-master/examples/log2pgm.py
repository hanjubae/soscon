#!/usr/bin/env python

'''
log2pgm.py : BreezySLAM Python demo.  Reads logfile with odometry and scan data
             from Paris Mines Tech and produces a .PGM image file showing robot 
             trajectory and final map.
             
For details see

    @inproceedings{coreslam-2010,
      author    = {Bruno Steux and Oussama El Hamzaoui},
      title     = {CoreSLAM: a SLAM Algorithm in less than 200 lines of C code},
      booktitle = {11th International Conference on Control, Automation, 
                   Robotics and Vision, ICARCV 2010, Singapore, 7-10 
                   December 2010, Proceedings},
      pages     = {1975-1979},
      publisher = {IEEE},
      year      = {2010}
    }
                 
Copyright (C) 2014 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.

Change log:

20-APR-2014 - Simon D. Levy - Get params from command line
05-JUN-2014 - SDL - get random seed from command line
'''

# Map size, scale 출력되는 이미지 크기
MAP_SIZE_PIXELS          = 800
MAP_SIZE_METERS          = 25

import math
from breezyslam.algorithms import Deterministic_SLAM, RMHC_SLAM
from breezyslam.components import Laser
from breezyslam.robots import WheeledRobot

from mines import MinesLaser, Rover, load_data
from progressbar import ProgressBar
from pgm_utils import pgm_save

from sys import argv, exit, stdout
import time

def main():

    # Bozo filter for input args
    if len(argv) < 3:   #MakeFile에서 <dataset> 사용 X, use_odometry 사용, random_seed 뭔지 아직 모르겠음.
        print('Usage:   %s <dataset> <use_odometry> <random_seed>' % argv[0])
        print('Example: %s exp2 1 9999' % argv[0])
        exit(1)
    
    # Grab input args
    dataset = argv[1]
    use_odometry  =  True if int(argv[2]) else False
    seed =  int(argv[3]) if len(argv) > 3 else 0
    
	# Load the data from the file, ignoring timestamps
    _, loop_size, odometries = load_data('.', dataset)
    
    # Build a robot model if we want odometry
    robot = Rover() if use_odometry else None # 로봇 생성 mines.py에 정의 되어 있음.
        
    # Create a CoreSLAM object with laser params and optional robot object
    #MineLaser()는 lidar 센서를 생성하는 것으로써 mine.py에 정의되어 있음
    #RHMC_SLAM()함수는 따로 건들지 않았음.
    slam = RMHC_SLAM(MinesLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed=seed) \
           if seed \
           else Deterministic_SLAM(MinesLaser(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # Start with an empty trajectory of positions
    # trajectory : 로봇의 경로 추적하는 list
    trajectory = []

    #nscans : scan할 갯수로써 이 갯수에 따라 로봇의 추적 포인트 갯수가 정해짐.
    nscans = 0
    while True:
        try:
            # if robot.check_front_ir() >= 40: 맵핑을 위한 자율주행 알고리즘 들어갈 자리
            #     robot.move(30, 0)
            # else:
            #     robot.rotate_cw(90)

            #lidar센서 정보인 lidars와 breezyslam이 위치정보를 만드는 데 필요한 정보인 velocities list를 만듬
            lidars, velocities = robot.computeVelocities()

            #soscon에서도 NO MEASERD not yet이 처음에 뜨듯이 아직 lidar정보가 받아지지 않았을 시
            #위치정보를 만드는 함수로 들어가지 않도록 제약하는 if문
            if lidars is not None:
                #robot.computeVelocities()함수를 통해 받은 lidars변수는 soscon에서 정의한 class로 구성되어있다.
                #slam.update() 함수에서 첫번째 인자로 받는 lidar센서 정보는 list를 받기 때문에 list로 변환해준다.
                convertoMM = []
                for lidar in lidars:
                    #soscon에서 받은 lidar센서 정보는 cm단위이기때문에 10을 곱해주어 mm로 변환해준다.
                    convertoMM.append(lidar * 10)
                slam.update(convertoMM, velocities)

                # Get new position
                # 맵에서 어느 위치에 있는지에 대한 정보 반환 (정확하게는 아직 분석 X)
                x_mm, y_mm, theta_degrees = slam.getpos()
                # Add new position to trajectory
                trajectory.append((x_mm, y_mm))

                #위치 정보가 추가되었기때문에 +1해줌
                nscans = nscans + 1
                if nscans == 241:   #해당 제약조건에 맞추어 위치값갯수가 정해짐
                    break
            pass
        except RuntimeError as e:
            print(e)
        except KeyboardInterrupt:
            break

    print(trajectory.__str__())

    # Report elapsed time
    #elapsed_sec = time() - start_sec
                                
    # Create a byte array to receive the computed maps
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    
    # Get final map    
    slam.getmap(mapbytes)
    
    # Put trajectory into map as black pixels
    for coords in trajectory:
                
        x_mm, y_mm = coords
                               
        x_pix = mm2pix(x_mm)
        y_pix = mm2pix(y_mm)
                                                                                              
        mapbytes[y_pix * MAP_SIZE_PIXELS + x_pix] = 0;
                    
    # Save map and trajectory as PGM file    
    pgm_save('%s.pgm' % dataset, mapbytes, (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))
            
# Helpers ---------------------------------------------------------        

def mm2pix(mm):
        
    return int(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS))  
    
                    
main()
