'''
mines.py - classes for the SLAM apparatus used at Paris Mines Tech
             
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
'''
import math
import copy
import os
import time
from soscon.env import Env
from soscon.status import Status
from soscon.data.observation import Observation
from breezyslam.robots import WheeledRobot
from breezyslam.components import XVLidar, URG04LX


# Method to load all from file ------------------------------------------------
# Each line in the file has the format:
#
#  TIMESTAMP  ... Q1  Q1 ... Distances
#  (usec)                    (mm)
#  0          ... 2   3  ... 24 ... 
#  
#where Q1, Q2 are odometry values

def load_data(datadir, dataset):
    
    filename = '%s/%s.dat' % (datadir, dataset)
    print('Loading data from %s...' % filename)
    
    fd = open(filename, 'rt')
    
    timestamps = []
    scans = []
    odometries = []
    
    while True:  
        
        s = fd.readline()
        
        if len(s) == 0:
            break       
            
        toks = s.split()[0:-1] # ignore ''

        timestamp = int(toks[0])

        odometry = timestamp, int(toks[2]), int(toks[3])
                        
        lidar = [int(tok) for tok in toks[24:]]

        timestamps.append(timestamp)
        scans.append(lidar)
        odometries.append(odometry)
        
    fd.close()
        
    return timestamps, scans, odometries

class MinesLaser(XVLidar):  #component.py에서 정의된 XVLidar class를 부모 class로 정의
    
    def __init__(self):
        XVLidar.__init__(self, 0, 0)
        
# Class for MinesRover custom robot ------------------------------------------

class Rover(WheeledRobot):  #robot.py에서 정의된 WheeledRobot을 부모 class로 정의
    LIDAR_DATA_SIZE = 360
    IR_DATA_SIZE = 5

    def __init__(self):
        self._env = Env()
        WheeledRobot.__init__(self, self._env.wheel_radius, (self._env.robot_size.get_values()[0] * 10) / 2)

        self.ticks_per_cycle = self._env.encoder_resolution  # 바퀴가 1cycle을 도는데 몇 encoder가 필요한지 (soscon에서 encoder_resolution에 해당)
        self._prev_obs = None
        self._env.on_observation = self.on_observation
        self.stop()

    def stop(self):
        print("[STOP]")
        self._env.control_robot(0.0, 0.0)
        self._total_dx = 0
        self._total_dy = 0

    def on_observation(self, obs: Observation, status: Status):
        if status is Status.OK:
            self._prev_obs = copy.deepcopy(obs)

    def get_lidar(self) -> float:
        if self._prev_obs is None:
            raise RuntimeError("[ERROR] No measured not yet")

        if len(self._prev_obs.lidar) != self.LIDAR_DATA_SIZE:
            raise RuntimeError("[ERROR] LiDAR was not measured")

        return self._prev_obs.lidar

    def computeVelocities(self):
        if self._prev_obs is None:
            raise RuntimeError("[ERROR] No measured not yet")
        #0.1초 단위로 lidar센서 정보와 velocites정보를 받겠다.
        time.sleep(0.1)

        return self.get_lidar(), WheeledRobot.computeVelocities(self, time.time() * 1e6, self._prev_obs.encoder.left, self._prev_obs.encoder.right)

    def extractOdometry(self, timestamp, leftWheel, rightWheel):
                
        # Convert microseconds to seconds, ticks to angles        
        return timestamp / 1e6, \
               self._ticks_to_degrees(leftWheel), \
               self._ticks_to_degrees(rightWheel)
               
    def odometryStr(self, odometry):
        
        return '<timestamp=%d usec leftWheelTicks=%d rightWheelTicks=%d>' % \
               (odometry[0], odometry[1], odometry[2])
               
    def _ticks_to_degrees(self, ticks):

        return ticks * (180. / self.ticks_per_cycle)

    def __str__(self):

        return '<%s ticks_per_cycle=%d>' % (WheeledRobot.__str__(self), self.ticks_per_cycle)