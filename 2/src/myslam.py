import vrep
import sys
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import breezyslam as brs
from breezyslam.sensors import URG04LX, Laser
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.vehicles import WheeledVehicle
from roboviz import MapVisualizer
import keyboard
from settings import PioLaser, PioRobot
from threading import Thread

class MySlam():
    MAP_SIZE_PIXELS = 800
    MAP_SIZE_METERS = 32

    useMap = True
    toPoint = False
    mayCalc = False
    distX = 5
    distY = 28
    angleCf = 0.005 #0.0005

    def __init__(self, robot, useMap = False, distX = None, distY = None, angleCf = 0.0005):
        self.robot = robot
        self.useMap = useMap
        if distX not None and distY not None:
            self.toPoint = True
            self.distX = distX
            self.distY = distY
        self.angleCf = angleCf

        errorCode, self.sensorFH = vrep.simxGetObjectHandle(self.clientID, 'fastHokuyo', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'sensorFH')

        if (self.useMap):
            file = open('robotMap.map', 'rb')
            self.mapbytes = bytearray(file.read())
        else:
            self.mapbytes = bytearray(self.MAP_SIZE_PIXELS * self.MAP_SIZE_PIXELS)
        self.laser = PioLaser()
        self.robotSettings = PioRobot()
        self.slam = RMHC_SLAM(self.laser, self.MAP_SIZE_PIXELS, self.MAP_SIZE_METERS, random_seed = 9999, map_quality=1)
        self.viz = MapVisualizer(self.MAP_SIZE_PIXELS, self.MAP_SIZE_METERS, "1", True)
        if (self.useMap):
            self.slam.setmap(self.mapbytes)
        self.viz.display(self.distX, self.distY, 0, self.mapbytes)
        self.pose = [0, 0, 0]

    def calcAngle(self, x1, y1, x2, y2):
        k = (y1 - y2) / (x1 - x2)
        b = (x1 * y2 - x2 * y1) / (x1 - x2)
        val = math.atan(math.fabs(k))

        if (x2 < x1 and y2 > y1):
            return 180. - val * 57.2958
        elif (x2 < x1 and y2 < y1):
            return 180. + val * 57.2958
        elif (x2 > x1 and y2 < y1):
            return 270. + val * 57.2958
        else:
            return val * 57.2958

    def updateMap(self, robot):
        data = vrep.simxGetStringSignal(robot.clientID, 'measuredDataAtThisTime', vrep.simx_opmode_streaming)
        dataDists = vrep.simxGetStringSignal(robot.clientID, 'dataDistsAtThisTime', vrep.simx_opmode_streaming)
        rawOdometry = vrep.simxGetStringSignal(robot.clientID, 'odometryAtThisTime', vrep.simx_opmode_streaming)

        odometry = vrep.simxUnpackFloats(rawOdometry[1])
        meaData = vrep.simxUnpackFloats(data[1])
        dists = vrep.simxUnpackFloats(dataDists[1])
        if (len(dists) == 0):
            continue

        for i in range(len(dists)):
            dists[i] = (dists[i] * 1000)

        velocities = self.robotSettings.computePoseChange(time.time(), odometry[0], odometry[1])
        self.slam.update(dists) #velocities
        self.pose[0], self.pose[1], self.pose[2] = self.slam.getpos()
        self.slam.getmap(self.mapbytes)
        self.mayCalc = True
        if not self.viz.display(self.pose[0]/1000., self.pose[1]/1000., self.pose[2], self.mapbytes):
            exit(0)