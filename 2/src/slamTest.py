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


class Slam():
    MAP_SIZE_PIXELS = 800#800 # 1000
    MAP_SIZE_METERS = 32#40

    propConst = 10.1 # 10.1
    integralConst = 1.5
    diffConst = 0.5

    integralMaxVal = 0.2
    integralMinVal = -0.2
    integralSum = 0.0
    prevDist = -1

    leftWeelSpeed = 1
    rightWeelSpeed = 1
    maxSpeed = 1.5
    minSpeed = -1.5

    reqDist = 0.55
    frontAdd = -0.15

    def addLeftSpeed(self, newSpeed):
        ns = self.leftWeelSpeed + newSpeed
        ns = min(ns, self.maxSpeed)
        ns = max(ns, self.minSpeed)
        e = vrep.simxSetJointTargetVelocity(self.clientID, self.leftMotor, ns, vrep.simx_opmode_oneshot_wait)
        self.erCheck(e, 'leftMotor')

    def addRightSpeed(self, newSpeed):
        ns = self.rightWeelSpeed + newSpeed
        ns = min(ns, self.maxSpeed)
        ns = max(ns, self.minSpeed)
        e = vrep.simxSetJointTargetVelocity(self.clientID, self.rightMotor, ns, vrep.simx_opmode_oneshot_wait)
        self.erCheck(e, 'rightMotor')

    def calulate(self, state, dist):
        deltaDist = self.reqDist - dist

        propComponent = self.propConst * deltaDist

        self.integralSum = self.integralSum + deltaDist
        self.integralSum = min(self.integralSum, self.integralMaxVal)
        self.integralSum = max(self.integralSum, self.integralMinVal)
        integralComponent = self.integralConst * self.integralSum

        if self.prevDist == -1:
            self.prevDist = dist
        
        diffComponent = self.diffConst * (dist - self.prevDist)

        self.prevDist = dist
        result = propComponent + diffComponent + integralComponent
        
        self.addLeftSpeed(-result)
        self.addRightSpeed(result)

    def __init__(self):
        vrep.simxFinish(-1)
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
        if self.clientID == -1:
            print('Connection not successful')
            sys.exit('Could not connect')
        else:
            print("Connected to remote server")
        
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_streaming)

        errorCode, self.sensorFr = vrep.simxGetObjectHandle(self.clientID, 'Prox1', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'Prox1')
        errorCode, self.sensor = vrep.simxGetObjectHandle(self.clientID, 'Prox2', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'Prox2')
        
        errorCode, self.leftMotor = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'leftMotor')
        errorCode, self.rightMotor = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'rightMotor')
        errorCode, self.sensorFH = vrep.simxGetObjectHandle(self.clientID, 'fastHokuyo', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'sensorFH')

        self.mapbytes = bytearray(self.MAP_SIZE_PIXELS * self.MAP_SIZE_PIXELS)
        self.laser = PioLaser()
        self.robot = PioRobot()
        self.slam = RMHC_SLAM(self.laser, self.MAP_SIZE_PIXELS, self.MAP_SIZE_METERS, random_seed = 9999, map_quality=1)
        self.viz = MapVisualizer(self.MAP_SIZE_PIXELS, self.MAP_SIZE_METERS, "1", True)
        self.pose = [0, 0, 0]
    
    def erCheck(self, e, str):
        if e == -1:
            print('Somthing wrong with {0}'.format(str))
            sys.exit()

    def simulate(self):
        prevTime = 0

        while vrep.simxGetConnectionId(self.clientID) != -1:
            
            (errorCode, sensorState, sensorDetection, detectedObjectHandle,
                detectedSurfaceNormalVectorUp) = vrep.simxReadProximitySensor(self.clientID, self.sensor, vrep.simx_opmode_streaming)
            (errorCode, frontState, frontDetection, detectedObjectHandle,
                detectedSurfaceNormalVectorFr) = vrep.simxReadProximitySensor(self.clientID, self.sensorFr, vrep.simx_opmode_streaming)
            if (frontState and sensorState):
                self.calulate(sensorState, min(sensorDetection[2], frontDetection[2] + self.frontAdd))
            elif (frontState):
                self.calulate(frontState, frontDetection[2] + self.frontAdd)
            elif (sensorState):
                self.calulate(sensorState, sensorDetection[2])
            else:
                self.calulate(sensorState, self.reqDist + 0.1)

            data = vrep.simxGetStringSignal(self.clientID, 'measuredDataAtThisTime', vrep.simx_opmode_streaming)
            dataDists = vrep.simxGetStringSignal(self.clientID, 'dataDistsAtThisTime', vrep.simx_opmode_streaming)
            rawOdometry = vrep.simxGetStringSignal(self.clientID, 'odometryAtThisTime', vrep.simx_opmode_streaming)

            odometry = vrep.simxUnpackFloats(rawOdometry[1])
            meaData = vrep.simxUnpackFloats(data[1])
            dists = vrep.simxUnpackFloats(dataDists[1])
            if (len(dists) == 0):
                continue

            for i in range(len(dists)):
                dists[i] = (dists[i] * 1000)

            velocities = self.robot.computePoseChange(time.time(), odometry[0], odometry[1])
            self.slam.update(dists) #velocities
            self.pose[0], self.pose[1], self.pose[2] = self.slam.getpos()
            self.slam.getmap(self.mapbytes)

            print(f"Odo: {odometry[0], odometry[1]}, vel: {velocities}, pos: {self.slam.getpos()}")
            if not self.viz.display(self.pose[0]/1000., self.pose[1]/1000., self.pose[2], self.mapbytes):
                exit(0)

if __name__ == "__main__":
    slam = Slam()
    slam.simulate()