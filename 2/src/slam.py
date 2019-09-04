import vrep
import sys
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import breezyslam as brs
from breezyslam.sensors import URG04LX, Laser
from breezyslam.algorithms import RMHC_SLAM
from roboviz import MapVisualizer
import keyboard
# from rviz import MapVisualizer

class Slam():
    MAP_SIZE_PIXELS = 800
    MAP_SIZE_METERS = 32

    propConst = 10.1
    integralConst = 1.5
    diffConst = 0.5

    integralMaxVal = 0.2
    integralMinVal = -0.2
    integralSum = 0.0
    prevDist = -1

    leftWeelSpeed = 1
    rightWeelSpeed = 1

    reqDist = 0.55

    def addLeftSpeed(self, newSpeed):
        e = vrep.simxSetJointTargetVelocity(self.clientID, self.leftMotor, self.leftWeelSpeed + newSpeed, vrep.simx_opmode_oneshot_wait)
        self.erCheck(e, 'leftMotor')

    def addRightSpeed(self, newSpeed):
        e = vrep.simxSetJointTargetVelocity(self.clientID, self.rightMotor, self.rightWeelSpeed + newSpeed, vrep.simx_opmode_oneshot_wait)
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
        # self.laser = URG04LX(70, 145)
        self.laser = Laser(684, 10, 240, 4000, 70, 145)
        self.slam = RMHC_SLAM(self.laser, self.MAP_SIZE_PIXELS, self.MAP_SIZE_METERS, random_seed = 9999)
        self.viz = MapVisualizer(self.MAP_SIZE_PIXELS, self.MAP_SIZE_METERS, "1")
        self.pose = [0, 0, 0]

        self.addLeftSpeed(1)
        self.addRightSpeed(1)
    
    def erCheck(self, e, str):
        if e == -1:
            print('Somthing wrong with {0}'.format(str))
            sys.exit()

    def simulate(self):
        # self.test = 0
        prevTime = 0
        while vrep.simxGetConnectionId(self.clientID) != -1:
            if keyboard.is_pressed('w'):
                self.addLeftSpeed(1)
                self.addRightSpeed(1)
            elif keyboard.is_pressed('s'):
                self.addLeftSpeed(0)
                self.addRightSpeed(0)
            elif keyboard.is_pressed('a'):
                self.addLeftSpeed(0)
                self.addRightSpeed(1)
            elif keyboard.is_pressed('d'):
                self.addLeftSpeed(1)
                self.addRightSpeed(0)
            # (errorCode, sensorState, sensorDetection, detectedObjectHandle,
            #     detectedSurfaceNormalVectorUp) = vrep.simxReadProximitySensor(self.clientID, self.sensor, vrep.simx_opmode_streaming)
            # (errorCode, frontState, frontDetection, detectedObjectHandle,
            #     detectedSurfaceNormalVectorFr) = vrep.simxReadProximitySensor(self.clientID, self.sensorFr, vrep.simx_opmode_streaming)
            # if (frontState and sensorState):
            #     self.calulate(sensorState, min(sensorDetection[2], frontDetection[2]))
            # elif (frontState):
            #     self.calulate(frontState, frontDetection[2])
            # elif (sensorState):
            #     self.calulate(sensorState, sensorDetection[2])
            # else:
            #     self.calulate(sensorState, self.reqDist + 0.1)
            
            data = vrep.simxGetStringSignal(self.clientID, 'measuredDataAtThisTime', vrep.simx_opmode_oneshot_wait) #simx_opmode_streaming
            dataDists = vrep.simxGetStringSignal(self.clientID, 'dataDistsAtThisTime', vrep.simx_opmode_oneshot_wait)
            meaData = vrep.simxUnpackFloats(data[1])
            dists = vrep.simxUnpackFloats(dataDists[1])
            # print(f"{len(dists)}")
            # print(dists)
            for i in range(len(dists)):
                dists[i] = math.trunc(dists[i] * 1000)
                if dists[i] == 5000 or dists[i] == 4999:
                    dists[i] = 0
            self.slam.update(dists)
            self.pose[0], self.pose[1], self.pose[2] = self.slam.getpos()
            self.slam.getmap(self.mapbytes)

            # curTime = time.time()
            # if prevTime > 0:
            #     time.sleep(curTime - prevTime)
            # prevTime = curTime
            
            if not self.viz.display(self.pose[0]/1000., self.pose[1]/1000., self.pose[2], self.mapbytes):
                exit(0)
            # time.sleep(0.1)

if __name__ == "__main__":
    slam = Slam()
    slam.simulate()