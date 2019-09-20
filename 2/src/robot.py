import vrep
import sys
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import os
from settings import PioLaser, PioRobot
from myslam import MySlam


class Robot():
    propConst = 10.1
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
    
    def erCheck(self, e, str):
        if e == -1:
            print('Somthing wrong with {0}'.format(str))
            sys.exit()
    
    def robotControl(self):
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

    def slamRobotControl(self, slam):
        (errorCode, sensorState, sensorDetection, detectedObjectHandle,
            detectedSurfaceNormalVectorUp) = vrep.simxReadProximitySensor(self.clientID, self.sensor, vrep.simx_opmode_streaming)
        (errorCode, frontState, frontDetection, detectedObjectHandle,
            detectedSurfaceNormalVectorFr) = vrep.simxReadProximitySensor(self.clientID, self.sensorFr, vrep.simx_opmode_streaming)
        if (slam.toPoint and slam.mayCalc):
            angle = slam.calcAngle(slam.pose[0]/1000., slam.pose[1]/1000., slam.distX, slam.distY)
            delta = -angle + slam.pose[2] % 360.0
            print(f"robot angle: {slam.pose[2]}, course: {angle}")
            if not frontState and not sensorState:
                self.calulate(True, self.reqDist + delta * slam.angleCf)
            elif frontState and sensorState:
                val1 = self.reqDist + delta * slam.angleCf
                val2 = frontDetection[2] + self.frontAdd
                val3 = sensorDetection[2]
                self.calulate(True, min(val1, min(val2, val3)))
            elif sensorState:
                val1 = self.reqDist + delta * slam.angleCf
                val2 = sensorDetection[2]
                self.calulate(True, min(val1, val2))
            else:
                val1 = self.reqDist + delta * slam.angleCf
                val2 = frontDetection[2] + self.frontAdd
                self.calulate(True, min(val1, val2))
        else:    
            if (frontState and sensorState):
                self.calulate(sensorState, min(sensorDetection[2], frontDetection[2] + self.frontAdd))
            elif (frontState):
                self.calulate(frontState, frontDetection[2] + self.frontAdd)
            elif (sensorState):
                self.calulate(sensorState, sensorDetection[2])
            else:
                self.calulate(sensorState, self.reqDist + 0.1)
        slam.updateMap(self)

    def simulate(self):
        slam = MySlam(self, True, 5, 28)
        while vrep.simxGetConnectionId(self.clientID) != -1:
            self.slamRobotControl(slam)


if __name__ == "__main__":
    robot = Robot()
    robot.simulate()