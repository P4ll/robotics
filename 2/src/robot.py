import vrep
import sys
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import os
from settings import PioLaser, PioRobot
from slam import MySlam


class Robot():
    """
    Robot class
    Consts:
        propConst - proportional constant in PID
        integralConst - integral constant in PID
        diffConst - diff const in PID
        integralMaxVal, integralMinVal - lower and upper bounds of integral module
        integralSum - inital integral sum
        prevDist - previous distance to PID
        leftWeelSpeed, rightWeelSpeed - std speed
        reqDist - required dist
        frontAdd - smoothing const of front detector
    """
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
        """
        Constructor
        Connection to V-REP and parms init
        """
        vrep.simxFinish(-1)
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
        if self.clientID == -1:
            print('Connection not successful')
            sys.exit('Could not connect')
        else:
            print("Connected to remote server")

        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_streaming)
        
        # getting handlers of V-REP obj
        errorCode, self.sensorFr = vrep.simxGetObjectHandle(self.clientID, 'Prox1', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'Prox1')
        errorCode, self.sensor = vrep.simxGetObjectHandle(self.clientID, 'Prox2', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'Prox2')
        
        errorCode, self.leftMotor = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'leftMotor')
        errorCode, self.rightMotor = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'rightMotor')

    def addLeftSpeed(self, newSpeed):
        """
        Control speed of left wheel
        """
        ns = self.leftWeelSpeed + newSpeed
        ns = min(ns, self.maxSpeed)
        ns = max(ns, self.minSpeed)
        e = vrep.simxSetJointTargetVelocity(self.clientID, self.leftMotor, ns, vrep.simx_opmode_oneshot_wait)
        self.erCheck(e, 'leftMotor')

    def addRightSpeed(self, newSpeed):
        """
        Control speed of right wheel
        """
        ns = self.rightWeelSpeed + newSpeed
        ns = min(ns, self.maxSpeed)
        ns = max(ns, self.minSpeed)
        e = vrep.simxSetJointTargetVelocity(self.clientID, self.rightMotor, ns, vrep.simx_opmode_oneshot_wait)
        self.erCheck(e, 'rightMotor')

    def calulate(self, state, dist):
        #calc of a integral, proportional and diff componets
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
        # getting a PID res
        result = propComponent + diffComponent + integralComponent
        
        # speed control
        self.addLeftSpeed(-result)
        self.addRightSpeed(result)
    
    def erCheck(self, e, str):
        if e == -1:
            print('Somthing wrong with {0}'.format(str))
            sys.exit()
    
    def robotControl(self):
        # std alg of a Pioneer control
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
        # getting data from a side and front sensors
        (errorCode, sensorState, sensorDetection, detectedObjectHandle,
            detectedSurfaceNormalVectorUp) = vrep.simxReadProximitySensor(self.clientID, self.sensor, vrep.simx_opmode_streaming)
        (errorCode, frontState, frontDetection, detectedObjectHandle,
            detectedSurfaceNormalVectorFr) = vrep.simxReadProximitySensor(self.clientID, self.sensorFr, vrep.simx_opmode_streaming)
        
        if (slam.toPoint and slam.mayCalc): # if need ride to the point and len(data) != 0
            # calc angle between Ox and robot line
            angle = slam.calcAngle(slam.pose[0]/1000., slam.pose[1]/1000., slam.distX, slam.distY)
            delta = -angle + slam.pose[2] % 360.0

            if not frontState and not sensorState: # if nothing bothers
                self.calulate(True, self.reqDist + delta * slam.angleCf)
            elif frontState and sensorState: # if wall right and front
                val1 = self.reqDist + delta * slam.angleCf
                val2 = frontDetection[2] + self.frontAdd
                val3 = sensorDetection[2]
                self.calulate(True, min(val1, min(val2, val3)))
            elif sensorState: # if wall right
                val1 = self.reqDist + delta * slam.angleCf
                val2 = sensorDetection[2]
                self.calulate(True, min(val1, val2))
            else: # wall front
                val1 = self.reqDist + delta * slam.angleCf
                val2 = frontDetection[2] + self.frontAdd
                self.calulate(True, min(val1, val2))
        else:
            # std alg
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
        slam = MySlam(self)
        while vrep.simxGetConnectionId(self.clientID) != -1:
            self.slamRobotControl(slam)


if __name__ == "__main__":
    robot = Robot()
    robot.simulate()