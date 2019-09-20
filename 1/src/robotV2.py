import vrep
import sys
import math
import time
import numpy as np
import matplotlib.pyplot as plt

class Pioneer:
    timeLine = list()
    errorsPoints = list()
    leftSpeedPoints = list()
    rightSpeedPoints = list()

    clientID = -1
    #10.1, 0.5, 1.5
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

    def erCheck(self, e, str):
        if e == -1:
            print('Somthing wrong with {0}'.format(str))
            sys.exit()

    def addLeftSpeed(self, newSpeed):
        e = vrep.simxSetJointTargetVelocity(self.clientID, self.leftMotor, self.leftWeelSpeed + newSpeed, vrep.simx_opmode_oneshot_wait)
        self.erCheck(e, 'leftMotor')

    def addRightSpeed(self, newSpeed):
        e = vrep.simxSetJointTargetVelocity(self.clientID, self.rightMotor, self.rightWeelSpeed + newSpeed, vrep.simx_opmode_oneshot_wait)
        self.erCheck(e, 'rightMotor')

    def __init__(self):
        vrep.simxFinish(-1)
        self.clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
        if self.clientID == -1:
            print('Connection not successful')
            sys.exit('Could not connect')
        else:
            print("Connected to remote server")
        
        # vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_streaming)
        
        errorCode, self.leftMotor = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'leftMotor')
        errorCode, self.rightMotor = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'rightMotor')
        errorCode, self.sensorFr = vrep.simxGetObjectHandle(self.clientID, 'Prox1', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'Prox1')
        errorCode, self.sensor = vrep.simxGetObjectHandle(self.clientID, 'Prox2', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'Prox2')

        self.addLeftSpeed(0)
        self.addRightSpeed(0)
    
    def getH(self, a, b):
        c = math.sqrt(a ** 2 + b ** 2)
        if abs(c - 0) < 0.000001:
            c = 1.0
        h = a * b / c
        return h
    def getH2(self, a, b):
        return min(a, b)

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

        self.timeLine.append(time.clock())
        self.errorsPoints.append(deltaDist)
        self.rightSpeedPoints.append(self.rightWeelSpeed + result)
        self.leftSpeedPoints.append(self.leftWeelSpeed - result)

        print('dist = {0} speedLeft = {1} speedRight = {2} res = {3}'.format(dist, self.leftWeelSpeed - result, self.rightWeelSpeed + result, result))
    def rotL(self):
        self.addLeftSpeed(-2.0 * self.leftWeelSpeed)
        #self.addRightSpeed()
    def simulate(self):
        t = time.clock()
        recs = list()
        while vrep.simxGetConnectionId(self.clientID) != -1:
            (errorCode, sensorState, sensorDetection, detectedObjectHandle,
                detectedSurfaceNormalVectorUp) = vrep.simxReadProximitySensor(self.clientID, self.sensor, vrep.simx_opmode_streaming)
            (errorCode, frontState, frontDetection, detectedObjectHandle,
                detectedSurfaceNormalVectorFr) = vrep.simxReadProximitySensor(self.clientID, self.sensorFr, vrep.simx_opmode_streaming)
            if (frontState and sensorState):
                self.calulate(sensorState, min(sensorDetection[2], frontDetection[2]))
            elif (frontState):
                self.calulate(frontState, frontDetection[2])
            elif (sensorState):
                self.calulate(sensorState, sensorDetection[2])
            else:
                self.calulate(sensorState, self.reqDist + 0.1)
            time.sleep(0.1)


if __name__ == "__main__":
    pio = Pioneer()
    pio.simulate()

    plt.figure(1)
    plt.plot(pio.timeLine, pio.errorsPoints)
    plt.xlabel('time, sec')
    plt.ylabel('error')
    plt.title('Error')
    plt.figure(2)
    plt.plot(pio.timeLine, pio.leftSpeedPoints, 'y', pio.timeLine, pio.rightSpeedPoints, 'g')
    plt.xlabel('time, sec')
    plt.ylabel('speed')
    plt.title('Speed, yellow - left, green - right')
    plt.show()