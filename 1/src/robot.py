import vrep
import sys
import math
import time

class Pioneer:
    clientID = -1

    propConst = 1.0
    integralConst = 0.07
    diffConst = 0.7

    integralMaxVal = 0.2
    integralMinVal = -0.2
    integralSum = 0.0
    prevDist = -1

    #startSpeed = 0.2
    leftWeelSpeed = 0.2
    rightWeelSpeed = 0.2

    reqDist = 0.55

    def erCheck(self, e, str):
        if e == -1:
            print('Somthing wrong with {0}'.format(str))
            sys.exit()

    def addLeftSpeed(self, newSpeed):
        #self.leftWeelSpeed = newSpeed + self.leftWeelSpeed
        e = vrep.simxSetJointTargetVelocity(self.clientID, self.leftMotor, self.leftWeelSpeed + newSpeed, vrep.simx_opmode_oneshot_wait)
        self.erCheck(e, 'leftMotor')

    def addRightSpeed(self, newSpeed):
        #self.rightWeelSpeed = newSpeed + self.rightWeelSpeed
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

        errorCode, self.leftMotor = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'leftMotor')
        errorCode, self.rightMotor = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'rightMotor')
        errorCode, self.sensorFr = vrep.simxGetObjectHandle(self.clientID, 'Prox1', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'Prox1')
        errorCode, self.sensorUp = vrep.simxGetObjectHandle(self.clientID, 'Prox2', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'Prox2')
        # errorCode, self.sensorDown = vrep.simxGetObjectHandle(self.clientID, 'Prox3', vrep.simx_opmode_oneshot_wait)
        # self.erCheck(errorCode, 'Prox3')

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

    def calulate(self, states, detections):
        # a = detections[0][2]
        # b = detections[1][2]
        # h = self.getH(a, b)
        h = detections[0][2]
        if (h == 0):
            h = 0.7
        if (h < 0.1):
            h = 0.7

        #h = self.getH2(a, b)
        deltaDist = self.reqDist - h 
        propComponent = self.propConst * deltaDist
        self.integralSum = self.integralSum + deltaDist
        self.integralSum = min(self.integralSum, self.integralMaxVal)
        self.integralSum = max(self.integralSum, self.integralMinVal)
        integralComponent = self.integralConst * self.integralSum
        if self.prevDist == -1:
            self.prevDist = h
        diffComponent = self.diffConst * (h - self.prevDist)
        self.prevDist = h
        amt = propComponent + diffComponent + integralComponent
        
        self.addLeftSpeed(-amt)
        self.addRightSpeed(amt)
        print('dist = {0} speedLft = {1} speedRight = {2}'.format(h, self.leftWeelSpeed - amt, self.rightWeelSpeed + amt))

    def simulate(self):
        # vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_streaming)
        states = [False, False, False]
        detections = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]] # 0 - UP, 1 - DOWN, 2 - FRONT
        while vrep.simxGetConnectionId(self.clientID) != -1:
            (errorCode, states[0], detections[0], detectedObjectHandle,
                detectedSurfaceNormalVectorUp) = vrep.simxReadProximitySensor(self.clientID, self.sensorUp, vrep.simx_opmode_streaming)
            # (errorCode, states[1], detections[1], detectedObjectHandle,
            #     detectedSurfaceNormalVectorDown) = vrep.simxReadProximitySensor(self.clientID, self.sensorDown, vrep.simx_opmode_streaming)
            (errorCode, states[2], detections[2], detectedObjectHandle,
                detectedSurfaceNormalVectorFr) = vrep.simxReadProximitySensor(self.clientID, self.sensorFr, vrep.simx_opmode_streaming)
            
            self.calulate(states, detections)
            time.sleep(0.1)


if __name__ == "__main__":
    pio = Pioneer()
    pio.simulate()