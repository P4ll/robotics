import vrep
import sys
import math
import time

class Pioneer:
    clientID = -1

    propConst = 1.0
    integralConst = 0
    diffConst = 1.0

    integralMaxVal = 0.2
    integralMinVal = -0.2
    integralSum = 0.0
    prevDist = -1

    leftWeelSpeed = 0.2
    rightWeelSpeed = 0.2

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
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
        if self.clientID == -1:
            print('Connection not successful')
            sys.exit('Could not connect')
        else:
            print("Connected to remote server")
        
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_oneshot_wait)
        
        errorCode, self.leftMotor = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'leftMotor')
        errorCode, self.rightMotor = vrep.simxGetObjectHandle(self.clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'rightMotor')
        errorCode, self.sensorFr = vrep.simxGetObjectHandle(self.clientID, 'Prox1', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'Prox1')
        errorCode, self.sensor = vrep.simxGetObjectHandle(self.clientID, 'Prox2', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'Prox2')
        # errorCode, self.sensorLeft = vrep.simxGetObjectHandle(self.clientID, 'Prox3', vrep.simx_opmode_oneshot_wait)
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

        # print('dist = {0} speedLeft = {1} speedRight = {2} res = {3}'.format(dist, self.leftWeelSpeed - result, self.rightWeelSpeed + result, result))

    def simulate(self, kP, kD, kI):
        self.propConst = kP
        self.integralConst = kI
        self.diffConst = kD
        t = time.clock()
        recs = list()
        maxVal, minVal = 0.0, 1e9
        while vrep.simxGetConnectionId(self.clientID) != -1:
            (errorCode, sensorState, sensorDetection, detectedObjectHandle,
                detectedSurfaceNormalVectorUp) = vrep.simxReadProximitySensor(self.clientID, self.sensor, vrep.simx_opmode_streaming)
            # (errorCode, sensorStateLeft, sensorDetectionLeft, detectedObjectHandle,
            #     detectedSurfaceNormalVectorUp) = vrep.simxReadProximitySensor(self.clientID, self.sensorLeft, vrep.simx_opmode_streaming)
            
            (errorCode, frontState, frontDetection, detectedObjectHandle,
                detectedSurfaceNormalVectorFr) = vrep.simxReadProximitySensor(self.clientID, self.sensorFr, vrep.simx_opmode_streaming)
            # if ():
            #     vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot_wait)
            #     vrep.simxFinish(self.clientID)
            #     break
            # if :
            #     vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot_wait)
            #     vrep.simxFinish(self.clientID)
            #     break
            if sensorState == True:
                self.calulate(sensorState, sensorDetection[2])
                maxVal = max(sensorDetection[2], maxVal)
                minVal = min(sensorDetection[2], minVal)
            recs.append(sensorDetection[2])
            if (len(recs) == 30):
                recs.pop(0)
            res = 0.0
            for el in recs:
                res += abs(el - self.reqDist)
            res /= len(recs)
            flag = min(self.reqDist, res) / max(self.reqDist, res) < 0.01
            flag1 = (time.clock() - t) > 100
            flag2 = frontDetection == True and frontDetection[2] < 0.5
            if (flag or flag1 or flag2):
                t2 = time.clock()
                t2 -= t
                overs = abs(self.reqDist - maxVal) / self.reqDist * 100.0
                print(f'done with time =  {t2} and overshoot = {overs}')
                print(f'kP = {kP}, kD = {kD}, kI = {kI}')
                vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot_wait)
                vrep.simxFinish(self.clientID)
                break
            time.sleep(0.1)


if __name__ == "__main__":
    # kP, kD, kI = 10.1, 0.5, 1.5
    # step = 0.5
    it1 = [i / 10.0 for i in range(0, 55, 5)]
    it2 = [i / 10.0 for i in range(51, 102, 5)]

    # pio = Pioneer()
    # pio.simulate(kP, kD, kI)

    # for p in it1:
    #     pio = Pioneer()
    #     pio.simulate(kP, kD, p)

    for i in it1:
        for d in it1:
            for p in it2:
                pio = Pioneer()
                pio.simulate(p, d, i)