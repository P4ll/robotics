import vrep
import sys
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import cv2
import array
from PIL import Image
import imutils


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
        lowerGreen, upperGreen - bounds of color detection
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

    lowerGreen = np.array([0, 180, 0], dtype=np.uint8)
    upperGreen = np.array([50, 255, 56], dtype=np.uint8)

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
        errorCode, self.visSensor = vrep.simxGetObjectHandle(self.clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'visSensor')

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

    def calcVertByCont(self, c):
        # calc vertex by contours
        perimetr = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * perimetr, True)
        return len(approx)

    def isSq(self, c):
        # check for square
        perimetr = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * perimetr, True)
        
        if (len(approx) == 6):
            return True
        _, _, w, h = cv2.boundingRect(approx)
        ar = w / float(h)

        return True if ar >= 0.60 and ar <= 1.4 else False
        # return True if ar >= 0.90 and ar <= 1.1 else False

    def imageProcessing(self, img):
        # getting mask by color
        mask = cv2.inRange(img, self.lowerGreen, self.upperGreen)
        res = cv2.bitwise_and(img, img, mask=mask)

        # getting contours
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # check for cuboid for all cnt
        for c in cnts:
            if (self.isSq(c)): #  or self.calcVertByCont(c) == 4 or self.calcVertByCont(c) == 6
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(img, (x, y), (x + w, y + h), (250, 0, 0), 2)
        return img
    
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

    def simulate(self):
        res = vrep.simxGetVisionSensorImage(self.clientID, self.visSensor, 0, vrep.simx_opmode_streaming)
        cv2.namedWindow('camera', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('camera', 400, 400)

        while vrep.simxGetConnectionId(self.clientID) != -1:
            self.robotControl()

            visionSensorData = vrep.simxGetVisionSensorImage(self.clientID, self.visSensor, 0, vrep.simx_opmode_buffer)
            if (len(visionSensorData) == 0 or visionSensorData[0] != vrep.simx_return_ok):
                continue

            pixelFlow = np.array(visionSensorData[2][::-1], np.uint8)
            image = np.reshape(pixelFlow, (visionSensorData[1][0], visionSensorData[1][1], 3), 'C')
            image = cv2.flip(image, 1)
            image = self.imageProcessing(image)

            cv2.imshow('camera', image)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()


if __name__ == "__main__":
    robot = Robot()
    robot.simulate()