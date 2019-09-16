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
import cv2
import array
from PIL import Image
import imutils


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

    lowerGreen = np.array([0, 180, 0], dtype=np.uint8)
    upperGreen = np.array([50, 255, 56], dtype=np.uint8)

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
        errorCode, self.visSensor = vrep.simxGetObjectHandle(self.clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)
        self.erCheck(errorCode, 'visSensor')

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

    def calcVertByCont(self, c):
        perimetr = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * perimetr, True)
        return len(approx)

    def imageProcessing(self, img):
        mask = cv2.inRange(img, self.lowerGreen, self.upperGreen)
        res = cv2.bitwise_and(img, img, mask=mask)
        # return res

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        for c in cnts:
            if (self.calcVertByCont(c) == 4):
                cv2.drawContours(img, [c], -1, (255, 0, 0), 2)
        return np.hstack([img, res])
        

    def simulate(self):
        prevTime = 0
        res = vrep.simxGetVisionSensorImage(self.clientID, self.visSensor, 0, vrep.simx_opmode_streaming)
        cv2.namedWindow('camera', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('camera', 400, 400)
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

            # print(f"Odo: {odometry[0], odometry[1]}, vel: {velocities}, pos: {self.slam.getpos()}")
            # if not self.viz.display(self.pose[0]/1000., self.pose[1]/1000., self.pose[2], self.mapbytes):
            #     exit(0)

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
    slam = Slam()
    slam.simulate()