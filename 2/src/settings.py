from breezyslam.vehicles import WheeledVehicle
from breezyslam.sensors import URG04LX, Laser


class PioLaser(Laser):
    
    def __init__(self):
        # detection_margin=70, offset_mm=145
        Laser.__init__(self, 684, 10, 240, 20000, detection_margin=70, offset_mm=145) #detection_margin=70
        # URG04LX.__init__(self, 70, 145)
        

class PioRobot(WheeledVehicle):
    
    def __init__(self):
        
        WheeledVehicle.__init__(self, 195 / 2, 381 / 2)
        
        self.ticks_per_cycle = 2000
                        
    def __str__(self):
        
        return '<%s ticks_per_cycle=%d>' % (WheeledVehicle.__str__(self), self.ticks_per_cycle)
        
    def computePoseChange(self, timestamp, leftWheelOdometry, rightWheelOdometry):
        
        return WheeledVehicle.computePoseChange(self, timestamp, leftWheelOdometry, rightWheelOdometry)

    def extractOdometry(self, timestamp, leftWheel, rightWheel):     
        return timestamp, \
               self._rad_to_degrees(leftWheel), \
               self._rad_to_degrees(rightWheel)
               
    def odometryStr(self, odometry):
        
        return '<timestamp=%d usec leftWheelTicks=%d rightWheelTicks=%d>' % \
               (odometry[0], odometry[1], odometry[2])
               
    def _rad_to_degrees(self, rad):
        return rad * 100#57.2958

    def _ticks_to_degrees(self, ticks):
        
        return ticks * (180. / self.ticks_per_cycle)
        
        
        
        
        
