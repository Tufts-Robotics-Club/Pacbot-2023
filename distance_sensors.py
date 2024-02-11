# NAME: distanceSensor.py
# PURPOSE: handling low level distance sensor reading/control
# AUTHOR: Emma Bethel

import adafruit_vl53l4cd
from enum import Enum

READING_AMOUNT = 2


class DistanceSensor:

    # PURPOSE: constructor
    # PARAMETERS: name - a human-readable name for the distance sensor (for use 
    #                    in print statements)
    #             tca_channel - tca channel number of the sensor to be read from
    #             avoidance_action - callable to occur when the .avoid function 
    #                                (of the distance sensor) is called (full 
    #                                 disclosure, this made a lot more sense in 
    #                                 earlier versions of the code)
    #             min_dist - threshold reading at which distance sensor is 
    #                        considered "too close" to an obstruction
    # RETURNS: N/A
    def __init__(self, name, tca_channel, min_dist):
        self.name = name
        self.min_dist = min_dist
        self.sensor = adafruit_vl53l4cd.VL53L4CD(tca_channel)
        self.count = 0
    
    @property
    def range(self):
        return self.sensor.distance
    
    # PURPOSE: determines whether there is an object within the "danger zone" 
    #          of the distance sensor (closer than stored min_dist)
    # PARAMETERS: N/A
    # RETURNS: True if there is an obstruction within min dist of the object, 
    #          False otherwise
    def is_too_close(self):
        if self.range < self.min_dist:
            print("I am too close")
            self.count += 1
            if self.count > READING_AMOUNT:
                return True
            print("... but not for long enough")
        else:
            self.count = 0
            print("I'm freeeeeee")
            return False
    
    # PURPOSE: determines whether there is ANYTHING currently within range of 
    #          the distance sensor (regardless of whether it is within the 
    #          danger zone or not)
    # PARAMETERS: N/A
    # RETURNS: True if obstruction found, False otherwise
    def obstructed(self):
        return self.range < 200
    
    def __str__(self):
        return self.name


class DistanceSensors:

    class Direction(Enum):
        LEFT = 2
        RIGHT = 0
        FRONT = 3
        BACK = 1

    def __init__(self, multiplexer, ports):
         self.sensors = {}
         for i in range(4):
             self.sensors[i] = DistanceSensor("name", multiplexer[ports[i]], 5)

    def which_obstructed(self):
         for (key, sensor) in self.sensors.items():
            if sensor.is_too_close():
                return key

