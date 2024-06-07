import math
import numpy as np


def cartesian_vector_to_polar(x, y):
    """
        Converts a cartesian vector (x and y coordinates) to polar form (magnitude and direction).
        Outputs a tuple of magnitude and direction of the inputted vector
    """
    magnitude = np.sqrt(x**2 + y**2)
    direction = np.arctan2(y, x) # radians
    direction = direction * (180/np.pi)  # angle from -180 to 180 degrees
    direction = direction % 360  # angle from 0 to 360 degrees
    return magnitude, direction

def get_bearing(cur_lat, cur_long, dest_lat, dest_long):
    '''
    utility function to get the bearing towards a specific destination point, from our current location.
    This returns the bearing as an angle between 0 to 360, counter clockwise, measured from north
    '''
    X = float(dest_long) - float(cur_long) #delta X
    Y = float(dest_lat) - float(cur_lat) #delta Y
    return (math.atan2(Y, X)*(180/math.pi) + 360)%360
    