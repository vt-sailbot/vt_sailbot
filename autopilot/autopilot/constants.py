# TODO: convert all of the constants to uppercase to match the python standard

# maps wind angles to sail positions for optimal sailing
# default value
SAIL_LOOKUP_TABLE = {
    0: 80, 
    45: 60, 
    90: 40,
    135: 20,
    180: 10,
    225: 20,
    270: 40,
    315: 60,
    360: 80
}
        
PERFORM_FORCED_JIBE_INSTEAD_OF_TACK = False
WAYPOINT_ACCURACY = 5 # meters
TACK_DISTANCE = 17  # rough distance to sail before changing tacks based on dynamic tacking system. Ask Chris for more info

RUDDER_P_GAIN = 1
RUDDER_I_GAIN = 0.3
RUDDER_D_GAIN = 0.2
RUDDER_N_GAIN = 1

RUDDER_HARD_OVER = 35

NO_SAIL_ZONE_SIZE = 100  # in degrees. With a no sail zone size of 40 degrees, this means that the boat shouldn't sail 20 degrees cw or ccw of true north
AUTOPILOT_REFRESH_RATE = 5 # how many hz the autopilot publishes at

FLOAT_EQUIVALENCE_TOLERANCE = 0.01
JIBE_TOLERANCE = 3
TACK_TOLERANCE = 3

MIN_SAIL_ANGLE = 0
MAX_SAIL_ANGLE = 90

MIN_RUDDER_ANGLE = -25
MAX_RUDDER_ANGLE = 25

TELEMETRY_SERVER_URL = 'http://107.23.136.207:8082/'
TIMEOUT_TIME_LIMIT = 10