# TODO: convert all of the constants to uppercase to match the python standard

# maps wind angles to sail positions for optimal sailing
# default value
lookup_table = {
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
        

perform_forced_jibe_instead_of_tack = False
waypoint_accuracy = 5   # meters
tack_distance = 17  # rough distance to sail before changing tacks based on dynamic tacking system. Ask Chris for more info

rudder_i_gain = 0.3
rudder_p_gain = 1
rudder_d_gain = 0.2
rudder_n_gain = 1


rudder_hard_over = 35

no_sail_zone_size = 100 # in degrees. With a no sail zone size of 40 degrees, this means that the boat shouldn't sail 20 degrees cw or ccw of true north
autopilot_refresh_rate = 5 # how many hz the autopilot publishes at

float_equivalence_tolerance = 0.01
jibe_tolerance = 3
tack_tolerance = 3

MIN_SAIL_ANGLE = 0
MAX_SAIL_ANGLE = 90

MIN_RUDDER_ANGLE = -35
MAX_RUDDER_ANGLE = 35

TELEMETRY_SERVER_URL = 'http://107.23.136.207:8082/'
TIMEOUT_TIME_LIMIT = 10

# obstacle_size = 1 # meters in radius
# # nogo = 80 #nogo zone

# action_time_limit = 210 #about 3.5 minutes
# route_cycle_count = 3
# stall_speed = 0.01 # m/s (0.51 m/s is around 1 knot)  TODO change this later I just set it really low for now!!
# sailing_time_step = 1
# tack_dist_reduction_step = 0.000015 #about 8 meters in degrees on the equator, needs updating for claytor, maybe have someone walk around with their 
# #phone to get GPS coordinates or smth idk

# heading_offset = 0

# address = "ff:1b:d1:04:80:04"
# DIR_ID = '00002a73-0000-1000-8000-00805f9b34fb'
# SPEED_ID = '00002a72-0000-1000-8000-00805f9b34fb'

# arduino_vid = 0x2341
# arduino_pid = 0x0042

# gps_vid = 0x067b
# gps_pid = 0x2303

# imu_vid = 0x10c4
# imu_pid = 0xea60
