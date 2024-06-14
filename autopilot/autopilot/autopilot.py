import geopy
import geopy.distance

from .discrete_pid import Discrete_PID
from .utils import *
from . import constants

# TODO rename cur_waypoint_index to waypoint_index or something better
# TODO make all methods except for step private

class SailbotAutopilot:
    """
    Controls the boat based on a hard coded policy.
    This class contains all of the code to control a sailboat given an observation.

    The main method you should care about is the step method at the very bottom 
    which takes in all of the observations and outputs the correct sail and rudder angle to navigate to the next waypoint

    This class is used by the autopilot_node to control the boat through a ros topic
    """
    
    def __init__(self):
        self.rudder_pid_controller = Discrete_PID(
            sample_period=(1/constants.AUTOPILOT_REFRESH_RATE), 
            Kp=constants.RUDDER_P_GAIN, Ki=constants.RUDDER_I_GAIN, Kd=constants.RUDDER_D_GAIN, n=constants.RUDDER_N_GAIN, 
        )
        
        self.waypoints = None
        self.cur_waypoint_index = 0
        
        self.current_state = States.NORMAL
        
        self.jibe_direction = 0
        self.desired_tacking_angle = 0
    
   
   
    def reset(self):
        # TODO implement and test this
        # when the boat receives this is should clear out its current route and 
        self.__init__()
    
            

    def get_optimal_sail_angle(self, apparent_wind_angle: float):
        """
        Runs a single step by using the sail lookup table. No side effects. Apparent wind angle is measured ccw from the right hand side of the boat.
        
        Doesn't return an exit code because there is no reason why this should fail and this part of the code doesn't figure out if the boat has reached the waypoint
        Returns the desired sail angle and rudder angle as a tuple given the current observations
        """
    
        # 180 means wind pushing you backwards, 90 for the sail means let the sails all the way out
        # these are for close hauled, close reach, beam reach, broad reach and running respectively
        # the angles were estimated from a sailing position diagram and adam should probably take a look and move things around as he sees fit

        sail_positions = list(constants.SAIL_LOOKUP_TABLE.values())
        wind_angles = list(constants.SAIL_LOOKUP_TABLE.keys())

        left = max(filter(lambda pos: pos <= float(apparent_wind_angle), wind_angles))
        right = min(filter(lambda pos: pos >= float(apparent_wind_angle), wind_angles))

        left = wind_angles.index(left)
        right = wind_angles.index(right)
        
        sail_angle = 0
        if (left == right):
            for i in range(len(sail_positions)):
                if float(apparent_wind_angle) == wind_angles[i]:
                    sail_angle = sail_positions[i]
        else:
            slope = (sail_positions[right] - sail_positions[left])/(wind_angles[right] - wind_angles[left])
            sail_angle = slope * (float(apparent_wind_angle) - wind_angles[left]) + sail_positions[left]
        
        return sail_angle
        
        
    def get_optimal_rudder_angle(self, heading, desired_heading):
        error = get_distance_between_angles(desired_heading, heading)
        
        rudder_angle = self.rudder_pid_controller(error)
        rudder_angle = np.clip(rudder_angle, -constants.RUDDER_HARD_OVER, constants.RUDDER_HARD_OVER)
        return rudder_angle
    
    
    
    def get_decision_zone_size(self, distance_to_waypoint):
        inner = (constants.TACK_DISTANCE/distance_to_waypoint) * np.sin(np.deg2rad(constants.NO_SAIL_ZONE_SIZE/2))
        inner = np.clip(inner, -1, 1)
        return np.clip(np.rad2deg(np.arcsin(inner)), 0, constants.NO_SAIL_ZONE_SIZE/2)
     
        
        
    def apply_decision_zone_tacking_logic(self, heading, desired_heading, true_wind_angle, distance_to_waypoint):
        """
        If you don't know how this works (you don't) feel free to ask Chris about this and he can explain it to you
        """
        
        global_true_wind_angle = (heading + true_wind_angle) % 360
        global_true_up_wind_angle = (global_true_wind_angle + 180) % 360
        print(f"no sail size: {constants.NO_SAIL_ZONE_SIZE/2}")
        no_sail_zone_bounds = ((global_true_up_wind_angle - constants.NO_SAIL_ZONE_SIZE/2) % 360, (global_true_up_wind_angle + constants.NO_SAIL_ZONE_SIZE/2) % 360)  # lower, upper

        decision_zone_size = self.get_decision_zone_size(distance_to_waypoint)
        print(f"decision zone bounds: {decision_zone_size}")
        decision_zone_bounds = ((global_true_up_wind_angle - decision_zone_size) % 360, (global_true_up_wind_angle + decision_zone_size) % 360)
        
        # desired_heading = (desired_heading + 180) % 360
        print(f"desired heading: {desired_heading}")
        print(f"heading: {heading}")
        print(f"no sail bounds: {no_sail_zone_bounds}")
        print(f"decision bounds: {decision_zone_bounds}")
        
    
        # If desired heading it is not in any of the zones
        if not is_angle_between_boundaries(desired_heading, no_sail_zone_bounds[0], no_sail_zone_bounds[1]): 
            print("I AM IN ZONE NONE, SAILING NORMAL")  
            if get_maneuver_from_desired_heading(heading, desired_heading, true_wind_angle) == Maneuvers.TACK:
                return desired_heading, True
            else:
                return desired_heading, False


        # If desired heading is in zone 1
        if is_angle_between_boundaries(desired_heading, decision_zone_bounds[1], no_sail_zone_bounds[1]):
            print("I AM IN ZONE 1")  
            if (heading - global_true_up_wind_angle) % 360 < 180:   # Starboard side of true wind
                return no_sail_zone_bounds[1], False
                
            else:   # Port side of the true wind
                # sbtd tack
                return no_sail_zone_bounds[1], True
                   
                                
        # If desired heading is in zone 3
        if is_angle_between_boundaries(desired_heading, decision_zone_bounds[0], no_sail_zone_bounds[0]):
            print("I AM IN ZONE 3")  
            if (heading - global_true_up_wind_angle) % 360 < 180:   # Starboard side of true wind
                # port tack
                return no_sail_zone_bounds[0], True
                
            else:   # Port side of the true wind
                return no_sail_zone_bounds[0], False
    
        
        # If desired heading in zone 2      
        print("I AM IN ZONE 2")  
        distance_to_lower_no_sail_zone = abs(get_distance_between_angles(no_sail_zone_bounds[0], heading))
        distance_to_upper_no_sail_zone = abs(get_distance_between_angles(no_sail_zone_bounds[1], heading))
        
        print(f"global_true_up_wind_angle: {global_true_up_wind_angle}")
        print(f"no_sail_zone_bounds: {no_sail_zone_bounds}")
        print(f"distance to lower no sail zone: {distance_to_lower_no_sail_zone}")
        print(f"distance to upper no sail zone: {distance_to_upper_no_sail_zone}")
        print(f"")
        print()
        
        if distance_to_lower_no_sail_zone < distance_to_upper_no_sail_zone: 
            return no_sail_zone_bounds[0], False
        
        else: 
            return no_sail_zone_bounds[1], False
       
       
       
        
    
    def run_rc_control(self, joystick_left_y, joystick_right_x):
        # https://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio 
        sail_angle = (((joystick_left_y + 100) * (constants.MAX_SAIL_ANGLE - constants.MIN_SAIL_ANGLE)) / 200) + constants.MIN_SAIL_ANGLE
        rudder_angle = (((joystick_right_x + 100) * (constants.MAX_RUDDER_ANGLE - constants.MIN_RUDDER_ANGLE)) / 200) + constants.MIN_RUDDER_ANGLE
    
        return sail_angle, rudder_angle


    def run_hold_best_sail_step(self, apparent_wind_vector):
        # TODO refactor so that I don't have to convert in here (we are converting back to angle when we don't have to, just pass in an angle)
        _, apparent_wind_angle = cartesian_vector_to_polar(apparent_wind_vector[0], apparent_wind_vector[1])
        
        sail_angle = self.get_optimal_sail_angle(apparent_wind_angle)
        return sail_angle, None
    
    
    def run_hold_heading_step(self, heading, desired_heading):
        rudder_angle = self.get_optimal_rudder_angle(heading, desired_heading)
        return None, rudder_angle
    
    
    def run_hold_sail_and_rudder_step(self, apparent_wind_vector, heading, desired_heading):
        # TODO refactor so that I don't have to convert in here (we are converting back to angle when we don't have to, just pass in an angle)
        _, apparent_wind_angle = cartesian_vector_to_polar(apparent_wind_vector[0], apparent_wind_vector[1])
        
        sail_angle = self.get_optimal_sail_angle(apparent_wind_angle)
        rudder_angle = self.get_optimal_rudder_angle(heading, desired_heading)
        return sail_angle, rudder_angle
        
        
        
        
    def run_waypoint_mission_step(self, cur_position: Position, velocity_vector: np.ndarray, heading_: float, apparent_wind_vector_: np.ndarray):
        """
        takes in the following:
            - position as a Position object (from autopilot_utils)
            - velocity as a numpy array with 2 elements (x, y) in meters/ second
            - heading as a float in degrees measured ccw from true east
            - apparent wind vector as a numpy array with 2 elements (x, y) in meters/ second. This captures data about AWA and AWS
        
        returns the sail angle, rudder angle, and exit code
        """

        boat_speed = np.sqrt(velocity_vector[0]**2 + velocity_vector[1]**2)
        heading = heading_

        # https://en.wikipedia.org/wiki/Apparent_wind#/media/File:DiagramApparentWind.png
        true_wind_vector = apparent_wind_vector_ + velocity_vector
        
        true_wind_speed, true_wind_angle = cartesian_vector_to_polar(true_wind_vector[0], true_wind_vector[1])
        apparent_wind_speed, apparent_wind_angle = cartesian_vector_to_polar(apparent_wind_vector_[0], apparent_wind_vector_[1])
        
        global_true_wind_angle = true_wind_angle + heading

        if self.waypoints:
            desired_pos = self.waypoints[self.cur_waypoint_index]
        else:
            desired_pos = Position(90, 90)  # we are never reaching this position

        distance_to_desired_pos = get_distance_between_positions(cur_position, desired_pos)

        
        
        # Has Reached The Waypoint
        if geopy.distance.geodesic(cur_position.get_lat_lon(), desired_pos.get_lat_lon()).m < constants.WAYPOINT_ACCURACY: 
            
            if len(self.waypoints) <= self.cur_waypoint_index + 1:    # Has Reached The Final Waypoint
                self.reset()
                return None, None              
            
            self.cur_waypoint_index += 1
            
        
        
        if self.current_state == States.NORMAL:
                        
            desired_heading = get_bearing(current_pos=cur_position, destination_pos=desired_pos)
            desired_heading, should_tack1 = self.apply_decision_zone_tacking_logic(heading, desired_heading, true_wind_angle, distance_to_desired_pos)
            
            global_true_up_wind_angle = (180 + global_true_wind_angle) % 360
            should_tack2 = is_angle_between_boundaries(global_true_up_wind_angle, heading, desired_heading)
            
            if should_tack1 or should_tack2:
                optimal_rudder_angle = self.get_optimal_rudder_angle(heading, desired_heading)
                
                self.desired_tacking_angle = desired_heading
                
                if optimal_rudder_angle > 0: 
                    self.current_state = States.CW_TACK  
                else: 
                    self.current_state = States.CCW_TACK

            
            rudder_angle = self.get_optimal_rudder_angle(heading, desired_heading)
            sail_angle = self.get_optimal_sail_angle(apparent_wind_angle)
            
            
            

        elif self.current_state == States.CW_TACK or self.current_state == States.CCW_TACK:
            sail_angle = self.get_optimal_sail_angle(apparent_wind_angle)

            if self.current_state == States.CW_TACK:
                tack_direction = 1
            else:
                tack_direction = -1

            rudder_angle = constants.RUDDER_HARD_OVER * tack_direction
            
            if constants.PERFORM_FORCED_JIBE_INSTEAD_OF_TACK: 
                rudder_angle *= -1

            if abs((heading - self.desired_tacking_angle)) % 360 < constants.TACK_TOLERANCE: # if we have finished the tack
                self.current_state == States.NORMAL
            
        
        else: Exception("Unsupported State Transitioned Into")
        
        return sail_angle, rudder_angle