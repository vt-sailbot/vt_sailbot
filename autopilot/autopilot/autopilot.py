import math
import time
import geopy
import geopy.distance
from .discrete_pid import Discrete_PID

from .autopilot_util import *
from . import constants

# TODO rename cur_waypoint_index to waypoint_index or something better
# TODO make all methods except for step private
# TODO try combining jibing and tacking into 1 maneuver so that we don't rewrite a lot of the same code


class SailbotAutoController:
    """
    Controls the boat based on a hard coded policy.
    This class contains all of the code to control a sailboat given an observation.

    The main method you should care about is the step method at the very bottom 
    which takes in all of the observations and outputs the correct mast and rudder angle to navigate to the next waypoint

    This method is used by the autopilot_node to control the boat through a ros topic
    """
    
    def __init__(self):
        self.rudder_pid_controller = Discrete_PID(
            sample_period=(1/constants.autopilot_refresh_rate), 
            Kp=constants.rudder_p_gain, Ki=constants.rudder_i_gain, Kd=constants.rudder_d_gain, n=constants.rudder_n_gain, 
        )
        
        self.route = None
        self.cur_waypoint_index = 0
        
        self.current_state = States.NORMAL
        
        self.start_jibe_time = 0
        self.jibe_direction = 0
        
        self.start_tack_time = 0
        self.tack_direction = 0
        self.desired_tacking_angle = 0
    
   
   
    def reset(self):
        # TODO implement and test this
        # when the boat receives this is should clear out its current route and 
        self.__init__()
    
    def set_new_route(self, waypoints: list[Position]):
        self.route = Route(waypoints=waypoints)
        self.cur_waypoint_index = 0

    
    
    
    def run_tack_step(self, tack_angle, tack_direction, heading):
        '''
        This executes one tack step if the boat is currently in the tacking state.
        
        Exit codes:
            0: normally exited but still needs to continue attempting to tack for the subsequent steps to reach the desired tack angle
            1: completed the tack successfully and should return to normal sailing
            
        Returns the desired rudder angle and exit code
        '''
        
        # if tack_direction != -1 and tack_direction != 1: raise Exception("Incorrect Arguments Passed into run_tack_step")
        
        # rudder_angle = constants.rudder_hard_over
        
        
        # if constants.perform_forced_jibe_instead_of_tack:
        #     rudder_angle = constants.rudder_hard_over * tack_direction
        # else:
        if constants.perform_forced_jibe_instead_of_tack:
            rudder_angle = -1 * constants.rudder_hard_over * tack_direction
        else:
            rudder_angle = constants.rudder_hard_over * tack_direction

        # # check if it has been too long since we started
        # if (time.time() - start_tack_time) > constants.action_time_limit:
        #     print("TACK FAILED")
        #     return rudder_angle, -1

        tack_complete = abs((heading - tack_angle)) % 360 < constants.tack_tolerance

        return rudder_angle, tack_complete
    
    
    
    def run_jibe_step(self, jibe_direction, jibe_angle, heading, start_jibe_time):
        '''
        This executes one jibe step if the boat is currently in the jibing state.
        The jibe_direction is -1 for counter-clockwise and 1 for clockwise.
        
        Exit codes:
            0: normally exited but still needs to continue attempting to jibe for the subsequent steps to reach the desired jibe angle
            1: completed the jibe successfully and should return to normal sailing
            -1: failed to jibe because it took too long
            
        Returns the desired rudder angle and exit code
        '''
        
        if jibe_direction != -1 and jibe_direction != 1: raise Exception("Incorrect Arguments Passed into run_jibe_step")
        
        rudder_angle = constants.rudder_hard_over
        if jibe_direction == -1: rudder_angle *= -1

        # check if it has been too long since we started
        if (time.time() - start_jibe_time) > constants.action_time_limit:
            print("JIBE FAILED")
            return rudder_angle, -1

        jibe_complete = abs((heading - jibe_angle)) % 360 < constants.jibe_tolerance

        return rudder_angle, jibe_complete
        
        

    def get_optimal_sail_angle(self, apparent_wind_angle: float):
        """
        Runs a single step by using the sail lookup table. No side effects. Apparent wind angle is measured ccw from the right hand side of the boat.
        
        Doesn't return an exit code because there is no reason why this should fail and this part of the code doesn't figure out if the boat has reached the waypoint
        Returns the desired mast angle and rudder angle as a tuple given the current observations
        """
    
        # 180 means wind pushing you backwards, 90 for the sail means let the sails all the way out
        # these are for close hauled, close reach, beam reach, broad reach and running respectively
        # the angles were estimated from a sailing position diagram and adam should probably take a look and move things around as he sees fit

        sail_positions = list(constants.lookup_table.values())
        wind_angles = list(constants.lookup_table.keys())

        left = max(filter(lambda pos: pos <= float(apparent_wind_angle), wind_angles))
        right = min(filter(lambda pos: pos >= float(apparent_wind_angle), wind_angles))

        left = wind_angles.index(left)
        right = wind_angles.index(right)
        
        mast_angle = 0
        if (left == right):
            for i in range(len(sail_positions)):
                if float(apparent_wind_angle) == wind_angles[i]:
                    mast_angle = sail_positions[i]
        else:
            slope = (sail_positions[right] - sail_positions[left])/(wind_angles[right] - wind_angles[left])
            mast_angle = slope * (float(apparent_wind_angle) - wind_angles[left]) + sail_positions[left]
        
        return mast_angle
        
        
    def get_optimal_rudder_angle(self, heading, desired_heading):
        error = get_distance_between_angles(desired_heading, heading)
        
        rudder_angle = self.rudder_pid_controller(error)
        rudder_angle = np.clip(rudder_angle, -constants.rudder_hard_over, constants.rudder_hard_over)
        return rudder_angle
    
    
    
    def get_decision_zone_size(self, distance_to_waypoint):
        inner = (constants.tack_distance/distance_to_waypoint) * np.sin(np.deg2rad(constants.no_sail_zone_size/2))
        inner = np.clip(inner, -1, 1)
        return np.clip(np.rad2deg(np.arcsin(inner)), 0, constants.no_sail_zone_size/2)
     
        
        
    def clip_desired_heading_to_no_sail_zone(self, heading, desired_heading, true_wind_angle, distance_to_waypoint):
        global_true_wind_angle = (heading + true_wind_angle) % 360
        global_true_up_wind_angle = (global_true_wind_angle + 180) % 360
        print(f"no sail size: {constants.no_sail_zone_size/2}")
        no_sail_zone_bounds = ((global_true_up_wind_angle - constants.no_sail_zone_size/2) % 360, (global_true_up_wind_angle + constants.no_sail_zone_size/2) % 360)  # lower, upper

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
        
        # # if it is not in the decision boundary but it is in the no go zone
        # if not is_angle_between_boundaries(desired_heading, decision_zone_bounds[0], decision_zone_bounds[1]):
            
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
        
        
        
    # TODO: make everything either an instance variable or not an instance variable. None of this inconsistancy is ok
    def run_waypoint_mission_step(self, cur_position: Position, velocity_vector: np.ndarray, heading_: float, apparent_wind_vector_: np.ndarray):
        """
        takes in the following:
            - position as a Position object (from autopilot_utils)
            - velocity as a numpy array with 2 elements (x, y) in meters/ second
            - heading as a float in degrees measured ccw from true east
            - apparent wind vector as a numpy array with 2 elements (x, y) in meters/ second. This captures data about AWA and AWS
        
        returns the mast angle, rudder angle, and exit code
        """
        # if not self.route: return None, None
        
        boat_speed = np.sqrt(velocity_vector[0]**2 + velocity_vector[1]**2)
        heading = heading_

        # https://en.wikipedia.org/wiki/Apparent_wind#/media/File:DiagramApparentWind.png
        true_wind_vector = apparent_wind_vector_ + velocity_vector
        
        true_wind_speed, true_wind_angle = cartesian_vector_to_polar(true_wind_vector[0], true_wind_vector[1])
        apparent_wind_speed, apparent_wind_angle = cartesian_vector_to_polar(apparent_wind_vector_[0], apparent_wind_vector_[1])
        
        global_true_wind_angle = true_wind_angle + heading

        if self.route:
            self.route.recalculate_route(cur_position, true_wind_angle, self.cur_waypoint_index)
            desired_pos = self.route.waypoints[self.cur_waypoint_index]
        else:
            # NOTE: This should only be called when the boat is in semi auto mode so that
            desired_pos = Position(90, 90)  # we are never reaching this position

        distance_to_desired_pos = get_distance_between_positions(cur_position, desired_pos)
        print(cur_position.get_lat_lon())
        print(desired_pos.get_lat_lon())

        
        # Handle State Transitions for the Finite State Machine (FSM)
        
        # Has Reached The Waypoint
        if geopy.distance.geodesic(cur_position.get_lat_lon(), desired_pos.get_lat_lon()).m < constants.waypoint_accuracy: 
            
            if len(self.route.waypoints) <= self.cur_waypoint_index + 1:    # Has Reached The Final Waypoint
                self.reset()
                return None, None              
            
            self.cur_waypoint_index += 1
            
        
        
        
        # Code for Sailing Normally
        if self.current_state == States.NORMAL:
                        
            desired_heading = get_bearing(current_pos=cur_position, destination_pos=desired_pos)
            desired_heading, should_tack1 = self.clip_desired_heading_to_no_sail_zone(heading, desired_heading, true_wind_angle, distance_to_desired_pos)
            
            global_true_up_wind_angle = (180 + global_true_wind_angle) % 360
            should_tack2 = is_angle_between_boundaries(global_true_up_wind_angle, heading, desired_heading)
            
            if should_tack1 or should_tack2:
                self.current_state = States.TACK
                self.desired_tacking_angle = desired_heading
                optimal_rudder_angle = self.get_optimal_rudder_angle(heading, desired_heading)
                self.tack_direction = optimal_rudder_angle / abs(optimal_rudder_angle)
            
            rudder_angle = self.get_optimal_rudder_angle(heading, desired_heading)
            mast_angle = self.get_optimal_sail_angle(apparent_wind_angle)
            
            

            print(f"optimal rudder angle: {rudder_angle}")

        
        
        # # Code for Executing a Jibe
        # elif self.current_state == States.JIBE:
        #     desired_angle = get_bearing(cur_position, desired_pos)
        #     rudder_angle, jibe_exit_code = self.run_jibe_step(self.jibe_direction, desired_angle, heading, self.start_jibe_time)
        #     mast_angle = self.get_optimal_sail_angle(apparent_wind_angle)
            
        #     if jibe_exit_code == 1:
        #         self.current_state = States.NORMAL
            
        #     elif jibe_exit_code == -1:
        #         print("ERROR: Jibe failed. Retrying jibe")
                
            
            
        # Code for Executing a Tack  
        elif self.current_state == States.TACK:
            # desired_angle = get_bearing(cur_position, desired_pos)
            print(f"tacking: {self.desired_tacking_angle}")
            rudder_angle, tack_exit_code = self.run_tack_step(self.desired_tacking_angle, self.tack_direction, heading)
            mast_angle = self.get_optimal_sail_angle(apparent_wind_angle)

            if tack_exit_code == 1:
                self.current_state = States.NORMAL
            
            # elif tack_exit_code == -1:
            #     print("ERROR: Tack failed. Retrying tack")
                

        return mast_angle, rudder_angle
    
    
    
    
    def run_hold_best_sail_step(self, apparent_wind_vector):
        # TODO refactor so that I don't have to convert in here
        _, apparent_wind_angle = cartesian_vector_to_polar(apparent_wind_vector[0], apparent_wind_vector[1])
        
        mast_angle = self.get_optimal_sail_angle(apparent_wind_angle)
        return mast_angle, None
    
    
    def run_hold_heading_step(self, heading, desired_heading):
        rudder_angle = self.get_optimal_rudder_angle(heading, desired_heading)
        return None, rudder_angle
    
    
    def run_hold_sail_and_rudder_step(self, apparent_wind_vector, heading, desired_heading):
        # TODO refactor so that I don't have to convert in here
        _, apparent_wind_angle = cartesian_vector_to_polar(apparent_wind_vector[0], apparent_wind_vector[1])
        
        mast_angle = self.get_optimal_sail_angle(apparent_wind_angle)
        rudder_angle = self.get_optimal_rudder_angle(heading, desired_heading)
        return mast_angle, rudder_angle