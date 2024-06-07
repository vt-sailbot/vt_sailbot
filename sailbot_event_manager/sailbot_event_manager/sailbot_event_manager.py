#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, Bool, String, UInt8, Int32
from sailbot_msgs.msg import RCData, WaypointList, AutopilotParameters
from sensor_msgs.msg import NavSatFix
import numpy as np
import requests, json
import time

from .utils import *
from .globals import *

DEBUG = False

class SailbotEventManager(Node):
    """
    This is a script that tells every other node how they are supposed to complete the current event for the SailBot competition.
    """

    def __init__(self):
        super().__init__("SailbotEventManager")
        
        self.create_timer(0.025, self.main_loop)
        
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.rc_listener = self.create_subscription(msg_type=RCData, topic="/rc_data", callback=self.rc_data_callback, qos_profile=sensor_qos_profile)
        self.heading_listener = self.create_subscription(msg_type=Float32, topic="/gps_data/heading", callback=self.heading_callback, qos_profile=sensor_qos_profile)
        
        self.mast_angle_publisher = self.create_publisher(msg_type=Float32, topic="/actions/mast_angle", qos_profile=sensor_qos_profile)
        self.rudder_angle_publisher = self.create_publisher(msg_type=Float32, topic="/actions/rudder_angle", qos_profile=sensor_qos_profile)
        
        self.autopilot_mode_publisher = self.create_publisher(msg_type=String, topic='/autopilot_mode', qos_profile=sensor_qos_profile)
        self.autopilot_parameters_publisher = self.create_publisher(msg_type=AutopilotParameters, topic='/autopilot_parameters', qos_profile=10)
        
        self.desired_route_publisher = self.create_publisher(WaypointList, '/desired_route', qos_profile=10)
        self.telemetry_messages_publisher = self.create_publisher(String, '/telemetry_messages', qos_profile=10)
        
        self.no_rudder_control_publisher = self.create_publisher(msg_type=Bool, topic="/no_rudder_control", qos_profile=10)
        self.disable_autopilot_publisher = self.create_publisher(msg_type=Bool, topic="/disable_autopilot", qos_profile=10)
        self.termination_listener = self.create_subscription(msg_type=Bool, topic="/should_terminate", callback=self.should_terminate_callback, qos_profile=10)


        self.holding_heading_semi_auto = 0.
        self.heading = 0.
        self.rc_data = RCData()
        self.current_autopilot_mode = AutopilotMode.Disabled
        self.waypoints = []
        
        self.joystick_left_x = 0.
        self.joystick_left_y = 0.
        self.joystick_right_x = 0.
        self.joystick_right_y = 0.
        
        self.toggle_b = 0
        self.toggle_c = 0
        self.toggle_f = 2
        
                
        waypoints = [
            NavSatFix(longitude=0., latitude=0.),
            NavSatFix(longitude=0.5e-4, latitude=-0.5e-4), 
            NavSatFix(longitude=1.5e-4, latitude=0.),
            NavSatFix(longitude=0.5e-4, latitude=0.5e-4),
            NavSatFix(longitude=0., latitude=0.),
            NavSatFix(longitude=0.5e-4, latitude=-0.5e-4), 
            NavSatFix(longitude=1.5e-4, latitude=0.),
            NavSatFix(longitude=0.5e-4, latitude=0.5e-4),
            NavSatFix(longitude=0., latitude=0.)
        ]
        
        if DEBUG:
            self.desired_route_publisher.publish(WaypointList(waypoints=waypoints))   
        

    def heading_callback(self, heading: Float32):
        self.heading = heading.data
        

    def rc_data_callback(self, joystick_msg: RCData):
        if joystick_msg.toggle_f == 1 and self.toggle_f != 1: 
            self.holding_heading_semi_auto = self.heading
            
        self.joystick_left_x = joystick_msg.joystick_left_x
        self.joystick_left_y = joystick_msg.joystick_left_y
        self.joystick_right_x = joystick_msg.joystick_right_x
        self.joystick_right_y = joystick_msg.joystick_right_y
        
        self.toggle_b = joystick_msg.toggle_b
        self.toggle_c = joystick_msg.toggle_c
        self.toggle_f = joystick_msg.toggle_f


    def execute_rc_control(self, disable_mast=False, disable_rudder=False):
        # https://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio 
        
        # self.autopilot_mode_publisher.publish(String(data="MANUAL"))
        mast_angle = (((self.joystick_left_y + 100) * (MAX_MAST_ANGLE - MIN_MAST_ANGLE)) / 200) + MIN_MAST_ANGLE
        rudder_angle = (((self.joystick_right_x + 100) * (MAX_RUDDER_ANGLE - MIN_RUDDER_ANGLE)) / 200) + MIN_RUDDER_ANGLE
        
        if not disable_mast:
            self.mast_angle_publisher.publish(Float32(data=float(mast_angle)))
            
        if not disable_rudder:
            self.rudder_angle_publisher.publish(Float32(data=float(rudder_angle)))
        
    def get_raw_response(self):
        try:
            return json.loads(requests.get(TELEMETRY_SERVER_URL + "api/wp/one").text)
        except:
            time.sleep(1)
            return self.get_raw_response()
        
    def get_waypoints_and_autopilot_parameters_from_server(self) -> WaypointList:
        """
            Waypoints are of the format: WP1_lat, WP2_lon; WP2_lat, WP2_lon; WP3_lat, WP3_lon; etc
        """
        raw_response = self.get_raw_response()
        
        if raw_response == None: return None, None
        else: requests.delete(TELEMETRY_SERVER_URL + "api/wp/deleteAll")
    
        response_dict = dict(json.loads(raw_response["w"]))
        
        # parse waypoints
        waypoints_list = response_dict["waypoints"]
        
        waypoints_nav_sat_fix_list = []
        for waypoint in waypoints_list:
            if not waypoint: continue
            
            lat, lon = waypoint
            
            try: float(lat) and float(lon)
            except: raise Exception("Waypoints from Server Were Improperly Formatted")
            
            waypoints_nav_sat_fix_list.append(NavSatFix(latitude=float(lat), longitude=float(lon)))
            waypoint_list = WaypointList(waypoints=waypoints_nav_sat_fix_list)
        
        
        # parse parameters
        parameters = AutopilotParameters()
        parameters.sail_lookup_table_wind_angles = response_dict["sail_lookup_table_wind_angles"]
        parameters.sail_lookup_table_sail_positions = response_dict["sail_lookup_table_sail_positions"]
        parameters.forced_jibe_only = bool(response_dict["forced_jibe_only"])
        parameters.waypoint_accuracy = float(response_dict["waypoint_accuracy"])
        parameters.no_sail_zone_size = float(response_dict["no_sail_zone_size"])
        parameters.tack_distance = float(response_dict["tack_distance"])
        
        parameters.rudder_p_gain = float(response_dict["rudder_p_gain"])
        parameters.rudder_i_gain = float(response_dict["rudder_i_gain"])
        parameters.rudder_d_gain = float(response_dict["rudder_d_gain"])
        parameters.rudder_n_gain = float(response_dict["rudder_n_gain"])
        
        return waypoint_list, parameters
    
    
    def main_loop(self):
        """
        receive information about which event we are currently trying to complete and then use the correct logic to complete it.
        This should be called as frequently as possible so that it is as responsive to human input as possible
        """ 
        if DEBUG: return
            
        waypoints, parameters = self.get_waypoints_and_autopilot_parameters_from_server()
        
        if parameters: self.autopilot_parameters_publisher.publish(parameters)
        if waypoints: self.desired_route_publisher.publish(waypoints)
        
        
        # disarmed
        if self.toggle_b != 0:
            self.current_autopilot_mode = AutopilotMode.Disabled
            
        # full autonomy
        elif self.toggle_f == 2:
            self.current_autopilot_mode = AutopilotMode.Waypoint_Mission
            
        # hold heading to the direction that we started this mode in. the rudder is controlled via RC
        elif self.toggle_f == 1 and self.toggle_c == 0:
            self.current_autopilot_mode = AutopilotMode.Hold_Heading
            self.execute_rc_control(disable_rudder=True)
            
        # choose the best sail angle based on the lookup table. the rudder is controlled via RC
        elif self.toggle_f == 1 and self.toggle_c == 1:
            self.current_autopilot_mode = AutopilotMode.Hold_Best_Sail
            self.execute_rc_control(disable_mast=True)

        # hold heading and best sail
        elif self.toggle_f == 1 and self.toggle_c == 2:
            self.current_autopilot_mode = AutopilotMode.Hold_Heading_And_Best_Sail

        # remote controlled
        elif self.toggle_f == 0:
            self.current_autopilot_mode = AutopilotMode.Disabled
            self.execute_rc_control()
        
        else:
            print("WARNING: INCORRECT COMBINATION OF RC SWITCHES USED")
            return
            
        self.autopilot_mode_publisher.publish(String(data=self.current_autopilot_mode.name))       
       
       
    # def send_message_to_telemetry(self, message):
    #     self.telemetry_messages_publisher.publish(String(data=message))
        
    def should_terminate_callback(self, msg: Bool):
        if msg.data == False: return
        rclpy.shutdown()
        
        
def main(args=None):
    rclpy.init(args=args)

    sailbot_event_manager = SailbotEventManager()
    rclpy.spin(sailbot_event_manager)


if __name__ == "__main__":
    main()
