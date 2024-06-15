#!usr/bin/python3

from autopilot.utils import *

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, Bool, String, Int32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from sailbot_msgs.msg import WaypointList, AutopilotParameters

import numpy as np
import array, time, json, requests

TELEMETRY_SERVER_URL = 'http://107.23.136.207:8082/'

# TODO: MOST IMPORTANT MAKE THE IP AN ARGUMENT INTO THE ROS NODE AND DOCKER CONTAINER

# TODO: Update the website info at a fixed rate
# TODO: add current state of the board and the FSM

class TelemetryNode(Node):

    def __init__(self):
        super().__init__("telemetry")

        # NOTE: All units are in standard SI units and angle is measured in degrees
        self.create_timer(0.2, self.update_everything)

        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.autopilot_parameter_listener = self.create_subscription(AutopilotParameters, '/autopilot_parameters', callback=self.autopilot_parameters_callback, qos_profile=10)
        self.autopilot_parameters_publisher = self.create_publisher(msg_type=AutopilotParameters, topic='/autopilot_parameters', qos_profile=10)
        
        self.desired_heading_listener = self.create_subscription(Float32, '/desired_heading', self.desired_heading_callback, 10)
        self.waypoints_list_listener = self.create_subscription(WaypointList, '/waypoints_list', self.waypoints_list_callback, 10)
        self.waypoints_list_publisher = self.create_publisher(WaypointList, '/waypoints_list', qos_profile=10)
        self.cur_waypoint_index_listener = self.create_subscription(Int32, '/cur_waypoint_index', self.cur_waypoint_index_callback, 10)
        
        self.full_autonomy_maneuver_listener = self.create_subscription(msg_type=String, topic="/full_autonomy_maneuver", callback=self.full_autonomy_maneuver_callback, qos_profile=sensor_qos_profile)
        self.autopilot_mode_listener = self.create_subscription(msg_type=String, topic="/autopilot_mode", callback=self.autopilot_mode_callback, qos_profile=sensor_qos_profile)
        
        self.position_listener = self.create_subscription(msg_type=NavSatFix, topic="/position", callback=self.position_callback, qos_profile=sensor_qos_profile)
        self.velocity_listener = self.create_subscription(msg_type=Vector3, topic="/velocity", callback=self.velocity_callback, qos_profile=sensor_qos_profile)
        self.heading_listener = self.create_subscription(msg_type=Float32, topic="/heading", callback=self.heading_callback, qos_profile=sensor_qos_profile)
        self.apparent_wind_vector_listener = self.create_subscription(msg_type=Vector3, topic="/apparent_wind_vector", callback=self.apparent_wind_vector_callback, qos_profile=sensor_qos_profile)
        
        self.sail_angle_listener = self.create_subscription(msg_type=Float32, topic="/actions/sail_angle", callback=self.sail_angle_callback, qos_profile=sensor_qos_profile)
        self.rudder_angle_listener = self.create_subscription(msg_type=Float32, topic="/actions/rudder_angle", callback=self.rudder_angle_callback, qos_profile=sensor_qos_profile)



        # Default values in case these are never sent through ROS
        # If these values aren't changing then the ros node thats supposed to be sending these values may not be working correctly
        self.waypoints_list: list[NavSatFix] = []
        self.cur_waypoint_index = 0
        self.position = NavSatFix(latitude=0., longitude=0.)
        
        self.autopilot_mode = ""
        self.full_autonomy_maneuver = ""
        self.autopilot_parameters = {}

        self.velocity_vector = np.array([0., 0.])
        self.speed = 0.
        self.heading = 0.
        self.desired_heading = 0.

        self.apparent_wind_vector = np.array([0., 0.])
        self.apparent_wind_speed = 0.
        self.apparent_wind_angle = 0.

        self.sail_angle = 0.
        self.rudder_angle = 0.
        
        self.time = 0 # keeps track of how many callbacks on the website telemetry were made

    
    def desired_heading_callback(self, desired_heading: Float32):
        self.desired_heading = desired_heading.data
    
    def autopilot_parameters_callback(self, autopilot_parameters: AutopilotParameters):
        parameter_names = list(autopilot_parameters._fields_and_field_types.keys())
        self.autopilot_parameters = {parameter_names[i]: getattr(autopilot_parameters, parameter_names[i]) for i in range(len(parameter_names))}
        
        for key in self.autopilot_parameters.keys():
            if type(self.autopilot_parameters[key]) == array.array:
                self.autopilot_parameters[key] = list(self.autopilot_parameters[key])
        
        print(self.autopilot_parameters)
        
        
    def waypoints_list_callback(self, waypointsList: WaypointList):
        self.waypoints_list = waypointsList.waypoints
    
    def cur_waypoint_index_callback(self, cur_waypoint_index: Int32):
        self.cur_waypoint_index = cur_waypoint_index.data
    
    def full_autonomy_maneuver_callback(self, full_autonomy_maneuver: String):
        self.full_autonomy_maneuver = full_autonomy_maneuver.data
        
    def autopilot_mode_callback(self, autopilot_mode: String):
        self.autopilot_mode = autopilot_mode.data
        
    def position_callback(self, position: NavSatFix):
        self.position = position

    # TODO change this so that the telemetry stores both the vector and speed internally
    def velocity_callback(self, velocity_vector: Vector3):
        self.velocity_vector = np.array([velocity_vector.x, velocity_vector.y])
        self.speed = np.sqrt(velocity_vector.x**2 + velocity_vector.y**2)

    def heading_callback(self, heading: Float32):
        self.heading = heading.data

    def apparent_wind_vector_callback(self, apparent_wind_vector: Vector3):
        self.apparent_wind_vector = np.array([apparent_wind_vector.x, apparent_wind_vector.y])

        self.apparent_wind_speed, self.apparent_wind_angle = cartesian_vector_to_polar(apparent_wind_vector.x, apparent_wind_vector.y)
        

    def sail_angle_callback(self, sail_angle: Float32):
        self.sail_angle = sail_angle.data

    def rudder_angle_callback(self, rudder_angle: Float32):
        self.rudder_angle = rudder_angle.data
    
    def should_terminate_callback(self, msg: Bool):
        if msg.data == False: return
        rclpy.shutdown()
    
    
    
    
    def update_everything(self):
        self.update_telemetry_server()
        self.update_autopilot_parameters_from_telemetry()
    
    
    
    def update_telemetry_server(self):
        """
        FORMAT:
            position (Lat Lon tuple); current state; speed; bearing; heading; wind speed; wind direction; sail angle; rudder angle; cur_waypoint; the entire waypoint list for the current route
            each of them are separated as a semicolon
            tuples are denoted as: element1, element2, ... such as for latitude/ longitude
            different entries are denoted as: speed; heading; apparent_wind_speed
        """
        
        true_wind_vector = self.apparent_wind_vector + self.velocity_vector
        self.true_wind_speed, self.true_wind_angle = cartesian_vector_to_polar(true_wind_vector[0], true_wind_vector[1])
        
        telemetry_dict = {
            "position": (self.position.latitude, self.position.longitude), 
            "state": self.autopilot_mode,
            "full_autonomy_maneuver": self.full_autonomy_maneuver,
            "speed": self.speed,
            "velocity_vector": (self.velocity_vector[0], self.velocity_vector[1]),
            "bearing": self.desired_heading, "heading": self.heading,
            "true_wind_speed": self.true_wind_speed, "true_wind_angle": self.true_wind_angle,
            "apparent_wind_speed": self.apparent_wind_speed, "apparent_wind_angle": self.apparent_wind_angle,
            "sail_angle": self.sail_angle, "rudder_angle": self.rudder_angle,
            "current_waypoint_index": self.cur_waypoint_index,
            "current_route": [(waypoint.latitude, waypoint.longitude) for waypoint in self.waypoints_list],
            "parameters": self.autopilot_parameters
        }

        requests.post(url=TELEMETRY_SERVER_URL + "api/", json={"s1": json.dumps(telemetry_dict), "s2": f"{self.time}"}).text
        self.time+=1



    
    def get_raw_response(self):
        try:
            return json.loads(requests.get(TELEMETRY_SERVER_URL + "api/wp/one", timeout=constants.TIMEOUT_TIME_LIMIT).text)
        except Exception as e:
            print(e)
            time.sleep(1)
            print("retrying connection")
            return self.get_raw_response()
        
        
    def update_autopilot_parameters_from_telemetry(self):
        raw_response = self.get_raw_response()
        
        print(raw_response)
        if raw_response == None: 
            return
        else: 
            requests.delete(TELEMETRY_SERVER_URL + "api/wp/deleteAll")
    
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
        
        self.waypoints_list_publisher.publish(waypoint_list)
        self.autopilot_parameters_publisher.publish(parameters)


def main():
    rclpy.init()
    telem_node = TelemetryNode()
    rclpy.spin(telem_node)
    


if __name__ == "__main__": main()