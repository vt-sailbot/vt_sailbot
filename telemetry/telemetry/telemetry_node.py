#!usr/bin/python3

from .telemetry_utils import *
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, Bool, String, Int32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from sailbot_msgs.msg import WaypointList, AutopilotParameters
import requests
import numpy as np
import json
import array

TELEMETRY_SERVER_URL = 'http://107.23.136.207:8082/'

# TODO: MOST IMPORTANT MAKE THE IP AN ARGUMENT INTO THE ROS NODE AND DOCKER CONTAINER

# TODO: Update the website info at a fixed rate
# TODO: add current state of the board and the FSM

class TelemetryNode(Node):

    def __init__(self):
        super().__init__("telemetry")

        # NOTE: All units are in standard SI units and angle is measured in degrees
        self.create_timer(0.2, self.update_website_telemetry)

        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.autopilot_parameter_listener = self.create_subscription(AutopilotParameters, '/autopilot_parameters', callback=self.autopilot_parameters_callback, qos_profile=10)
        
        self.desired_heading_listener = self.create_subscription(Float32, '/desired_heading', self.desired_heading_callback, 10)
        self.desired_route_listener = self.create_subscription(WaypointList, '/desired_route', self.desired_route_callback, 10)
        self.current_waypoint_listener = self.create_subscription(Int32, '/cur_waypoint_index', self.cur_waypoint_index_callback, 10)
        
        self.full_autonomy_maneuver_listener = self.create_subscription(msg_type=String, topic="/full_autonomy_maneuver", callback=self.full_autonomy_maneuver_callback, qos_profile=sensor_qos_profile)
        self.autopilot_mode_listener = self.create_subscription(msg_type=String, topic="/autopilot_mode", callback=self.autopilot_mode_callback, qos_profile=sensor_qos_profile)
        
        self.position_listener = self.create_subscription(msg_type=NavSatFix, topic="/gps_data/position", callback=self.position_callback, qos_profile=sensor_qos_profile)
        self.velocity_listener = self.create_subscription(msg_type=Vector3, topic="/gps_data/velocity", callback=self.velocity_callback, qos_profile=sensor_qos_profile)
        self.heading_listener = self.create_subscription(msg_type=Float32, topic="/gps_data/heading", callback=self.heading_callback, qos_profile=sensor_qos_profile)
        self.apparent_wind_vector_listener = self.create_subscription(msg_type=Vector3, topic="/apparent_wind_vector", callback=self.apparent_wind_vector_callback, qos_profile=sensor_qos_profile)
        
        self.mast_angle_listener = self.create_subscription(msg_type=Float32, topic="/actions/mast_angle", callback=self.mast_angle_callback, qos_profile=sensor_qos_profile)
        self.rudder_angle_listener = self.create_subscription(msg_type=Float32, topic="/actions/rudder_angle", callback=self.rudder_angle_callback, qos_profile=sensor_qos_profile)

        self.termination_listener = self.create_subscription(msg_type=Bool, topic="/should_terminate", callback=self.should_terminate_callback, qos_profile=10)

        # Default values in case these are never sent through ROS
        # If these values aren't changing then the ros node thats supposed to be sending these values may not be working correctly
        self.current_route: list[NavSatFix] = []
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

        self.mast_angle = 0.
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
        
        
    def desired_route_callback(self, desired_route: WaypointList):
        self.current_route = desired_route.waypoints
    
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
        

    def mast_angle_callback(self, mast_angle: Float32):
        self.mast_angle = mast_angle.data

    def rudder_angle_callback(self, rudder_angle: Float32):
        self.rudder_angle = rudder_angle.data
    
    def should_terminate_callback(self, msg: Bool):
        if msg.data == False: return
        rclpy.shutdown()

        
    def update_website_telemetry(self):
        """
        FORMAT:
            position (Lat Lon tuple); current state; speed; bearing; heading; wind speed; wind direction; mast angle; rudder angle; cur_waypoint; the entire waypoint list for the current route
            each of them are separated as a semicolon
            tuples are denoted as: element1, element2, ... such as for latitude/ longitude
            different entries are denoted as: speed; heading; apparent_wind_speed
        """

        # if self.current_route != None and self.cur_waypoint_index < len(self.current_route):
        #     cur_waypoint = self.current_route[self.cur_waypoint_index]
        #     self.bearing = get_bearing(self.position.latitude, self.position.longitude, cur_waypoint.latitude, cur_waypoint.longitude)
        # else:
        #     self.bearing = 0
        
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
            "mast_angle": self.mast_angle, "rudder_angle": self.rudder_angle,
            "current_waypoint_index": self.cur_waypoint_index,
            "current_route": [(waypoint.latitude, waypoint.longitude) for waypoint in self.current_route],
            "parameters": self.autopilot_parameters
        }
        
        # telemetry_string = pickle.dumps(telemetry_dict)
        
#         f"{self.position.latitude}, {self.position.longitude}; {self.autopilot_mode}; \
# {format(self.speed, '.2f')}; {format(self.bearing, '.2f')}; {format(self.heading, '.2f')}; \
# {format(self.true_wind_speed, '.2f')}; {format(self.true_wind_angle, '.2f')}; \
# {format(self.apparent_wind_speed, '.2f')}; {format(self.apparent_wind_angle, '.2f')}; \
# {format(self.mast_angle, '.2f')}; {format(self.rudder_angle, '.2f')}; \
# {self.cur_waypoint.latitude}, {self.cur_waypoint.longitude}; "

        # for waypoint in self.current_route:
        #     telemetry_string += f"{waypoint.latitude}, {waypoint.longitude}; "
    
        # print(f"telemetry string: {telemetry_string}")
        
        requests.post(url=TELEMETRY_SERVER_URL + "api/", json={"s1": json.dumps(telemetry_dict), "s2": f"{self.time}"}).text
        
        self.time+=1

        


def main():
    rclpy.init()
    telem_node = TelemetryNode()
    rclpy.spin(telem_node)
    


if __name__ == "__main__": main()