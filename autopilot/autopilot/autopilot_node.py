# TODO VERY IMPORTANT: 
#   REFACTOR THE WIND SENSOR LISTENER HERE AND THE PUBLISHER IN THE SIM TO BE COMPATIBLE WITH THE WIND DATA TYPE IN SAILBOT_MSGS
#   FIND A WAY TO MAKE THE WIND DIRECTION RELATIVE TO TRUE NORTH

# TODO: Find a way to set the destination pos

from .autopilot import SailbotAutoController
from .autopilot_util import *


import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from sailbot_msgs.msg import WaypointList, AutopilotParameters
from std_msgs.msg import Float32, Bool, String, Int32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix



class AutopilotNode(Node):
    """
    Handles communications between the autopilot and all of the other nodes/ topics through ros2
    The autopilot takes in a route (list of gps positions that represent waypoints) and just traverses through the route by publishing to the mast angle and rudder angle topics.
    
    You may disable the autopilot by publishing to the /disable_autopilot topic. Otherwise, the autopilot will always publishes to the mast and rudder
    
    NOTE: All units are in standard SI units and angle is measured in degrees
    """
    
    def __init__(self):
        # NOTE: All units are in standard SI units and angle is measured in degrees
        
        
        super().__init__("autopilot")

        self.autonomous_controller = SailbotAutoController()

        self.create_timer(1 / constants.autopilot_refresh_rate, self.update_ros_topic)

        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        
        self.autopilot_parameters_listener = self.create_subscription(AutopilotParameters, '/autopilot_parameters', callback=self.autopilot_parameters_callback, qos_profile=10)
        self.autopilot_mode_listener = self.create_subscription(String, '/autopilot_mode', callback=self.autopilot_mode_callback, qos_profile=sensor_qos_profile)
        self.full_autonomy_maneuver_publisher = self.create_publisher(msg_type=String, topic='/full_autonomy_maneuver', qos_profile=sensor_qos_profile)
        
        self.desired_heading_publisher = self.create_publisher(Float32, '/desired_heading', qos_profile=10)
        self.desired_route_publisher = self.create_publisher(WaypointList, '/desired_route', qos_profile=10)
        self.desired_route_listener = self.create_subscription(WaypointList, '/desired_route', self.desired_route_callback, 10)
        self.current_waypoint_publisher = self.create_publisher(Int32, '/cur_waypoint_index', 10)
        
        self.position_listener = self.create_subscription(msg_type=NavSatFix, topic="/gps_data/position", callback=self.position_callback, qos_profile=sensor_qos_profile)
        self.velocity_listener = self.create_subscription(msg_type=Vector3, topic="/gps_data/velocity", callback=self.velocity_callback, qos_profile=sensor_qos_profile)
        self.heading_listener = self.create_subscription(msg_type=Float32, topic="/gps_data/heading", callback=self.heading_callback, qos_profile=sensor_qos_profile)

        self.apparent_wind_vector_listener = self.create_subscription(msg_type=Vector3, topic="/apparent_wind_vector", callback=self.apparent_wind_vector_callback, qos_profile=sensor_qos_profile)
        
        self.mast_angle_publisher = self.create_publisher(msg_type=Float32, topic="/actions/mast_angle", qos_profile=sensor_qos_profile)
        self.rudder_angle_publisher = self.create_publisher(msg_type=Float32, topic="/actions/rudder_angle", qos_profile=sensor_qos_profile)

        # self.disable_autopilot_listener = self.create_subscription(msg_type=Bool, topic="/disable_autopilot", callback=self.disable_autopilot_callback, qos_profile=10)
        # self.no_rudder_control_listener = self.create_subscription(msg_type=Bool, topic="/no_rudder_control", callback=self.no_rudder_control_callback, qos_profile=10)
        
        self.termination_publisher = self.create_publisher(msg_type=Bool, topic="/should_terminate", qos_profile=10)


        # Default values
        self.position = Position(longitude=0., latitude=0.)
        self.velocity = np.array([0., 0.])
        self.speed = 0.
        self.heading = 0.
        self.apparent_wind_vector = np.array([0., 0.])
        self.mast_angle = 0.
        self.rudder_angle = 0.
        
        self.autopilot_mode = ""
        self.full_autonomy_maneuver = ""
        self.heading_to_hold = 0.
    
    

    def autopilot_mode_callback(self, mode: String):
        if AutopilotMode[mode.data] == AutopilotMode.Hold_Heading and self.autopilot_mode != AutopilotMode.Hold_Heading:
            self.heading_to_hold = self.heading
            
        if AutopilotMode[mode.data] == AutopilotMode.Hold_Heading_And_Best_Sail and self.autopilot_mode != AutopilotMode.Hold_Heading_And_Best_Sail:
            self.heading_to_hold = self.heading
        
        self.autopilot_mode = AutopilotMode[mode.data]
        

        
    def autopilot_parameters_callback(self, parameters: AutopilotParameters):
        constants.waypoint_accuracy = parameters.waypoint_accuracy
        constants.no_sail_zone_size = parameters.no_sail_zone_size
        constants.perform_forced_jibe_instead_of_tack = parameters.forced_jibe_only
        constants.tack_distance = parameters.tack_distance
        
        lookup_wind_angles = parameters.sail_lookup_table_wind_angles
        lookup_sail_positions = parameters.sail_lookup_table_sail_positions
        
        constants.lookup_table = {lookup_wind_angles[i]: lookup_sail_positions[i] for i in range(len(lookup_wind_angles))}
        
        
        self.autonomous_controller.rudder_pid_controller.Kp = parameters.rudder_p_gain
        constants.rudder_p_gain = parameters.rudder_p_gain
        
        self.autonomous_controller.rudder_pid_controller.Ki = parameters.rudder_i_gain
        constants.rudder_i_gain = parameters.rudder_i_gain
        
        self.autonomous_controller.rudder_pid_controller.Kd = parameters.rudder_d_gain
        constants.rudder_d_gain = parameters.rudder_d_gain
        
        self.autonomous_controller.rudder_pid_controller.n = parameters.rudder_n_gain
        constants.rudder_n_gain = parameters.rudder_n_gain
        
        
        
    def desired_route_callback(self, waypoints: WaypointList):
        """
        convert the list of Nav Sat Fix objects (ros2) to a list of Position objects, which are a custom datatype that has some useful helper methods.
        The Position object should be simpler to do calculations with
        """
        if len(waypoints.waypoints) == 0: return
        
        print(waypoints)
        self.autonomous_controller.reset()
        
        gps_positions: list[NavSatFix] = waypoints.waypoints
        route_list = []
        
        for gps_position in gps_positions:
            route_list.append(Position(gps_position.longitude, gps_position.latitude))
            
        self.autonomous_controller.set_new_route(route_list)


    def position_callback(self, position: NavSatFix):
        self.position = Position(longitude=position.longitude, latitude=position.latitude)

    def velocity_callback(self, velocity: Vector3):
        self.velocity = np.array([velocity.x, velocity.y])
        self.speed = np.sqrt(velocity.x**2 + velocity.y**2)

    def heading_callback(self, heading: Float32):
        self.heading = heading.data
    
    def apparent_wind_vector_callback(self, apparent_wind_vector: Vector3):
        self.apparent_wind_vector = np.array([apparent_wind_vector.x, apparent_wind_vector.y])
        
        
    # def no_rudder_control_callback(self, no_rudder_control: Bool):
    #     self.no_rudder_control = no_rudder_control.data
        
    # def disable_autopilot_callback(self, is_autopilot_disabled: Bool):
    #     self.is_autopilot_disabled = is_autopilot_disabled.data
    #     if self.is_autopilot_disabled:
    #         self.desired_route_publisher.publish(WaypointList(waypoints=[]))
    #         self.autonomous_controller.reset()
            
        
        
    def send_termination_message(self):
        """
        Send a message to all of the other nodes that gives them the signal to shutdown and clean up.
        """
        self.termination_publisher.publish(Bool(data=True))




    def step(self):
        """
        Computes the best mast and rudder angles for the given mode and state
        
        Returns (tuple): (mast_angle, rudder_angle)
            mast angle or rudder angle are None if the autopilot doesn't have authority over them 
        """
        print(self.autopilot_mode)
        if self.autopilot_mode == AutopilotMode.Waypoint_Mission and self.autonomous_controller.route != None:
            mast_angle, rudder_angle = self.autonomous_controller.run_waypoint_mission_step(self.position, self.velocity, self.heading, self.apparent_wind_vector)
        
        elif self.autopilot_mode == AutopilotMode.Hold_Best_Sail:
            mast_angle, rudder_angle = self.autonomous_controller.run_hold_best_sail_step(self.apparent_wind_vector)
            
        elif self.autopilot_mode == AutopilotMode.Hold_Heading:
            mast_angle, rudder_angle = self.autonomous_controller.run_hold_heading_step(self.heading, self.heading_to_hold)
            
        elif self.autopilot_mode == AutopilotMode.Hold_Heading_And_Best_Sail:
            mast_angle, rudder_angle = self.autonomous_controller.run_hold_sail_and_rudder_step(self.apparent_wind_vector, self.heading, self.heading_to_hold)
    
        else: 
            return None, None
        
            
        return rudder_angle, mast_angle


    def update_ros_topic(self):
        """
        Updates the mast_angle and rudder_angle topics based on the output of stepping in the autopilot controller
        """

        desired_rudder_angle, desired_mast_angle = self.step()
        
        
        if self.autopilot_mode == AutopilotMode.Waypoint_Mission:
            self.full_autonomy_maneuver_publisher.publish(String(data=self.autonomous_controller.current_state.name))
        else:
            self.full_autonomy_maneuver_publisher.publish(String(data="N/A"))

        self.current_waypoint_publisher.publish(Int32(data=self.autonomous_controller.cur_waypoint_index))
        
        
        if self.autopilot_mode == AutopilotMode.Hold_Heading or self.autopilot_mode == AutopilotMode.Hold_Heading_And_Best_Sail:
            self.desired_heading_publisher.publish(Float32(data=float(self.heading_to_hold)))
            
        elif self.autopilot_mode == AutopilotMode.Waypoint_Mission and self.autonomous_controller.route != None:
            current_waypoint = self.autonomous_controller.route.waypoints[self.autonomous_controller.cur_waypoint_index]
            bearing_to_waypoint = get_bearing(self.position, current_waypoint) #TODO
            self.desired_heading_publisher.publish(Float32(data=float(bearing_to_waypoint)))
            
        else:
            self.desired_heading_publisher.publish(Float32(data=0.))
            
            
        if desired_rudder_angle != None:
            self.rudder_angle_publisher.publish(Float32(data=float(desired_rudder_angle)))
            
        if desired_mast_angle != None:
            self.mast_angle_publisher.publish(Float32(data=float(desired_mast_angle)))
        


def main():
    rclpy.init()
    autopilot_node = AutopilotNode()
    rclpy.spin(autopilot_node)


if __name__ == "__main__": main()