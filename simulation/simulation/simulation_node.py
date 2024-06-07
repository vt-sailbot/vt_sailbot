#!usr/bin/python3

# TODO: Implement a graceful way to handle simulation termination with the control scripts

import numpy as np, cv2
import random
import gymnasium as gym
from gymnasium.wrappers.time_limit import TimeLimit
from gymnasium.wrappers.record_video import RecordVideo

from sailboat_gym import CV2DRenderer, Observation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from sailbot_msgs.msg import WaypointList
import navpy


# this is the wind direction measured as (what seems like) counter clockwise from true east
WIND_DIRECTION = np.deg2rad(-90)
WIND_SPEED = 3

sim_time = 0

# def generate_wind(_): 
#     return np.array([np.cos(WIND_DIRECTION), np.sin(WIND_DIRECTION)]) * WIND_SPEED * random.gauss(sigma=0.7)
def generate_wind(_): 
    # wind_direction = 0.4 * np.sin(0 * np.deg2rad(sim_time)) + WIND_DIRECTION
    return np.array([np.cos(WIND_DIRECTION) + random.random()*0.2, np.sin(WIND_DIRECTION) + random.random()*0.2]) * WIND_SPEED

def up_wind_generator(_):
    if sim_time >= 150:
        return np.array([np.cos(np.deg2rad(180)), np.sin(np.deg2rad(180))]) * WIND_SPEED
    return np.array([np.cos(0), np.sin(0)]) * WIND_SPEED


class SimNode(Node):

    def __init__(self):
        global sim_time

        super().__init__("simulation")
        
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # NOTE: All units are in standard SI units and angle is measured in degrees
        self.desired_route_listener = self.create_subscription(WaypointList, '/desired_route', self.desired_route_callback, 10)
        
        self.position_publisher = self.create_publisher(msg_type=NavSatFix, topic="/gps_data/position", qos_profile=sensor_qos_profile)
        self.velocity_publisher = self.create_publisher(msg_type=Vector3, topic="/gps_data/velocity", qos_profile=sensor_qos_profile)
        self.heading_publisher = self.create_publisher(msg_type=Float32, topic="/gps_data/heading", qos_profile=sensor_qos_profile)
        self.apparent_wind_vector_publisher = self.create_publisher(msg_type=Vector3, topic="/apparent_wind_vector", qos_profile=sensor_qos_profile)
        
        self.rudder_angle_listener = self.create_subscription(msg_type=Float32, topic="/actions/rudder_angle", callback=self.rudder_angle_callback, qos_profile=sensor_qos_profile)
        self.mast_angle_listener = self.create_subscription(msg_type=Float32, topic="/actions/mast_angle", callback=self.mast_angle_callback, qos_profile=sensor_qos_profile)
        
        self.termination_listener = self.create_subscription(msg_type=Bool, topic="/should_terminate", callback=self.should_terminate_callback, qos_profile=10)
        
        self.env = gym.make('SailboatLSAEnv-v0',
            renderer=CV2DRenderer(),
            wind_generator_fn=generate_wind, 
            video_speed=20,
            map_scale=0.1,
            keep_sim_alive=True
        )
        
        self.episode_length = self.env.NB_STEPS_PER_SECONDS * 60 * 400
        sim_time = 0

        self.env = TimeLimit(self.env, max_episode_steps=self.episode_length)
        self.env = RecordVideo(self.env, video_folder='./output/videos/')

        obs, info = self.env.reset(seed=10)
        
        self.display_image(self.env.render())
        self.publish_observation_data(obs)
        
        self.desired_rudder_angle, self.desired_mast_angle = None, None
        self.apparent_wind_vector = Vector3(x=1.)
        self.true_wind_vector = Vector3(x=1.)
        self.route = None



    def __del__(self):
        self.env.close()
        rclpy.shutdown()



    def desired_route_callback(self, waypoint_list: WaypointList):
        self.route = [(waypoint.longitude, waypoint.latitude) for waypoint in waypoint_list.waypoints]
        
        
    def rudder_angle_callback(self, msg: Float32):
        self.desired_rudder_angle = np.array(msg.data)

        if self.desired_mast_angle != None:
            self.step_simulation()
            self.desired_rudder_angle, self.desired_mast_angle = None, None

    def mast_angle_callback(self, msg: Float32):
        
        _, true_wind_angle = self.cartesian_vector_to_polar(self.true_wind_vector.x, self.true_wind_vector.y)
        mast_dir_fix = -1 if 0 < true_wind_angle < 180 else 1
        self.desired_mast_angle = np.array(mast_dir_fix * msg.data)

        if self.desired_rudder_angle != None:
            self.step_simulation()
            self.desired_rudder_angle, self.desired_mast_angle = None, None
            
            
    def should_terminate_callback(self, msg: Bool):
        if msg.data == False: return
        
        self.env.close()
        rclpy.shutdown()
        
        
    def publish_observation_data(self, obs: Observation):

        # converts the position from local ned to longitude, latitude, altitude
        position = Vector3(x=obs["p_boat"][0].item(), y=obs["p_boat"][1].item(), z=-obs["p_boat"][2].item())
        gps_position = NavSatFix()
        
        # TODO: lat ref could be: 37.229572, lon ref: -80.413940
        latitude, longitude, _ = navpy.ned2lla([position.y, position.x, position.z], lat_ref=0., lon_ref=0., alt_ref=0)
        gps_position.latitude = latitude
        gps_position.longitude = longitude
        
        # saves the current velocity vector and speed 
        boat_velocity_vector = Vector3(x=obs["dt_p_boat"][0].item(), y=obs["dt_p_boat"][1].item(), z=obs["dt_p_boat"][2].item())
        boat_speed = Float32(data=np.sqrt(boat_velocity_vector.x**2 + boat_velocity_vector.y**2 + boat_velocity_vector.z**2))

        roll, pitch, yaw = obs["theta_boat"]
        # standard heading calculations from pitch, yaw, and roll
        
        heading_vector = np.array([np.cos(yaw) * np.cos(pitch), np.sin(yaw) * np.cos(pitch), np.sin(pitch)])
        heading_vector = Vector3(x = heading_vector[0].item(), y = heading_vector[1].item(), z = heading_vector[2].item())
        
        _, heading_angle = self.cartesian_vector_to_polar(heading_vector.x, heading_vector.y)
        heading_angle = Float32(data=heading_angle)

        # calculates the magnitude and direction of the wind velocity both true and apparent
        # Approximately 7:30 was most useful for me: https://www.youtube.com/watch?v=ndL1FcTRPwU&t=472s&ab_channel=BasicCruisingwithOwen
        # https://en.wikipedia.org/wiki/Apparent_wind#/media/File:DiagramApparentWind.png 
        # Apparent Wind = True Wind + Velocity
        # Global refers to being measured ccw from true east and not being measured from atop the boat
        true_wind_speed, global_wind_angle = self.cartesian_vector_to_polar(obs["wind"][0].item(), obs["wind"][1].item())
        true_wind_angle = global_wind_angle - heading_angle.data
        self.true_wind_vector = Vector3(x= (true_wind_speed * np.cos(np.deg2rad(true_wind_angle))), y= (true_wind_speed * np.sin(np.deg2rad(true_wind_angle))))
        self.apparent_wind_vector = Vector3(x= (self.true_wind_vector.x - boat_velocity_vector.x), y= (self.true_wind_vector.y - boat_velocity_vector.y)) 
        
        self.position_publisher.publish(gps_position)
        self.velocity_publisher.publish(boat_velocity_vector)
        self.heading_publisher.publish(heading_angle)
        self.apparent_wind_vector_publisher.publish(self.apparent_wind_vector)
        
        print()
        print(f"true wind angle: {true_wind_angle}")
        print(f"true wind speed: {true_wind_speed}")
        print(f'current rudder angle: {np.rad2deg(obs["theta_rudder"])[0]}; current mast angle: {np.rad2deg(obs["theta_sail"])[0]}')



    def step_simulation(self):
        global sim_time

        assert self.desired_rudder_angle != None
        assert self.desired_mast_angle != None

        print(f"desired rudder angle: {self.desired_rudder_angle}; desired mast angle: {self.desired_mast_angle}")
        
        action = {"theta_rudder": np.deg2rad(self.desired_rudder_angle), "theta_sail": np.deg2rad(self.desired_mast_angle)}
        obs, reward, terminated, truncated, info = self.env.step(action)

        sim_time += 1

        if terminated or truncated: rclpy.shutdown()

        # if self.route == None: self.display_image(self.env.render())
        self.display_image(self.env.render())

        self.publish_observation_data(obs)




    def cartesian_vector_to_polar(self, x, y):
        """
            Converts a cartesian vector (x and y coordinates) to polar form (magnitude and direction).
            Outputs a tuple of magnitude and direction of the inputted vector
        """
        magnitude = np.sqrt(x**2 + y**2)
        direction = np.arctan2(y, x) # radians
        direction = direction * (180/np.pi)  # angle from -180 to 180 degrees
        direction = direction % 360  # angle from 0 to 360 degrees
        return magnitude, direction
    
    def display_image(self, img):
        cv2.imshow("Simulation Real Time", img)
        cv2.waitKey(1)
    

def main():

    rclpy.init()
    sim_node = SimNode()
    rclpy.spin(sim_node)


if __name__ == "__main__": 
    main()