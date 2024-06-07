#!usr/bin/python3

# TODO: Implement a graceful way to handle simulation termination with the control scripts

import numpy as np, cv2
import gymnasium as gym
from gymnasium.wrappers.time_limit import TimeLimit
from gymnasium.wrappers.record_video import RecordVideo
import time

from sailboat_gym import CV2DRenderer, Observation, get_best_sail

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
import navpy
from pygeodesy import utm
from pygeodesy.ellipsoidalKarney import LatLon


# this is the wind direction measured as (what seems like) counter clockwise from true east
WIND_DIRECTION = np.deg2rad(0)
WIND_SPEED = 1.5

sim_time = 0

def generate_wind(_):
    return np.array([np.cos(WIND_DIRECTION), np.sin(WIND_DIRECTION)]) * WIND_SPEED

def up_wind_generator(_):
    if sim_time >= 150:
        return np.array([np.cos(np.deg2rad(180)), np.sin(np.deg2rad(180))]) * WIND_SPEED
    return np.array([np.cos(0), np.sin(0)]) * WIND_SPEED


class SimNode(Node):

    def __init__(self):
        global sim_time


        super().__init__("simulation")

        # NOTE: All units are in standard SI units and angle is measured in degrees

        self.position_publisher = self.create_publisher(msg_type=NavSatFix, topic="/gps_data/position", qos_profile=1)

        self.speed_publisher = self.create_publisher(msg_type=Float32, topic="/gps_data/speed", qos_profile=1)
        self.heading_publisher = self.create_publisher(msg_type=Float32, topic="/gps_data/heading", qos_profile=1)

        self.wind_vector_publisher = self.create_publisher(msg_type=Vector3, topic="/wind_vector", qos_profile=1)
        
        self.rudder_angle_listener = self.create_subscription(msg_type=Float32, topic="/actions/rudder_angle", callback=self.rudder_angle_callback, qos_profile=1)
        self.mast_angle_listener = self.create_subscription(msg_type=Float32, topic="/actions/mast_angle", callback=self.mast_angle_callback, qos_profile=1)
        
        self.termination_listener = self.create_subscription(msg_type=Bool, topic="/should_terminate", callback=self.should_terminate_callback, qos_profile=10)
        
        self.relative_wind_direction = 0
        
        self.env = gym.make('SailboatLSAEnv-v0',
            renderer=CV2DRenderer(),
            wind_generator_fn=generate_wind,
            video_speed=20,
            map_scale=0.1,
            keep_sim_alive=True
        )
        
        
    
        self.episode_length = self.env.NB_STEPS_PER_SECONDS * 60 * 1000

        self.env = TimeLimit(self.env, max_episode_steps=self.episode_length)
        self.env = RecordVideo(self.env, video_folder='./output/videos/')

        obs, info = self.env.reset(seed=10)
        
        self.display_image(self.env.render())
        
        self.publish_observation_data(obs)

        self.sail_angle = get_best_sail('SailboatLSAEnv-v0', WIND_DIRECTION)

        self.desired_rudder_angle, self.desired_mast_angle = None, None

        sim_time = 0


    def __del__(self):
        self.env.close()
        rclpy.shutdown()


    
    def rudder_angle_callback(self, msg: Float32):
        self.desired_rudder_angle = np.array(msg.data)

        if self.desired_mast_angle != None:
            self.step_simulation()
            self.desired_rudder_angle, self.desired_mast_angle = None, None

    def mast_angle_callback(self, msg: Float32):
        
        mast_dir_fix = -1 if 0 < self.relative_wind_direction < 180 else 1
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
        velocity = Vector3(x=obs["dt_p_boat"][0].item(), y=obs["dt_p_boat"][1].item(), z=obs["dt_p_boat"][2].item())
        speed = Float32(data=np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2))

        roll, pitch, yaw = obs["theta_boat"]
        # standard heading calculations from pitch, yaw, and roll
        heading = [np.cos(yaw) * np.cos(pitch), np.sin(yaw) * np.cos(pitch), np.sin(pitch)]
        heading = Vector3(x = heading[0].item(), y = heading[1].item(), z = heading[2].item())
        _, heading = self.cartesian_vector_to_polar(heading.x, heading.y)

        heading = Float32(data=heading)

        # calculates the magnitude and direction of the wind_velocity
        wind_velocity = Vector3(x=obs["wind"][0].item(), y=obs["wind"][1].item(), z=0.)
        
        # This is not necessary anymore as we are publishing x and y components instead of magnitude and velocity
        wind_speed, wind_direction = self.cartesian_vector_to_polar(wind_velocity.x, wind_velocity.y)
        
        # this is stored because it is useful to calculate the desired mast angle since whether the sail is on the left or right of the boat depends on where the wind is blowing from
        # TODO: make this not necessary anymore
        self.relative_wind_direction = (wind_direction - heading.data) % 360   
        
        self.position_publisher.publish(gps_position)
        self.speed_publisher.publish(speed)
        self.heading_publisher.publish(heading)
        self.wind_vector_publisher.publish(wind_velocity)
        
        print()
        print(f'current rudder angle: {np.rad2deg(obs["theta_rudder"])[0]}; current mast angle: {np.rad2deg(obs["theta_sail"])[0]}')


    def step_simulation(self):
        global sim_time

        assert self.desired_rudder_angle != None
        assert self.desired_mast_angle != None

        print(f"desired rudder angle: {self.desired_rudder_angle}; desired mast angle: {self.desired_mast_angle}")
        
        # print()
        action = {"theta_rudder": np.deg2rad(self.desired_rudder_angle), "theta_sail": np.deg2rad(self.desired_mast_angle)}
        obs, reward, terminated, truncated, info = self.env.step(action)

        sim_time += 1

        if terminated or truncated: rclpy.shutdown()

        self.display_image(self.env.render())

        self.publish_observation_data(obs)


    # HELPER METHODS
    def cartesian_vector_to_polar(self, x, y):
        """
            Converts a cartesian vector (x and y coordinates) to polar form (magnitude and direction).
            Outputs a tuple of magnitude and direction of the inputted vector
        """
        magnitude = np.sqrt(x**2 + y**2)
        direction = np.arctan2(y, x) # radians
        direction = direction * (180/np.pi)  # angle from -180 to 180 degrees
        direction = (direction + 180) % 360  # angle from 0 to 360 degrees
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