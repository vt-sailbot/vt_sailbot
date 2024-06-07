#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import time
import numpy as np
from collections import deque
import serial
from serial.tools import list_ports

from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3


# SERIAL_PORT = "/dev/ttyUSB0"  # temporary for now
WIND_SENSOR_VID = 0x0403
WIND_SENSOR_PID = 0x6001
WIND_SENSOR_SERIAL_NUMBER = "AB7IMXEU"
BAUD_RATE = 38400

KNOTS_TO_METERS_PER_SECOND = 0.514444


def getPort(vid, pid, serial_number) -> str:
    device_list = list_ports.comports()
    for device in device_list:
        print(device.serial_number)
        if device.vid == vid and device.pid == pid and device.serial_number == serial_number:
            return device.device
    raise OSError('Device not found')



class WindSensorPublisher(Node):

    def __init__(self):
        super().__init__("wind_sensor_publisher")
        
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.apparent_wind_vector_publisher = self.create_publisher(Vector3, '/apparent_wind_vector', sensor_qos_profile)
        self.termination_listener = self.create_subscription(msg_type=Bool, topic="/should_terminate", callback=self.should_terminate_callback, qos_profile=10)
        
        self.get_logger().info("creating wind sensor publisher")
        
        timer_period_seconds = 0.1
        self.create_timer(timer_period_seconds, self.timer_callback)

        self.sensor_serial = None
        try:
            serial_port = getPort(WIND_SENSOR_VID, WIND_SENSOR_PID, WIND_SENSOR_SERIAL_NUMBER)
            self.sensor_serial = serial.Serial(serial_port, BAUD_RATE)
        except:
            self.on_disconnect()
        
        self.apparent_wind_vector_queue = deque(maxlen=15)
    
    def on_disconnect(self):
        """
        uses the fact that this is a mutually exclusive callback group to block subsequent timer_callback calls
        see the following: https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html#basics-of-callback-groups
        """
        
        disconnected = True
        while disconnected:
            try:
                serial_port = getPort(WIND_SENSOR_VID, WIND_SENSOR_PID)
                self.sensor_serial = serial.Serial(serial_port, BAUD_RATE)
                disconnected = False
            except:
                if self.sensor_serial:
                    self.sensor_serial.close()
                print("Reconnecting...")
                time.sleep(1)
                
    
    
    def sum_integers(self, integer):
        """ Sums up all positive integers less than or equal to the number that was passed in"""
        # nothing but this formula: https://www.youtube.com/watch?app=desktop&v=bWZwF1H9YbU
        return (integer * (integer + 1))/ 2

    def linear_moving_weighted_average(self, data_tuple):
        weighted_list_1 = []
        weighted_list_2 = []
        for index, (data1, data2) in enumerate(data_tuple):
            weighted_list_1.append(data1 * (index+1))
            weighted_list_2.append(data2 * (index+1))

        length = len(data_tuple) 
        return (sum(weighted_list_1)/ self.sum_integers(length)), sum(weighted_list_2)/ self.sum_integers(length)



    def timer_callback(self):
        try:
            raw_data = self.sensor_serial.readline().decode('ascii')
        except Exception as e:
            print(e)
            self.on_disconnect()
            return
        
        split_data = raw_data.split(',')

        if len(split_data) != 6: return
        
        NMEA_encoding, apparent_wind_angle, _, apparent_wind_speed, speed_type, checksum = split_data
        
        print(f"Raw AWA: {apparent_wind_angle}")
        apparent_wind_speed = float(apparent_wind_speed) * KNOTS_TO_METERS_PER_SECOND  # given knots but want m/s
        # we are given the wind angle from the source in cw from the centerline of the boat based on the labels adam made, but we would like ccw from the centerline of the boat for where the wind is blowing to
        apparent_wind_angle = (180 - float(apparent_wind_angle)) % 360 
        
        wind_vector = (apparent_wind_speed * np.cos(np.deg2rad(apparent_wind_angle)), apparent_wind_speed * np.sin(np.deg2rad(apparent_wind_angle)))
        self.apparent_wind_vector_queue.append(wind_vector)
        filtered_apparent_wind_vector = self.linear_moving_weighted_average(self.apparent_wind_vector_queue)
        
        msg = Vector3(x=filtered_apparent_wind_vector[0], y=filtered_apparent_wind_vector[1])
        self.apparent_wind_vector_publisher.publish(msg=msg)


    def should_terminate_callback(self, msg: Bool):
        if msg.data == False: return
        rclpy.shutdown()
        
        
    def __del__(self):
        self.get_logger().info("closing wind sensor publisher")
        self.sensor_serial.close()


def main(args=None):
    rclpy.init(args=args)

    wind_sensor = WindSensorPublisher()

    rclpy.spin(wind_sensor)

    wind_sensor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
