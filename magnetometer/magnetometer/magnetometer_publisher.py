#!/usr/bin/python3

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from serial.tools import list_ports
from rclpy.node import Node
from std_msgs.msg import Float32
from .globals import *

import serial
import time


# ARDUNIO_NANO_VID = 0x1a86
# ARDUNIO_NANO_PID = 0x7523
ARDUINO_UNO_VID = 0x2341
ARDUINO_UNO_PID = 0x0043
BAUD_RATE = 115200

def getPort(vid, pid) -> str:
    device_list = list_ports.comports()
    for device in device_list:
        print(device.serial_number)
        if device.vid == vid and device.pid == pid:
            return device.device
    raise OSError('Device not found')


class MagnetometerPublisher(Node):

    def __init__(self):
        super().__init__('magnetometer_publisher')
        
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.heading_publisher = self.create_publisher(Float32, '/gps_data/heading', sensor_qos_profile)

        self.create_timer(0.2, self.publish)
        
        self.sensor_serial = None
        try:
            serial_port = getPort(ARDUINO_UNO_VID, ARDUINO_UNO_PID)
            self.sensor_serial = serial.Serial(serial_port, baudrate=BAUD_RATE, timeout=1)
        except:
            self.on_disconnect()
            
        self.last_time = time.time()
    
    def on_disconnect(self):
        """
        uses the fact that this is a mutually exclusive callback group to block subsequent timer_callback calls
        see the following: https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html#basics-of-callback-groups
        """
        
        disconnected = True
        while disconnected:
            try:
                serial_port = getPort(ARDUINO_UNO_VID, ARDUINO_UNO_PID)
                self.sensor_serial = serial.Serial(serial_port, BAUD_RATE)
                disconnected = False
            except:
                if self.sensor_serial:
                    self.sensor_serial.close()
                print("Reconnecting...")
                time.sleep(1)
                

    def publish(self):
        try:
            heading_messages = self.sensor_serial.read_all().strip().decode().splitlines()
        except Exception as e:
            print(e)
            self.on_disconnect()
            return
        
        if not heading_messages: return
        if len(heading_messages) < 2: return
        
        heading_message = heading_messages[-2]
        
        heading = heading_message.removeprefix("Compass Heading: ")
        if heading == heading_message: return
        
        # I hate this maybe fix it in the future
        try: float(heading)
        except: return
        
        magnetic_heading_cw_north = (float(heading) + MAGNETOMETER_CALIBRATION) % 360
        true_heading_cw_north = (float(heading) + MAGNETOMETER_CALIBRATION - MAGNETIC_DECLINATION) % 360
        
        magnetic_heading_ccw_east = (90 - magnetic_heading_cw_north) % 360
        true_heading_ccw_east = (90 - true_heading_cw_north) % 360
        
        self.heading_publisher.publish(Float32(data=true_heading_ccw_east))
        
        print(f"magnetic heading (cw from north): {magnetic_heading_cw_north}")
        print(f"true heading (cw from north): {true_heading_cw_north}")
        print(f"magnetic heading (ccw from east): {magnetic_heading_ccw_east}")
        print(f"true heading (ccw from east): {true_heading_ccw_east}")
        print(f"delta_time: {time.time() - self.last_time}")
        print()
        
        self.last_time = time.time()
        
            

    def __del__(self):
        self.sensor_serial.close()
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    magnetometer_publisher = MagnetometerPublisher()
    rclpy.spin(magnetometer_publisher)


if __name__ == '__main__':
    main()
