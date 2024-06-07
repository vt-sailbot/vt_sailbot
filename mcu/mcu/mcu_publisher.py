import time
import serial
from serial.tools import list_ports

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32


TEENSY_VID = 0x16c0
TEENSY_PID = 0x0483
BAUD_RATE = 115200

# TEST_ARDUINO_VID = 0x2341
# TEST_ARDUINO_PID = 0x0043


def getPort(vid, pid) -> str:
    device_list = list_ports.comports()
    for device in device_list:
        if device.vid == vid and device.pid == pid:
            return device.device
    raise OSError('Device not found')


class MCUPublisher(Node):
    
    def __init__(self):
        super().__init__("mcu_publisher")
        
        self.create_timer(0.05, self.update_mcu)

        self.mcu_serial = None
        try:
            mcu_serial_port = getPort(TEENSY_VID, TEENSY_PID)
            self.mcu_serial = serial.Serial(mcu_serial_port, BAUD_RATE)
        except:
            self.on_disconnect()
            
        
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.mast_angle_publisher = self.create_subscription(msg_type=Float32, topic="/actions/mast_angle", callback=self.mast_angle_callback, qos_profile=sensor_qos_profile)
        self.rudder_angle_publisher = self.create_subscription(msg_type=Float32, topic="/actions/rudder_angle", callback=self.rudder_angle_callback, qos_profile=sensor_qos_profile)
        
        self.mast_angle = 0.
        self.rudder_angle = 0.
        
    def on_disconnect(self):
        """
        uses the fact that this is a mutually exclusive callback group to block subsequent timer_callback calls
        see the following: https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html#basics-of-callback-groups
        """
        
        disconnected = True
        while disconnected:
            try:
                serial_port = getPort(TEENSY_VID, TEENSY_PID)
                self.mcu_serial = serial.Serial(serial_port, BAUD_RATE)
                disconnected = False
            except:
                if self.mcu_serial:
                    self.mcu_serial.close()
                print("Reconnecting...")
                time.sleep(1)


    def mast_angle_callback(self, mast_angle: Float32):
        self.mast_angle = mast_angle.data
        
    def rudder_angle_callback(self, rudder_angle: Float32):
        # print(f"rudder angle callback: {rudder_angle}")
        self.rudder_angle = rudder_angle.data

    def update_mcu(self):
        try:
            self.mcu_serial.write(f'mast angle: {self.mast_angle}; rudder angle: {self.rudder_angle}\n'.encode('ascii'))
            print(f"Received: {self.mcu_serial.read_all().decode('ascii')}")
        except Exception as e:
            print(e)
            self.on_disconnect()
            return
            
        
        
def main():
    rclpy.init()
    mcu_publisher = MCUPublisher()
    rclpy.spin(mcu_publisher)

if __name__ == "__main__": main()