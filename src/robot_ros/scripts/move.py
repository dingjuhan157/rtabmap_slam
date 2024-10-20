#!/usr/bin/env python3

import rospy
import serial
from geometry_msgs.msg import Twist

class STM32Controller:
    def __init__(self):
        rospy.init_node('stm32_controller', anonymous=True)
        
        # Serial port configuration
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud = rospy.get_param('~baud', 115200)
        
        self.ser = None
        self.current_command = "STOP"
        
        # Subscribe to /cmd_vel topic
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        
    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            rospy.loginfo(f"Connected to STM32F407 on {self.port} at {self.baud} baud")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port: {e}")
            return False
        return True
        
    def send_command(self, command):
        if self.ser and command != self.current_command:
            self.ser.write(command.encode() + b'\n')
            rospy.loginfo(f"Sent command: {command}")
            self.current_command = command
    
    def cmd_vel_callback(self, msg):
        # Convert Twist message to directional commands
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        if linear_x > 0.1:
            self.send_command("UP")
        elif linear_x < -0.1:
            self.send_command("DOWN")
        elif angular_z > 0.1:
            self.send_command("LEFT")
        elif angular_z < -0.1:
            self.send_command("RIGHT")
        else:
            self.send_command("STOP")

    def run(self):
        if not self.connect_serial():
            return

        rospy.spin()

    def cleanup(self):
        if self.ser:
            rospy.loginfo("Closing serial port")
            self.ser.close()

if __name__ == '__main__':
    controller = STM32Controller()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.cleanup()
