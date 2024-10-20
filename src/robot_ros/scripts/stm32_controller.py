#!/usr/bin/env python3

import rospy
import serial
import sys
import tty
import termios

class STM32Controller:
    def __init__(self):
        rospy.init_node('stm32_controller', anonymous=True)
        
        # Serial port configuration
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud = rospy.get_param('~baud', 115200)
        
        self.ser = None
        self.current_command = "STOP"
        
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
    
    def getch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def check_keyboard(self, event):
        key = self.getch()
        if key == 'w':
            self.send_command("UP")
        elif key == 's':
            self.send_command("DOWN")
        elif key == 'a':
            self.send_command("LEFT")
        elif key == 'd':
            self.send_command("RIGHT")
        elif key == 'q':
            self.send_command("STOP")

        if key == '\x03':  # Ctrl+C
            rospy.signal_shutdown("Ctrl+C pressed")

    def run(self):
        if not self.connect_serial():
            return

        rate = rospy.Rate(10)  # 10Hz
        rospy.Timer(rospy.Duration(0.1), self.check_keyboard)
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
