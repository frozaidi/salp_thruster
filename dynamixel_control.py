import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import csv
import time
import threading
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import atexit

class DynamixelControl:
    def __init__(self):
        # ---------- USER CONFIG ----------
        self.DEVICENAME = '/dev/tty.usbserial-FT891KEM'  # Update this to your serial port
        self.BAUDRATE = 57600
        self.DXL_ID = 1
        self.VELOCITY = 50  # Velocity in Dynamixel units (approx. RPM * 0.229)
        # ----------------------------------

        self.PROTOCOL_VERSION = 2.0

        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_OPERATING_MODE = 11
        self.ADDR_GOAL_VELOCITY = 104
        self.ADDR_PRESENT_POSITION = 132

        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.VELOCITY_CONTROL_MODE = 1

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if not self.portHandler.openPort():
            print("Failed to open port")
            quit()

        # Set baudrate
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            print("Failed to set baudrate")
            quit()

        self.set_velocity_control()

        atexit.register(self.exit_sequence)

    def set_velocity_control(self):
        # Set operating mode to velocity control
        self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE, self.VELOCITY_CONTROL_MODE)
        self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)

    def set_velocity(self):
        while True:
            vel = input("Enter desired velocity [-1023 to 1023, 0 to stop]: ")
            try:
                vel = int(vel)
                if -1023 <= vel <= 1023:
                    # Convert to unsigned 4-byte value
                    if vel < 0:
                        vel = (1 << 32) + vel  # Convert to 2's complement
                    self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_VELOCITY, vel)
                else:
                    print("Velocity out of range.")
            except ValueError:
                print("Invalid input. Enter an integer.")

    def exit_sequence(self):
        self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_VELOCITY, 0)
        self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        self.portHandler.closePort()


if __name__ == "__main__":
    DC = DynamixelControl()
    DC.set_velocity()
    # thread_vel = threading.Thread(target=DC.set_velocity, daemon=True)
    # thread_vel.start()