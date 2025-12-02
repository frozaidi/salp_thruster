#!/usr/bin/env python3
import sys
import tty
import termios
import atexit
from dynamixel_sdk import *  # Uses Dynamixel SDK library


class TriDynamixelTeleop:
    def __init__(self):
        # ---------- USER CONFIG ----------
        self.DEVICENAME = '/dev/ttyUSB0'  # <- change if needed on the Pi
        self.BAUDRATE = 57600

        # Dynamixel IDs
        self.DXL_IDS = [1, 2, 3]

        # “Full speed” in Dynamixel velocity units (≈0.229 rpm per LSB on X-series)
        # Adjust this to a safe value for your setup.
        self.FULL_VEL = 200
        # ----------------------------------

        self.PROTOCOL_VERSION = 2.0

        # Control table addresses (X-series, Protocol 2.0)
        self.ADDR_TORQUE_ENABLE   = 64
        self.ADDR_OPERATING_MODE  = 11
        self.ADDR_GOAL_VELOCITY   = 104

        # Operating mode values
        self.VELOCITY_MODE        = 1

        # Data values
        self.TORQUE_ENABLE        = 1
        self.TORQUE_DISABLE       = 0

        # Setup SDK Port & Packet handler
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if not self.portHandler.openPort():
            print("Failed to open the port")
            sys.exit(1)

        # Set baudrate
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            print("Failed to set baudrate")
            sys.exit(1)

        # Set up motors
        for dxl_id in self.DXL_IDS:
            self._set_operating_mode(dxl_id, self.VELOCITY_MODE)
            self._enable_torque(dxl_id, True)

        # On exit, clean up
        atexit.register(self.shutdown)

    # --------------- Low-level helpers ---------------
    def _dxl_comm_ok(self, dxl_comm_result, dxl_error):
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return False
        return True

    def _set_operating_mode(self, dxl_id, mode):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, dxl_id, self.ADDR_OPERATING_MODE, mode
        )
        if not self._dxl_comm_ok(dxl_comm_result, dxl_error):
            print(f"Failed to set operating mode for ID {dxl_id}")

    def _enable_torque(self, dxl_id, enable=True):
        val = self.TORQUE_ENABLE if enable else self.TORQUE_DISABLE
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, val
        )
        if not self._dxl_comm_ok(dxl_comm_result, dxl_error):
            print(f"Failed to {'enable' if enable else 'disable'} torque for ID {dxl_id}")

    def _set_velocity_raw(self, dxl_id, vel):
        """
        vel: signed integer in Dynamixel velocity units.
             Positive = CCW, negative = CW (depends on your mounting).
        """
        # Convert signed 32-bit to unsigned representation for the SDK
        if vel < 0:
            vel = (1 << 32) + vel

        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, dxl_id, self.ADDR_GOAL_VELOCITY, int(vel)
        )
        if not self._dxl_comm_ok(dxl_comm_result, dxl_error):
            print(f"Failed to set velocity for ID {dxl_id}")

    # --------------- High-level commands ---------------
    def set_velocity(self, dxl_id, direction):
        """
        direction: +1 for forward, -1 for backward, 0 for stop
        """
        if direction == 0:
            vel = 0
        else:
            vel = direction * self.FULL_VEL

        self._set_velocity_raw(dxl_id, vel)
        print(f"ID {dxl_id} -> vel {vel}")

    def stop_all(self):
        for dxl_id in self.DXL_IDS:
            self._set_velocity_raw(dxl_id, 0)
        print("All motors stopped.")

    # --------------- Keyboard handling ---------------
    @staticmethod
    def getch():
        """
        Reads a single character from stdin (no Enter needed).
        Works in a terminal over SSH.
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def print_help(self):
        print("------ Tri Dynamixel Teleop ------")
        print("Forward : q (ID 1), w (ID 2), e (ID 3)")
        print("Backward: a (ID 1), s (ID 2), d (ID 3)")
        print("Space   : stop all motors")
        print("Ctrl+C  : exit (also stops and disables torque)")
        print("----------------------------------")

    def run(self):
        self.print_help()

        # Key bindings:
        # qwe = forward, asd = backward
        keymap = {
            'q': (1, +1),
            'a': (1, -1),
            'w': (2, +1),
            's': (2, -1),
            'e': (3, +1),
            'd': (3, -1),
            ' ': ('all', 0),  # stop all
        }

        try:
            while True:
                ch = self.getch()
                if ch in keymap:
                    target, direction = keymap[ch]
                    if target == 'all':
                        self.stop_all()
                    else:
                        self.set_velocity(target, direction)
                else:
                    # Ignore other keys; you can add more functions here if you like.
                    pass

        except KeyboardInterrupt:
            print("\nKeyboardInterrupt detected. Exiting teleop...")

    # --------------- Shutdown ---------------
    def shutdown(self):
        # Safely stop and disable torque on all motors
        print("Shutting down: stopping motors and disabling torque.")
        self.stop_all()
        for dxl_id in self.DXL_IDS:
            self._enable_torque(dxl_id, False)
        self.portHandler.closePort()


if __name__ == "__main__":
    teleop = TriDynamixelTeleop()
    teleop.run()
