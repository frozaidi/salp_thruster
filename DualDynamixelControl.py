import time
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import atexit

class DualDynamixelControl:
    def __init__(self):
        # ---------- USER CONFIG ----------
        self.DEVICENAME = '/dev/tty.usbserial-FT891KEM'  # Update this to your serial port
        self.BAUDRATE = 57600

        # Set your left/right motor IDs here
        self.DXL_LEFT_ID  = 1
        self.DXL_RIGHT_ID = 2

        # “Full speed” in Dynamixel velocity units (≈0.229 rpm per LSB on X-series)
        # Adjust to the safe max you want. ±1023 is common and safe for many models.
        self.FULL_VEL = 306
        # ----------------------------------

        self.PROTOCOL_VERSION = 2.0

        # Control table addresses (X-series, Protocol 2.0)
        self.ADDR_TORQUE_ENABLE     = 64
        self.ADDR_OPERATING_MODE    = 11
        self.ADDR_GOAL_VELOCITY     = 104
        self.ADDR_PRESENT_POSITION  = 132  # not used here, but kept for convenience

        # Constants
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.VELOCITY_CONTROL_MODE = 1

        # SDK setup
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        if not self.portHandler.openPort():
            raise RuntimeError("Failed to open port")
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            raise RuntimeError("Failed to set baudrate")

        # Initialize both motors
        self.ids = [self.DXL_LEFT_ID, self.DXL_RIGHT_ID]
        self._set_velocity_mode_and_enable()

        atexit.register(self.exit_sequence)

    # ---------- Low-level helpers ----------
    def _write1(self, dxl_id, addr, value):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, dxl_id, addr, value
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[ID:{dxl_id}] Comm error (1B): {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"[ID:{dxl_id}] SDK error (1B): {self.packetHandler.getRxPacketError(dxl_error)}")

    def _write4(self, dxl_id, addr, value):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, dxl_id, addr, value
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[ID:{dxl_id}] Comm error (4B): {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"[ID:{dxl_id}] SDK error (4B): {self.packetHandler.getRxPacketError(dxl_error)}")

    def _to_unsigned_4b(self, vel):
        """Convert signed int velocity to unsigned 32-bit two's complement expected by Goal Velocity."""
        if vel < 0:
            return (1 << 32) + vel
        return vel

    # ---------- Motor setup ----------
    def _set_velocity_mode_and_enable(self):
        for dxl_id in self.ids:
            # Disable torque to change mode safely
            self._write1(dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            # Set Velocity Control Mode
            self._write1(dxl_id, self.ADDR_OPERATING_MODE, self.VELOCITY_CONTROL_MODE)
            # Re-enable torque
            self._write1(dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            # Ensure initially stopped
            self._write4(dxl_id, self.ADDR_GOAL_VELOCITY, 0)

    # ---------- Velocity commands ----------
    def set_velocity_single(self, dxl_id, vel):
        """vel in [-FULL_VEL, FULL_VEL]"""
        vel = max(-self.FULL_VEL, min(self.FULL_VEL, vel))
        self._write4(dxl_id, self.ADDR_GOAL_VELOCITY, self._to_unsigned_4b(vel))

    def set_velocity_both(self, v_left, v_right):
        self.set_velocity_single(self.DXL_LEFT_ID,  v_left)
        self.set_velocity_single(self.DXL_RIGHT_ID, v_right)

    def stop(self):
        self.set_velocity_both(0, 0)

    # ---------- Simple keyboard control ----------
    def drive_loop(self):
        print(
            "Controls:\n"
            "  f = both motors forward (full)\n"
            "  l = left turn (RIGHT motor only)\n"
            "  r = right turn (LEFT motor only)\n"
            "  s = stop\n"
            "  q = stop & quit"
        )
        try:
            while True:
                cmd = input("Command [f/l/r/s/q]: ").strip().lower()
                if cmd == 'f':
                    # Forward: both positive. Flip sign if your mounting requires it.
                    self.set_velocity_both(self.FULL_VEL, -self.FULL_VEL)
                elif cmd == 'l':
                    # Turn left: spin RIGHT motor only
                    self.set_velocity_both(0, -self.FULL_VEL)
                elif cmd == 'r':
                    # Turn right: spin LEFT motor only
                    self.set_velocity_both(self.FULL_VEL, 0)
                elif cmd == 's':
                    self.stop()
                elif cmd == 'q':
                    self.stop()
                    break
                else:
                    print("Unknown command. Use f/l/r/s/q.")
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    # ---------- Cleanup ----------
    def exit_sequence(self):
        try:
            self.stop()
            for dxl_id in self.ids:
                self._write1(dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        finally:
            self.portHandler.closePort()


if __name__ == "__main__":
    DC = DualDynamixelControl()
    DC.drive_loop()
