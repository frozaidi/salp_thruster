import time
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import atexit

class TripleDynamixelControl:
    def __init__(self):
        # ---------- USER CONFIG ----------
        self.DEVICENAME = '/dev/tty.usbserial-FT891KEM'  # Update this to your serial port
        self.BAUDRATE = 57600

        # Set your motor IDs here
        self.DXL_IDS = [1, 2, 3]

        # “Full speed” in Dynamixel velocity units (≈0.229 rpm per LSB on X-series)
        self.FULL_VEL = 306  # clamp velocities to ±FULL_VEL

        # Interactive calibration jog speed (slow & safe)
        self.CALIB_VEL = 40
        # ----------------------------------

        self.PROTOCOL_VERSION = 2.0

        # Control table addresses (X-series, Protocol 2.0)
        self.ADDR_TORQUE_ENABLE     = 64
        self.ADDR_OPERATING_MODE    = 11
        self.ADDR_GOAL_VELOCITY     = 104
        self.ADDR_PRESENT_POSITION  = 132

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

        # Group sync write for Goal Velocity (4 bytes)
        self.groupSyncWrite = GroupSyncWrite(
            self.portHandler,
            self.packetHandler,
            self.ADDR_GOAL_VELOCITY,
            4
        )

        # Initialize motors
        self._set_velocity_mode_and_enable()

        # Dictionary to store “starting positions” you select
        self.start_pos = {}

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

    def _read4(self, dxl_id, addr):
        val, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
            self.portHandler, dxl_id, addr
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[ID:{dxl_id}] Comm error (read4B): {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"[ID:{dxl_id}] SDK error (read4B): {self.packetHandler.getRxPacketError(dxl_error)}")
        return val

    def _to_unsigned_4b(self, vel):
        """Convert signed int velocity to unsigned 32-bit two's complement expected by Goal Velocity."""
        if vel < 0:
            return (1 << 32) + vel
        return vel

    # ---------- Motor setup ----------
    def _set_velocity_mode_and_enable(self):
        for dxl_id in self.DXL_IDS:
            # Disable torque to change mode safely
            self._write1(dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            # Set Velocity Control Mode
            self._write1(dxl_id, self.ADDR_OPERATING_MODE, self.VELOCITY_CONTROL_MODE)
            # Re-enable torque
            self._write1(dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            # Ensure initially stopped
            self._write4(dxl_id, self.ADDR_GOAL_VELOCITY, 0)

    # ---------- Position / calibration ----------
    def read_position(self, dxl_id):
        return self._read4(dxl_id, self.ADDR_PRESENT_POSITION)

    def interactive_startup_calibration(self):
        """
        For each motor:
          j = jog slowly negative
          k = jog slowly positive
          s = stop
          c = confirm current angle as starting position
        """
        print("\n--- Interactive startup calibration ---")
        print("For each motor:")
        print("  j = jog negative   (slow)")
        print("  k = jog positive   (slow)")
        print("  s = stop/joint hold")
        print("  c = confirm current position as START\n")

        for dxl_id in self.DXL_IDS:
            print(f"\n=== Calibrating motor ID {dxl_id} ===")
            # start stopped
            self.set_velocity_single(dxl_id, 0)

            while True:
                pos = self.read_position(dxl_id)
                print(f"[ID {dxl_id}] Current position (ticks): {pos}")

                cmd = input("Command [j/k/s/c]: ").strip().lower()

                if cmd == 'j':
                    # jog negative direction
                    self.set_velocity_single(dxl_id, -self.CALIB_VEL)
                elif cmd == 'k':
                    # jog positive direction
                    self.set_velocity_single(dxl_id, self.CALIB_VEL)
                elif cmd == 's':
                    # stop at current angle
                    self.set_velocity_single(dxl_id, 0)
                elif cmd == 'c':
                    # confirm this as starting position
                    self.set_velocity_single(dxl_id, 0)
                    final_pos = self.read_position(dxl_id)
                    self.start_pos[dxl_id] = final_pos
                    print(f"  -> Saved start position for ID {dxl_id}: {final_pos} ticks")
                    break
                else:
                    print("Unknown command. Use j/k/s/c.")

        print("\nCalibration complete.")
        print("Start positions:", self.start_pos, "\n")

    # ---------- Velocity commands ----------
    def set_velocity_single(self, dxl_id, vel):
        """vel in [-FULL_VEL, FULL_VEL]"""
        vel = int(max(-self.FULL_VEL, min(self.FULL_VEL, vel)))
        self._write4(dxl_id, self.ADDR_GOAL_VELOCITY, self._to_unsigned_4b(vel))

    def set_velocity_all(self, vels):
        """
        vels: list/tuple of velocities, same order as DXL_IDS
        Uses GroupSyncWrite for one-packet update.
        """
        if len(vels) != len(self.DXL_IDS):
            raise ValueError("vels length must match number of IDs")

        self.groupSyncWrite.clearParam()

        for dxl_id, vel in zip(self.DXL_IDS, vels):
            vel = int(max(-self.FULL_VEL, min(self.FULL_VEL, vel)))
            u = self._to_unsigned_4b(vel)

            param_goal_vel = [
                DXL_LOBYTE(DXL_LOWORD(u)),
                DXL_HIBYTE(DXL_LOWORD(u)),
                DXL_LOBYTE(DXL_HIWORD(u)),
                DXL_HIBYTE(DXL_HIWORD(u)),
            ]
            add_ok = self.groupSyncWrite.addParam(dxl_id, bytearray(param_goal_vel))
            if not add_ok:
                print(f"[ID:{dxl_id}] Failed to add param to sync write")

        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"GroupSyncWrite comm error: {self.packetHandler.getTxRxResult(dxl_comm_result)}")

    def stop(self):
        self.set_velocity_all([0] * len(self.DXL_IDS))

    # ---------- Single-line keyboard control ----------
    def drive_loop(self):
        print(
            "Single-line velocity control:\n"
            "  Type 3 velocities (space or comma separated), e.g.:  100 0 -100\n"
            f"  Velocities are clamped to ±{self.FULL_VEL}\n"
            "  Empty line = repeat last command\n"
            "  s = stop all\n"
            "  q = stop & quit\n"
        )

        last_vels = [0] * len(self.DXL_IDS)

        try:
            while True:
                cmd = input("v1 v2 v3 | s | q: ").strip().lower()

                if cmd in ("q", "quit", "exit"):
                    self.stop()
                    break
                if cmd in ("s", "stop"):
                    self.stop()
                    last_vels = [0] * len(self.DXL_IDS)
                    continue
                if cmd == "":
                    # Empty line repeats last command (handy)
                    self.set_velocity_all(last_vels)
                    continue

                # Parse numbers from line
                parts = cmd.replace(",", " ").split()
                if len(parts) != len(self.DXL_IDS):
                    print(f"Please enter exactly {len(self.DXL_IDS)} velocities.")
                    continue

                try:
                    vels = [int(float(p)) for p in parts]
                except ValueError:
                    print("Could not parse velocities. Example: 100 0 -100")
                    continue

                self.set_velocity_all(vels)
                last_vels = vels

        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    # ---------- Cleanup ----------
    def exit_sequence(self):
        try:
            self.stop()
            for dxl_id in self.DXL_IDS:
                self._write1(dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        finally:
            self.portHandler.closePort()


if __name__ == "__main__":
    DC = TripleDynamixelControl()
    DC.interactive_startup_calibration()
    DC.drive_loop()
