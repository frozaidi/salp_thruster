from dynamixel_sdk import *  # Uses Dynamixel SDK library
import time

# ---------- USER CONFIG ----------
DEVICENAME = '/dev/tty.usbserial-FT891KEM'  # Update this to your serial port
BAUDRATE = 57600
DXL_ID = 1
VELOCITY = 50  # Velocity in Dynamixel units (approx. RPM * 0.229)
# ----------------------------------

PROTOCOL_VERSION = 2.0

ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_POSITION = 132

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
VELOCITY_CONTROL_MODE = 1

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if not portHandler.openPort():
    print("Failed to open port")
    quit()

# Set baudrate
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to set baudrate")
    quit()

# Set operating mode to velocity control
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, 30)

try:
    while True:
        vel = input("Enter desired velocity [-1023 to 1023, 0 to stop]: ")
        try:
            vel = int(vel)
            if -1023 <= vel <= 1023:
                # Convert to unsigned 4-byte value
                if vel < 0:
                    vel = (1 << 32) + vel  # Convert to 2's complement
                packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, vel)
            else:
                print("Velocity out of range.")
        except ValueError:
            print("Invalid input. Enter an integer.")

        # dxl_position, result, error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)

        # if result != COMM_SUCCESS:
        #     print("Comm Error:", packetHandler.getTxRxResult(result))
        # elif error != 0:
        #     print("DXL Error:", packetHandler.getRxPacketError(error))
        # else:
        #     position_deg = (dxl_position % 4096) * 360.0 / 4096

        #     print(f"Position: {dxl_position} ticks ({position_deg:.2f}°)")

        # time.sleep(0.1)  # 10 Hz update rate

except KeyboardInterrupt:
    print("\nStopping motor...")

# Stop motor and disable torque
time.sleep(0.1)
packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, 0)
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
portHandler.closePort()
