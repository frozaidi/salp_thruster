import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import csv
import time
import threading

# Serial port configuration (CHANGE THIS to match your setup)
SERIAL_PORT = '/dev/tty.usbmodem1303'
BAUD_RATE = 1000000

# Serial port open
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Plot buffer
max_points = 200
tick_buffer = deque([0]*max_points, maxlen=max_points)
adc_buffer = deque([0]*max_points, maxlen=max_points)

# CSV log file
timestamp = time.strftime("%Y%m%d_%H%M%S")
# csv_filename = f"loadcell_log_{timestamp}.csv"
csv_filename = f"Forward_test.csv"
csv_file = open(csv_filename, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["Tick (ms)", "ADC Value"])

# Lock for thread-safe buffer updates
buffer_lock = threading.Lock()

# Serial reading thread
def read_serial():
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                parts = line.split(',')
                if len(parts) == 2:
                    tick = int(parts[0].strip())
                    value = int(parts[1].strip())

                    with buffer_lock:
                        tick_buffer.append(tick)
                        adc_buffer.append(value)
                        csv_writer.writerow([tick, value])
        except Exception as e:
            print("Error reading line:", e)

# Start reader thread
thread = threading.Thread(target=read_serial, daemon=True)
thread.start()

# Matplotlib plot setup
fig, ax = plt.subplots()
line, = ax.plot([], [])
ax.set_title("STM32 Loadcell Real-Time Plot")
ax.set_xlabel("Tick (ms)")
ax.set_ylabel("ADC Value")
ax.grid(True)

def update_plot(frame):
    with buffer_lock:
        line.set_data(tick_buffer, adc_buffer)
        if tick_buffer:
            ax.set_xlim(min(tick_buffer), max(tick_buffer))
            ax.set_ylim(min(adc_buffer)-50, max(adc_buffer)+50)
    return line,

ani = animation.FuncAnimation(fig, update_plot, interval=60)

plt.tight_layout()
plt.show()

# Cleanup after window closed
ser.close()
csv_file.close()