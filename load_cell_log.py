import serial
import csv
import time
import os.path

# Configure serial port and CSV file path
ser = serial.Serial('/dev/cu.usbmodem1201', 57600)  # Replace 'COM3' with your Arduino's serial port
csv_file_path = 'water_test.csv'
append_num = 1

# Split the file path and extension
file_base, file_ext = os.path.splitext(csv_file_path)

# Loop to check if the file already exists
while os.path.isfile(csv_file_path):
    # Reconstruct the file path with the number before the extension
    csv_file_path = f"{file_base}_{append_num}{file_ext}"
    append_num += 1



# Open the CSV file for writing
with open(csv_file_path, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time (s)", "Sensor Value"])  # Headers: Time in seconds and sensor value
    
    # Record the start time
    start_time = time.time()
    
    try:
        while True:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()  # Read and decode data from Arduino
                
                # Calculate elapsed time in seconds
                elapsed_time = time.time() - start_time
                
                writer.writerow([elapsed_time, data])  # Write elapsed time and data to CSV
                print(f"{elapsed_time:.2f}, {data}")  # Optional: Print to console with 2 decimal places
    except KeyboardInterrupt:
        print("Logging stopped.")
    finally:
        ser.close()
