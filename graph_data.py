import csv
import matplotlib.pyplot as plt

# Define lists to store x and y values
x_data = []
y_data = []

# Read data from CSV file
with open('water_test_4.csv', 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Skip the first row (header)
    for row in reader:
        # Convert values to float and append to lists
        x_data.append(float(row[0]))
        y_data.append(float(row[1]))

# Plot the data
plt.plot(x_data, y_data, marker='o', linestyle='-', color='b')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Plot of CSV Data')
plt.grid(True)
plt.show()
