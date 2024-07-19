import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from matplotlib.widgets import Button

MAX_RADIUS = 20  # Maximum radius limit for the polar plot

def parse_packet(packet):
    try:
        # Remove the start and end markers
        packet = packet.strip('<>')
        
        # Split the data fields
        fields = packet.split(',')
        
        # Initialize the parsed data dictionary
        data = {}
        
        for field in fields:
            key, value = field.split(':')
            data[key] = float(value)
        
        print(data)
        return data
    except ValueError as e:
        print(f"Error parsing packet: {e}")
        return None

port = 'COM4'
baud_rate = 57600

ser = serial.Serial(port, baud_rate, timeout=1)

# Initialize the plot
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

# Adjusting for a polar plot
ax.set_theta_direction(1)  # clockwise
ax.set_theta_zero_location('E')  # set 0 degrees to North


# Initialize plot data
theta_data = []
r_data = []
line, = ax.plot(theta_data, r_data, 'bo', markersize=3)
ax.set_rlim(0, MAX_RADIUS)

def update(frame):
    if ser.in_waiting > 0:
        packet = ser.readline().decode('utf-8').strip()
        data = parse_packet(packet)
        print(data)
        
        if data:
            angle_yaw = data.get('A', None)
            distance = data.get('D', None)
            if angle_yaw is not None and distance is not None and distance is not -1:
                # Convert angle to radians for polar plot
                theta = np.radians(angle_yaw)
                
                # Append new data
                theta_data.append(theta)
                r_data.append(distance)

                # Update the plot data
                line.set_data(theta_data, r_data)
                
                # Adjust plot view
                ax.relim()
                ax.autoscale_view()
                
                plt.draw()
            else:
                plt.draw()
    
    return line,

def close(event):
    
    ser.close()
    plt.close(fig)

# Adding the stop button
ax_button = plt.axes([0.8, 0.01, 0.1, 0.075])
button = Button(ax_button, 'Stop')
button.on_clicked(close)

ani = animation.FuncAnimation(fig, update, blit=True, interval=1)
plt.title('LIDAR')
plt.show()
