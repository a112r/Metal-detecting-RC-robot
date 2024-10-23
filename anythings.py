import tkinter as tk
import serial 
import matplotlib.pyplot as plt
from collections import deque
import threading

# configure the serial port
ser = serial.Serial(
    port='COM5',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS
)

# Initialize deque to store data for frequency plotting
max_len_freq = 100  # Maximum number of data points to display
frequency_data = deque(maxlen=max_len_freq)

# Function to send direction based on keyboard input
def send_direction(key):
    if key == 'w':
        ser.write(b'4\n')
    elif key == 'a':
        ser.write(b'1\n')
    elif key == 's':
        ser.write(b'0\n')
    elif key == 'd':
        ser.write(b'5\n')

# Function to handle key press event
def key_press(event):
    key = event.char.lower()
    if key in ['w', 'a', 's', 'd']:
        send_direction(key)

# Function to read serial data and update frequency plot
def read_serial_and_plot():
    while True:
        data = ser.readline().decode('utf-8').strip()
        if data:
            try:
                frequency = int(data)
                frequency_data.append(frequency)
                plt.clf()
                plt.plot(frequency_data)
                plt.xlabel('Time')
                plt.ylabel('Frequency')
                plt.title('Frequency Plot')
                plt.pause(0.01)
            except ValueError:
                print("Invalid data format:", data)

# Create Tkinter window
root = tk.Tk()
root.geometry("200x200")
root.title("Key Press")

# Bind key press event to the window
root.bind("<KeyPress>", key_press)

# Start thread to read serial data and plot
serial_thread = threading.Thread(target=read_serial_and_plot)
serial_thread.start()

# Run the Tkinter event loop
root.mainloop()

# Close the serial connection when the program exits
ser.close()
