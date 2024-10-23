import serial
import keyboard
import matplotlib.pyplot as plt
#from collections import deque
#from threading import Thread
import time

# Set up the serial connection
ser = serial.serial('COM5',
                    115200,
                    timeout=1,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_TWO,
                    bytesize=serial.EIGHTBITS)  # Replace 'COM5' with the correct port for your device
ser.flushInput()


# Function to print direction based on keyboard input
def print_direction(event):
    if event.event_type == keyboard.KEY_DOWN:
        if keyboard.is_pressed('shift') and keyboard.is_pressed('w'):
            ser.write(b'6\n')
        elif event.name == 'w':
            ser.write(b'4\n')
        elif event.name == 'a':
            ser.write(b'1\n')
        elif event.name == 's':
            ser.write(b'0\n')
        elif event.name == 'd':
            ser.write(b'5\n')

# Register the key events
keyboard.on_press(print_direction)

# Function to read serial data and update plots
def read_serial_and_plot():
    frequency_data = deque(maxlen=100)  # Maximum number of data points to display

    # Create plot for frequency
    plt.figure(figsize=(6, 4))
    plt.title('Frequency Plot')
    plt.xlabel('Time')
    plt.ylabel('Frequency')
    line_freq, = plt.plot(frequency_data)
    plt.ylim(0, 10)  # Adjust y-axis limits as needed

    while True:
        data = ser.readline().decode('utf-8').rstrip()

        if data:
            try:
                # Assume data is an integer value
                value = int(data)
                # Update frequency plot
                frequency_data.append(value)
                line_freq.set_ydata(frequency_data)
                plt.draw()
                plt.pause(0.01)
            except ValueError:
                print("Invalid data format:", data)


# Start a thread to read from serial and plot
serial_thread = Thread(target=read_serial_and_plot)
serial_thread.daemon = True
serial_thread.start()

# Keep the main thread running to listen for keyboard events
while True:
    time.sleep(0.1)  # Sleep to avoid high CPU usage

ser.close()
