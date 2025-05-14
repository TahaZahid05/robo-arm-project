import serial
import time

# Replace with your actual COM port (e.g., 'COM3' on Windows, '/dev/ttyACM0' on Linux)
arduino = serial.Serial(port='COM3', baudrate=9600, timeout=1)
time.sleep(2)  # Wait for Arduino to reset

# Send data
arduino.write(b'Hello from Python\n')

# Read response
response = arduino.readline().decode('utf-8').strip()
print(f"Arduino replied: {response}")

arduino.close()
