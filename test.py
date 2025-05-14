import serial
import time

# Replace with your actual COM port (e.g., 'COM3' on Windows, '/dev/ttyACM0' on Linux)
arduino = serial.Serial(port='COM10', baudrate=9600, timeout=1)
time.sleep(2)  # Wait for Arduino to reset

try:
    while True:
        user_input = input("Enter message to arduino: ")
        if user_input.lower() == 'exit':
            break
        
        arduino.write((user_input + "\n").encode())
        
        time.sleep(0.1)
        while arduino.in_waiting:
            response = arduino.readline().decode().strip()
            print("From Arduino: " , response)
except KeyboardInterrupt:
    print("\n[INFO] Stopping..")            
    

finally:
    arduino.close()
    print("[INFO] Serial connection closed")
