import serial
import time

# --- Configure your COM port here ---
# Example: COM3 (Windows), /dev/ttyUSB0 (Linux)
ser = serial.Serial('COM4', 9600, timeout=1)
time.sleep(2)  # Wait for connection to stabilize

print("Reading MPU6050 data from TM4C123...\n")

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:  # If non-empty data received
                print(line)  # Print directly in terminal
except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    ser.close()
