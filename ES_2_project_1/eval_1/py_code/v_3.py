import serial
import time

# === Serial Setup (Change COM port as needed) ===
ser = serial.Serial('COM4', 9600, timeout=1)
time.sleep(2)

last_print_time = time.time()
PRINT_INTERVAL = 1.0  # Print every 1 second

print("📌 Starting RPM Monitor...\n")

try:
    while True:
        line_data = ser.readline().decode("utf-8").strip()

        if line_data.startswith("RPM:"):
            try:
                rpm = int(line_data.split(":")[1])

                # ✅ Print once per second
                if time.time() - last_print_time >= PRINT_INTERVAL:
                    print(f"Current RPM: {rpm}")
                    last_print_time = time.time()

            except ValueError:
                pass

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    ser.close()
    print("Serial closed. Program ended.")
