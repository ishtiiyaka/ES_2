import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re, time

# === USER SETTINGS ===
COM_PORT = 'COM4'    # Change to your Tiva COM port
BAUD_RATE = 9600

# === SERIAL ===
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)

# === DATA STORAGE ===
rpm1_data, rpm2_data, time_data = [], [], []
start_time = time.time()

# === PLOT SETUP ===
plt.style.use('seaborn-v0_8-darkgrid')
fig, ax = plt.subplots()
line1, = ax.plot([], [], 'r-', label='Motor 1 RPM')
line2, = ax.plot([], [], 'b-', label='Motor 2 RPM')
ax.set_xlabel('Time (s)')
ax.set_ylabel('RPM')
ax.set_title('Live RPM vs Time (1s interval)')
ax.legend()

# === UPDATE ===
def update(frame):
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line:
        match = re.search(r'RPM1=(\d+),\s*RPM2=(\d+)', line)
        if match:
            rpm1 = int(match.group(1))
            rpm2 = int(match.group(2))
            t = time.time() - start_time

            rpm1_data.append(rpm1)
            rpm2_data.append(rpm2)
            time_data.append(t)

            print(f"{t:.1f}s | RPM1={rpm1}, RPM2={rpm2}")

            if len(time_data) > 100:
                time_data.pop(0)
                rpm1_data.pop(0)
                rpm2_data.pop(0)

            line1.set_data(time_data, rpm1_data)
            line2.set_data(time_data, rpm2_data)
            ax.relim()
            ax.autoscale_view()

    return line1, line2

ani = FuncAnimation(fig, update, interval=1000)
plt.show()
ser.close()
