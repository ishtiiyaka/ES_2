import serial
import time
import matplotlib.pyplot as plt

# === Adjust COM port to match your PC ===
ser = serial.Serial('COM4', 9600, timeout=1)
time.sleep(2)

rpm_values = []
times = []

plt.ion()
fig, ax = plt.subplots(figsize=(8, 4))
(line,) = ax.plot(times, rpm_values, lw=2, color="#00AAFF", label="Motor RPM")
ax.set_xlabel("Time (s)")
ax.set_ylabel("RPM")
ax.set_title("🚀 Live Motor RPM Monitor", fontsize=13, pad=10)
ax.set_ylim(0, 10000)  # ✅ Fixed Y-axis range
ax.grid(True, linestyle='--', alpha=0.6)
ax.legend(loc='upper right', frameon=False)

start_time = time.time()
last_update_time = time.time()

try:
    while True:
        # Stop gracefully if plot window is closed
        if not plt.fignum_exists(fig.number):
            break

        line_data = ser.readline().decode("utf-8").strip()
        if line_data.startswith("RPM:"):
            try:
                rpm = int(line_data.split(":")[1])

                # 🔹 Treat RPM < 380 as motor stopped (show 0)
                if rpm < 380:
                    rpm = 0

                rpm_values.append(rpm)
                times.append(time.time() - start_time)

                # Keep only the latest 100 points
                if len(rpm_values) > 100:
                    rpm_values.pop(0)
                    times.pop(0)

                # Update plot every 0.2 seconds
                if time.time() - last_update_time > 0.2:
                    line.set_xdata(times)
                    line.set_ydata(rpm_values)
                    ax.relim()
                    ax.autoscale_view(scalex=True, scaley=False)
                    plt.draw()
                    plt.pause(0.001)
                    last_update_time = time.time()

            except ValueError:
                pass

except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    ser.close()
    plt.close(fig)
    print("Serial closed. Program ended.")