import serial
import time
import matplotlib.pyplot as plt

# === Serial Setup (Change COM port as needed) ===
ser = serial.Serial('COM4', 9600, timeout=1)
time.sleep(2)

rpm_values = []
times = []

plt.ion()
fig, ax = plt.subplots(figsize=(10, 5))
(line,) = ax.plot([], [], lw=2, color="#00AAFF", label="Motor RPM")

ax.set_xlabel("Time (s)")
ax.set_ylabel("RPM")
ax.set_title("🚀 Live Motor RPM Monitor", fontsize=13, pad=10)
ax.set_ylim(0, 6000)  # ✅ Fixed Y-axis
ax.grid(True, linestyle='--', alpha=0.6)
ax.legend(loc='upper right', frameon=False)

start_time = time.time()
last_plot_update = time.time()
last_terminal_print = time.time()

UPDATE_INTERVAL = 0.1   # Plot refresh = 10 FPS
PRINT_INTERVAL = 1.0    # Terminal print = 1 second

try:
    while True:
        if not plt.fignum_exists(fig.number):
            break

        line_data = ser.readline().decode("utf-8").strip()

        if line_data.startswith("RPM:"):
            try:
                rpm = int(line_data.split(":")[1])
                current_time = time.time() - start_time

                rpm_values.append(rpm)
                times.append(current_time)

                if len(rpm_values) > 100:
                    rpm_values.pop(0)
                    times.pop(0)

                # ✅ Print to terminal every 1 second
                if time.time() - last_terminal_print > PRINT_INTERVAL:
                    print(f"Current RPM: {rpm}")
                    last_terminal_print = time.time()

                # ✅ Update plot at fixed FPS
                if time.time() - last_plot_update > UPDATE_INTERVAL:
                    line.set_data(times, rpm_values)
                    ax.set_xlim(max(0, current_time - 5), current_time)  # show last 5 seconds
                    fig.canvas.draw()
                    fig.canvas.flush_events()
                    last_plot_update = time.time()

            except ValueError:
                pass

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    ser.close()
    plt.close(fig)
    print("Serial closed. Program ended.")
