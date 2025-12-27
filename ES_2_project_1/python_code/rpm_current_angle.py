import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import math
import re
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation

# --- Configuration ---
MAX_DATA_POINTS = 50  # How many points to show on the graph (width)
UPDATE_INTERVAL = 100 # Graph update interval in ms

class VisualDashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("TM4C123 Real-Time Telemetry")
        self.root.geometry("1000x700")
        
        # --- Data Storage (Deques for sliding window graphs) ---
        self.rpm1_data = deque([0]*MAX_DATA_POINTS, maxlen=MAX_DATA_POINTS)
        self.rpm2_data = deque([0]*MAX_DATA_POINTS, maxlen=MAX_DATA_POINTS)
        self.curr1_data = deque([0]*MAX_DATA_POINTS, maxlen=MAX_DATA_POINTS)
        self.curr2_data = deque([0]*MAX_DATA_POINTS, maxlen=MAX_DATA_POINTS)
        
        # Variables for Text Display
        self.pitch_var = tk.StringVar(value="Pitch: 0.00°")
        self.roll_var = tk.StringVar(value="Roll: 0.00°")
        self.status_var = tk.StringVar(value="Status: Disconnected")
        
        self.serial_port = None
        self.is_running = False
        
        self._setup_ui()
        self._scan_ports()
        
        # Start the graph update loop
        self._animate_graphs()

    def _setup_ui(self):
        # 1. Top Bar: Connection Controls
        top_frame = ttk.Frame(self.root, padding=10)
        top_frame.pack(side=tk.TOP, fill=tk.X)
        
        ttk.Label(top_frame, text="Port:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top_frame, width=15)
        self.port_combo.pack(side=tk.LEFT, padx=5)
        
        self.btn_connect = ttk.Button(top_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(top_frame, textvariable=self.status_var, foreground="gray").pack(side=tk.LEFT, padx=10)

        # 2. Main Graph Area (Matplotlib)
        graph_frame = ttk.Frame(self.root)
        graph_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Create Figure with 2 Subplots
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        self.fig.subplots_adjust(hspace=0.3) # Space between graphs
        
        # Setup RPM Graph (Top)
        self.line_r1, = self.ax1.plot(self.rpm1_data, label='Motor 1 RPM', color='cyan', linewidth=2)
        self.line_r2, = self.ax1.plot(self.rpm2_data, label='Motor 2 RPM', color='magenta', linewidth=2)
        self.ax1.set_title("Motor RPM")
        self.ax1.set_ylabel("RPM")
        self.ax1.legend(loc="upper left")
        self.ax1.grid(True, linestyle='--', alpha=0.6)
        
        # Setup Current Graph (Bottom)
        self.line_c1, = self.ax2.plot(self.curr1_data, label='Motor 1 Current', color='yellow', linewidth=2)
        self.line_c2, = self.ax2.plot(self.curr2_data, label='Motor 2 Current', color='orange', linewidth=2)
        self.ax2.set_title("Motor Current (mA)")
        self.ax2.set_ylabel("Current (mA)")
        self.ax2.legend(loc="upper left")
        self.ax2.grid(True, linestyle='--', alpha=0.6)

        # Embed plot in Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # 3. Bottom Bar: Angle Values
        info_frame = ttk.LabelFrame(self.root, text="Orientation (MPU6050)", padding=10)
        info_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=10)
        
        lbl_pitch = ttk.Label(info_frame, textvariable=self.pitch_var, font=("Consolas", 20, "bold"), foreground="#2c3e50")
        lbl_pitch.pack(side=tk.LEFT, expand=True)
        
        lbl_roll = ttk.Label(info_frame, textvariable=self.roll_var, font=("Consolas", 20, "bold"), foreground="#2c3e50")
        lbl_roll.pack(side=tk.LEFT, expand=True)

    def _animate_graphs(self):
        # This function runs every UPDATE_INTERVAL milliseconds to refresh the plot
        if self.is_running:
            # Update RPM lines
            self.line_r1.set_ydata(self.rpm1_data)
            self.line_r2.set_ydata(self.rpm2_data)
            self.ax1.relim()      # Recompute limits based on new data
            self.ax1.autoscale_view() # Rescale axis
            
            # Update Current lines
            self.line_c1.set_ydata(self.curr1_data)
            self.line_c2.set_ydata(self.curr2_data)
            self.ax2.relim()
            self.ax2.autoscale_view()
            
            self.canvas.draw()
        
        # Schedule next update
        self.root.after(UPDATE_INTERVAL, self._animate_graphs)

    def _scan_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports: self.port_combo.current(0)

    def toggle_connection(self):
        if not self.is_running:
            try:
                port = self.port_combo.get()
                self.serial_port = serial.Serial(port, 9600, timeout=1)
                self.is_running = True
                self.btn_connect.config(text="Disconnect")
                self.status_var.set("Status: Connected")
                
                # Start reading thread
                self.thread = threading.Thread(target=self.read_serial, daemon=True)
                self.thread.start()
            except Exception as e:
                self.status_var.set(f"Error: {e}")
        else:
            self.is_running = False
            if self.serial_port: self.serial_port.close()
            self.btn_connect.config(text="Connect")
            self.status_var.set("Status: Disconnected")

    def calculate_angles(self, ax, ay, az):
        try:
            pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
            roll  = math.degrees(math.atan2(ay, az))
            return pitch, roll
        except:
            return 0.0, 0.0

    def read_serial(self):
        while self.is_running:
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    self.process_data(line)
            except:
                break

    def process_data(self, line):
        # 1. Parse RPM/Current
        if "M1_RPM" in line:
            # Regex: "M1_RPM:1200 I1:50 ... M2_RPM:1300 I2:60"
            match = re.search(r"M1_RPM:(\d+).*I1:(\d+).*M2_RPM:(\d+).*I2:(\d+)", line)
            if match:
                r1, i1, r2, i2 = map(int, match.groups())
                
                # Append to data deques (automatically removes old items)
                self.rpm1_data.append(r1)
                self.curr1_data.append(i1)
                self.rpm2_data.append(r2)
                self.curr2_data.append(i2)

        # 2. Parse MPU
        elif "MPU:" in line:
            match = re.search(r"AX=([-\d\.]+).*AY=([-\d\.]+).*AZ=([-\d\.]+)", line)
            if match:
                ax, ay, az = map(float, match.groups())
                p, r = self.calculate_angles(ax, ay, az)
                
                # Update Text Variables (Thread safe enough for Tkinter Vars)
                self.pitch_var.set(f"Pitch: {p:.2f}°")
                self.roll_var.set(f"Roll: {r:.2f}°")

if __name__ == "__main__":
    root = tk.Tk()
    # Optional: Set a theme if available
    try:
        root.tk.call("source", "azure.tcl") # Example theme loading
        root.tk.call("set_theme", "dark")
    except:
        pass
        
    app = VisualDashboard(root)
    root.mainloop()