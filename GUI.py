import tkinter as tk
from tkinter import ttk
import threading
import time
import random
import serial
import csv
import serial.tools.list_ports
from datetime import datetime

# --- CONFIGURATION ---
# Set to True to generate local, randomized test data without an Arduino connected.
# Set to False to connect to live hardware via USB serial.
SIM_MODE = False  
# ---------------------

class BusPanel(ttk.LabelFrame):
    """
    A reusable UI widget that creates a standardized dashboard panel for a specific power bus.
    Handles the layout and rendering of Voltage, Current, Apparent Power, Real Power, and PF.
    """
    def __init__(self, parent, title, show_current=True, show_apparent=True, show_real=False, show_pf=False, wide_layout=False):
        super().__init__(parent, text=title, padding=(15, 15, 15, 25))
        
        self.show_current = show_current
        self.show_apparent = show_apparent
        self.show_real = show_real
        self.show_pf = show_pf
        self.wide_layout = wide_layout
        
        if self.wide_layout:
            self.columnconfigure(0, weight=1)
            self.columnconfigure(1, weight=1)
            self.columnconfigure(2, weight=1)
        else:
            self.columnconfigure(0, weight=1)
            self.columnconfigure(1, weight=1)
        
        # --- VOLTAGE SECTION ---
        v_container = ttk.Frame(self)
        v_container.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        
        ttk.Label(v_container, text="Voltage", font=("Segoe UI", 14, "bold")).pack(anchor="w")
        self.voltage = tk.StringVar(value="-- V")
        ttk.Label(v_container, textvariable=self.voltage, font=("Consolas", 40, "bold"), foreground="#007ACC", width=10).pack(anchor="w")

        # Voltage Progress Bar (0-150V Scale)
        v_bar_frame = ttk.Frame(v_container)
        v_bar_frame.pack(fill="x", pady=(5, 0), anchor="w")
        ttk.Label(v_bar_frame, text="0V", font=("Segoe UI", 10, "bold"), foreground="#666666").pack(side=tk.LEFT)
        self.v_bar = ttk.Progressbar(v_bar_frame, orient="horizontal", mode="determinate", maximum=150, style="Blue.Horizontal.TProgressbar")
        self.v_bar.pack(side=tk.LEFT, fill="x", expand=True, padx=8)
        ttk.Label(v_bar_frame, text="150V", font=("Segoe UI", 10, "bold"), foreground="#666666").pack(side=tk.LEFT)

        # --- CURRENT SECTION ---
        if self.show_current:
            i_container = ttk.Frame(self)
            i_container.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
            
            ttk.Label(i_container, text="Current", font=("Segoe UI", 14, "bold")).pack(anchor="w")
            self.current = tk.StringVar(value="-- A")
            ttk.Label(i_container, textvariable=self.current, font=("Consolas", 40, "bold"), foreground="#007ACC", width=10).pack(anchor="w")

            # Current Progress Bar (0-1A Scale)
            i_bar_frame = ttk.Frame(i_container)
            i_bar_frame.pack(fill="x", pady=(5, 0), anchor="w")
            ttk.Label(i_bar_frame, text="0A", font=("Segoe UI", 10, "bold"), foreground="#666666").pack(side=tk.LEFT)
            self.i_bar = ttk.Progressbar(i_bar_frame, orient="horizontal", mode="determinate", maximum=1.0, style="Blue.Horizontal.TProgressbar")
            self.i_bar.pack(side=tk.LEFT, fill="x", expand=True, padx=8)
            ttk.Label(i_bar_frame, text="1A", font=("Segoe UI", 10, "bold"), foreground="#666666").pack(side=tk.LEFT)
            
        # --- POWER & PF SECTION ---
        if self.wide_layout:
            va_row, va_col = 1, 0
            w_row, w_col = 1, 1
            pf_row, pf_col = 1, 2
        else:
            va_row, va_col = 1, 0
            w_row, w_col = 1, 1
            pf_row, pf_col = 2, 0 

        if self.show_apparent:
            self.apparent = tk.StringVar(value="-- VA")
            self._create_metric("Apparent Power", self.apparent, va_row, va_col)
            
        if self.show_real:
            self.real = tk.StringVar(value="-- W")
            self._create_metric("Real Power", self.real, w_row, w_col)
            
        if self.show_pf:
            self.pf = tk.StringVar(value="--")
            self._create_metric("PF", self.pf, pf_row, pf_col)

    def _create_metric(self, label, variable, row, col):
        """Helper function to cleanly generate the power/PF sub-widgets"""
        container = ttk.Frame(self)
        container.grid(row=row, column=col, sticky="nsew", padx=10, pady=10)
        
        ttk.Label(container, text=label, font=("Segoe UI", 14, "bold")).pack(anchor="w")
        ttk.Label(container, textvariable=variable, font=("Consolas", 40, "bold"), foreground="#007ACC", width=10).pack(anchor="w")

    def update_data(self, v, i=None, va=None, w=None, pf=None):
        """
        Updates the UI variables. Features type-checking to safely handle 
        string placeholders (like "N/A") without crashing the .4f numeric formatter.
        """
        self.voltage.set(f"{v:.4f} V" if isinstance(v, float) else str(v))
        self.v_bar["value"] = min(v, 150) if isinstance(v, float) else 0
        
        if self.show_current and i is not None:
            self.current.set(f"{i:.4f} A" if isinstance(i, float) else str(i))
            self.i_bar["value"] = min(i, 1.0) if isinstance(i, float) else 0
            
        if self.show_apparent and va is not None:
            self.apparent.set(f"{va:.4f} VA" if isinstance(va, float) else str(va))
            
        if self.show_real and w is not None:
            self.real.set(f"{w:.4f} W" if isinstance(w, float) else str(w))
            
        if self.show_pf and pf is not None:
            self.pf.set(f"{pf:.4f}" if isinstance(pf, float) else str(pf))

class PowerSystemGUI:
    """Main Application Class managing the UI loop, serial communications, and data logging."""
    def __init__(self, root):
        self.root = root
        self.root.title("Power Transmission Dashboard" + (" [SIMULATION MODE]" if SIM_MODE else ""))
        
        self.root.geometry("1200x800") 
        
        # State variables
        self.serial_port = None
        self.is_running = True
        self.is_recording = False
        self.csv_filename = None
        
        self.serial_buffer = []
        self.last_rx_time = time.time()
        
        # System status flags
        self.system_ready = False
        self.is_streaming = False
        
        # Dictionary mapping Arduino serial flags to full UI alerts
        self.message_map = {
            "warning0": ("Limit switch broken", "WARNING"),
            "warning1": ("I2C Failure Reset system", "WARNING"),
            "error1": ("Register configuration failed Reset System!", "ERROR"),
            "warning2": ("Home motor enable switch", "WARNING"),
            "warning3": ("Lid or Doors open, Close to continue", "WARNING"),
            "warning4": ("Home all fault selector switches", "WARNING"),
            "warning5": ("Home power factor select switch", "WARNING"),
            "error2": ("Generator Not Stable Reset System!", "ERROR"),
            "error3": ("Dangerous Voltage Detected: Check Chip Health and reset!", "ERROR"),
            "error4": ("Fault detected on Gen bus", "ERROR"),
            "error5": ("Fault detected on HV bus", "ERROR"),
            "error6": ("Fault detected on Industrial bus", "ERROR"),
            "error7": ("Fault detected on Residential bus", "ERROR"),
            "warning6": ("System entering safe mode clear faults to return to normal operation", "WARNING"),
            "error8": ("Unknown or generator fault occurred inspect and restart system!", "ERROR"),
            "error9": ("Second fault detected", "ERROR")
        }
        
        self.status_var = tk.StringVar(value="System Initializing...")
        self.record_btn_text = tk.StringVar(value="Start Recording")

        self._setup_ui()
        self._start_data_thread()

    def _setup_ui(self):
        """Builds the main Tkinter grid, frames, and components."""
        header = ttk.Frame(self.root, padding="15")
        header.pack(fill=tk.X, side=tk.TOP)
        ttk.Label(header, text="Grid Monitoring System", font=("Segoe UI", 28, "bold")).pack(side=tk.LEFT)
        ttk.Label(header, textvariable=self.status_var, font=("Consolas", 14)).pack(side=tk.RIGHT)

        footer = ttk.Frame(self.root, padding="15")
        footer.pack(fill=tk.X, side=tk.BOTTOM)
        
        self.rec_btn = tk.Button(footer, textvariable=self.record_btn_text, command=self._toggle_recording, 
                                 font=("Segoe UI", 16, "bold"), bg="#dddddd", height=2)
        self.rec_btn.pack(fill=tk.X, pady=(0, 10))

        # Include developer tools if running in SIM_MODE
        if SIM_MODE:
            sim_frame = ttk.LabelFrame(footer, text="Simulation Tools", padding="10")
            sim_frame.pack(fill=tk.X)
            
            sim_font = ("Segoe UI", 14)
            
            tk.Button(sim_frame, text="1. Send 'READY' Handshake", font=sim_font, 
                      command=lambda: self._handle_serial_line("ready")).pack(side=tk.LEFT, padx=(0, 20), pady=5)
            
            ttk.Label(sim_frame, text="2. Inject Fault:", font=sim_font).pack(side=tk.LEFT, padx=(0, 5))
            
            self.sim_error_var = tk.StringVar(value="warning1")
            error_dropdown = ttk.Combobox(sim_frame, textvariable=self.sim_error_var, values=list(self.message_map.keys()), 
                                          state="readonly", width=20, font=sim_font)
            error_dropdown.pack(side=tk.LEFT, padx=(0, 10))
            
            tk.Button(sim_frame, text="Send Fault", font=sim_font, 
                      command=lambda: self._handle_serial_line(self.sim_error_var.get())).pack(side=tk.LEFT, pady=5)

        # Alerts Terminal
        alert_frame = ttk.LabelFrame(self.root, text="System Alerts", padding="10")
        alert_frame.pack(fill=tk.X, padx=10, pady=5, side=tk.BOTTOM)
        
        self.alert_text = tk.Text(alert_frame, height=4, state=tk.DISABLED, bg="#2b2b2b", fg="#ffffff", font=("Consolas", 14))
        self.alert_text.pack(fill=tk.X)
        self._log_alert("Waiting for Arduino Diagnostics (Waiting for READY)...", "WARNING")

        # Main Dashboard Layout Area
        dashboard = ttk.Frame(self.root)
        dashboard.pack(fill=tk.BOTH, expand=True, padx=10, pady=5, side=tk.TOP)

        top_row_frame = ttk.Frame(dashboard)
        top_row_frame.pack(fill=tk.BOTH, expand=True)
        
        bottom_row_frame = ttk.Frame(dashboard)
        bottom_row_frame.pack(fill=tk.BOTH, expand=True)

        top_row_frame.columnconfigure(0, weight=1, uniform="top_col")
        top_row_frame.columnconfigure(1, weight=1, uniform="top_col")
        top_row_frame.rowconfigure(0, weight=1)

        bottom_row_frame.columnconfigure(0, weight=1)
        bottom_row_frame.columnconfigure(1, weight=2) 
        bottom_row_frame.rowconfigure(0, weight=1)

        # Instantiate BusPanels and place them in the grid
        self.buses = {}
        
        self.buses["Generator Bus"] = BusPanel(top_row_frame, "Generator Bus", show_apparent=True, show_real=True)
        self.buses["Residential Bus"] = BusPanel(top_row_frame, "Residential Bus", show_apparent=True)
        
        self.buses["High Voltage Bus"] = BusPanel(bottom_row_frame, "High Voltage Bus", show_current=False, show_apparent=False)
        self.buses["Industrial Bus"] = BusPanel(bottom_row_frame, "Industrial Bus", show_apparent=True, show_real=True, show_pf=True, wide_layout=True)

        self.buses["Generator Bus"].grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        self.buses["Residential Bus"].grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
        
        self.buses["High Voltage Bus"].grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        self.buses["Industrial Bus"].grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

    def _log_alert(self, message, level="WARNING"):
        """Thread-safe method to push timestamped, color-coded strings to the alerts terminal."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        def update_text():
            self.alert_text.config(state=tk.NORMAL)
            self.alert_text.tag_config("ERROR", foreground="#ff4444", font=("Consolas", 14, "bold"))
            self.alert_text.tag_config("WARNING", foreground="#ffcc00")
            self.alert_text.tag_config("INFO", foreground="#a3be8c")
            self.alert_text.insert(tk.END, f"[{timestamp}] {message}\n", level)
            self.alert_text.see(tk.END) 
            self.alert_text.config(state=tk.DISABLED)
            
        self.root.after(0, update_text)

    def _toggle_recording(self):
        """Creates a new CSV file with headers if toggled ON, saves and closes if toggled OFF."""
        if not self.is_recording:
            self.is_recording = True
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.csv_filename = f"GridData_{timestamp}.csv"
            
            with open(self.csv_filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                headers = ["Timestamp", "Gen_V", "HV_V", "Res_V", "Ind_V", "Gen_I", "Res_I", "Ind_I", "Gen_VA", "Res_VA", "Ind_VA", "Gen_W", "Ind_W", "Ind_PF"]
                writer.writerow(headers)
            
            self.record_btn_text.set(f"STOP RECORDING ({self.csv_filename})")
            self.rec_btn.config(bg="#ffcccc", fg="red")
        else:
            self.is_recording = False
            self.record_btn_text.set("Start Recording")
            self.rec_btn.config(bg="#dddddd", fg="black")
            self.status_var.set(f"Saved: {self.csv_filename}")

    def _log_data_to_csv(self, row_data):
        """Appends a new row of data to the CSV file if recording is active."""
        if self.is_recording and self.csv_filename:
            try:
                with open(self.csv_filename, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    # Prepend a high-resolution timestamp
                    row = [datetime.now().strftime("%H:%M:%S.%f")[:-3]] + row_data
                    writer.writerow(row)
            except Exception as e:
                pass

    def _start_data_thread(self):
        """Spawns the background daemon thread to handle blocking IO (Serial or Simulation)."""
        if SIM_MODE:
            thread = threading.Thread(target=self._run_simulation)
        else:
            thread = threading.Thread(target=self._run_serial_connection)
        
        thread.daemon = True 
        thread.start()

    def _run_simulation(self):
        """Generates bounded random arrays simulating the Arduino payload for offline testing."""
        self.status_var.set("Simulating Local Data...")
        while self.is_running:
            if self.system_ready:
                self._handle_serial_line("begin")
                
                # Simulates the 10-value payload including the missing hardware flags ("N/A")
                simulated_buffer = [
                    69.0 + random.uniform(-5.0, 5.0),   
                    138.0 + random.uniform(-10.0, 10.0),  
                    115.0 + random.uniform(-8.0, 8.0),  
                    38.0 + random.uniform(-2.0, 2.0),
                    0.8 + random.uniform(-0.1, 0.1), 
                    0.3 + random.uniform(-0.1, 0.1), 
                    0.6 + random.uniform(-0.1, 0.1), 
                    "N/A",  
                    "N/A",  
                    "N/A" 
                ]
                for val in simulated_buffer:
                    self._handle_serial_line(str(val))
            time.sleep(0.5)

    def _run_serial_connection(self):
        """
        Actively polls USB COM ports for an Arduino. Manages the connection, reads lines,
        and enforces the 5.0s software watchdog timer to detect dropped connections.
        """
        while self.is_running:
            port = None
            ports = serial.tools.list_ports.comports()
            
            # Identify likely Arduino ports
            for p in ports:
                if "usbserial" in p.device.lower() or "USB Serial" in p.description:
                    port = p.device
                    break
                    
            if not port:
                for p in ports:
                    if "Arduino" in p.description or "usbmodem" in p.device:
                        port = p.device
                        break
            
            if port:
                try:
                    self.serial_port = serial.Serial(port, 9600, timeout=1)
                    self.status_var.set(f"Connected: {port}")
                    self.last_rx_time = time.time()
                    
                    while self.is_running:
                        if self.serial_port.in_waiting:
                            line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                            self._handle_serial_line(line)
                        else:
                            # WATCHDOG TIMER: Trips if 5.0s elapse without data while stream is active
                            if self.system_ready and self.is_streaming and (time.time() - self.last_rx_time > 5.0):
                                self.system_ready = False
                                self.is_streaming = False
                                self.serial_buffer = []
                                self.status_var.set("Warning: Values Frozen")
                                self._log_alert("Data stream lost (5s timeout). Values frozen.", "WARNING")
                            time.sleep(0.01)
                            
                except serial.SerialException:
                    # Handles physical USB disconnects
                    self.status_var.set("Warning: Disconnected - Values Frozen")
                    self.serial_port = None
                    self.system_ready = False 
                    self.is_streaming = False
                    self.serial_buffer = [] 
                    self._log_alert("USB Disconnected. Dashboard frozen until reconnected...", "WARNING")
                    
                except Exception as e:
                    self.status_var.set(f"Error: {e}")
            else:
                self.status_var.set("Searching for Arduino...")
            
            time.sleep(2)

    def _handle_serial_line(self, line):
        """
        Parses incoming text from the serial port. Intercepts handshakes ("BEGIN", "READY"),
        system alerts, and filters string placeholders before adding values to the buffer.
        """
        line_clean = line.strip().upper() # Clean the text to catch 'n/a' or 'N/A'
        line_lower = line.lower()
        self.last_rx_time = time.time()
        
        # Frame-Alignment Handshake
        if "begin" in line_lower:
            if not self.system_ready or not self.is_streaming:
                self.system_ready = True
                self.is_streaming = True
                self.status_var.set("Connected and Streaming") 
                self._log_alert("Live data stream synced and running.", "INFO")

            # Flushes the buffer to guarantee perfect 10-value frame alignment    
            self.serial_buffer = [] 
            return

        # Initial Boot Handshake    
        if "ready" in line_lower:
            self.system_ready = True
            self.is_streaming = False # Ensures Watchdog stays asleep during boot phase
            self.status_var.set("Connected. Waiting for stream...") 
            self._log_alert("Arduino ready. Waiting for data...", "INFO")
            self.serial_buffer = [] 
            return  

        # Checks incoming strings against known hardware error flags
        for key, (msg, level) in self.message_map.items():
            if key in line_lower:
                self._log_alert(f"[{key.upper()}] {msg}", level)
                self.serial_buffer = [] 
                
                if level == "ERROR":
                    self.system_ready = False
                    self.is_streaming = False 
                return

        # Data ingestion and buffering
        if self.system_ready and self.is_streaming:
            # Explicitly checks for N/A before trying to convert to math numbers
            if line_clean == "N/A":
                self.serial_buffer.append("N/A")
            else:
                try:
                    value = float(line)
                    self.serial_buffer.append(value)
                except ValueError:
                    pass
            
            # Once the 10-value payload is complete, trigger processing
            if len(self.serial_buffer) == 10:
                self._process_packet(self.serial_buffer)
                self.serial_buffer = [] 

    def _process_packet(self, vals):
        """
        Unpacks the validated 10-value payload, calculates Apparent Power (VA) locally,
        and executes thread-safe UI rendering via the Tkinter event loop.
        """
        # Unpack the exact 10 values
        gen_v, hv_v, ind_v, res_v, ind_i, res_i, gen_i, gen_w, ind_w, pf = vals

        # Local Math Calculations
        gen_va = gen_v * gen_i
        res_va = res_v * res_i
        ind_va = ind_v * ind_i

        def update_dash():
            """Wrapper function safely called by the main thread event loop."""
            self.buses["Generator Bus"].update_data(v=gen_v, i=gen_i, va=gen_va, w=gen_w)
            self.buses["Residential Bus"].update_data(v=res_v, i=res_i, va=res_va)
            
            self.buses["High Voltage Bus"].update_data(v=hv_v) 
            self.buses["Industrial Bus"].update_data(v=ind_v, i=ind_i, va=ind_va, w=ind_w, pf=pf)
        
        # THREAD SAFETY: Uses .after(0, func) to hand the payload back to the main Tkinter thread
        self.root.after(0, update_dash)

        # The N/A strings will natively pipe right into the CSV log without breaking
        log_row = [gen_v, hv_v, res_v, ind_v, gen_i, res_i, ind_i, gen_va, res_va, ind_va, gen_w, ind_w, pf]
        self._log_data_to_csv(log_row)

if __name__ == "__main__":
    root = tk.Tk()
    
    try:
        style = ttk.Style()
        style.theme_use('clam')
        
        style.configure("Blue.Horizontal.TProgressbar", 
                        background="#007ACC", 
                        troughcolor="#dddddd", 
                        bordercolor="#cccccc", 
                        lightcolor="#007ACC", 
                        darkcolor="#007ACC")
    except:
        pass
        
    app = PowerSystemGUI(root)
    root.mainloop()