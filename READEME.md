# Power Transmission Grid Monitoring Dashboard

![Python](https://img.shields.io/badge/Python-3.8%2B-blue)
![Tkinter](https://img.shields.io/badge/GUI-Tkinter-lightgrey)
![pyserial](https://img.shields.io/badge/Library-pyserial-yellow)

A highly responsive, multi-threaded desktop application built in Python and Tkinter, designed to ingest, visualize, and log real-time telemetry data from an Arduino microcontroller. This project was developed as the User Interface (UI) subsystem for a Senior Design Capstone project focusing on high-voltage power generation and transmission.

## Features

* **Real-Time Telemetry Visualization:** Displays accurate RMS voltage, current, and locally calculated Apparent Power (VA) across four distinct power buses (Generator, High Voltage, Industrial, Residential).
* **Dynamic Progress Bars:** Color-coded `0-150V` and `0-1A` progress bars provide immediate visual feedback and rapid situational awareness.
* **Robust Multi-Threading:** Separates the Tkinter main rendering loop from the blocking serial read operations, ensuring the UI never locks up while receiving high-frequency data.
* **Fault-Tolerant Serial Parsing:** Implements a strict `"BEGIN"` handshake for perfect 10-value frame alignment. Features type-safe filtering that gracefully handles missing hardware metrics (e.g., safely accepting `"N/A"` strings without crashing numerical formatters).
* **Data Logging (CSV):** Includes a toggleable recording module that streams the full dataset, calculated VA, and a high-resolution timestamp to a local, non-volatile `.csv` file for offline analysis.
* **Software Watchdog Timer:** Actively monitors the USB connection. If the data stream drops for more than 5.0 seconds, the dashboard safely freezes the metrics and visually alerts the user to prevent actions based on stale data.
* **System Alerts Terminal:** A built-in terminal that captures automated hardware responses, limit switch trips, and system faults directly from the microcontroller.
* **Simulation Mode:** Includes a `SIM_MODE` framework that generates randomized, bounded data arrays, allowing developers to test and demonstrate the GUI without physical hardware connected.

---

## Prerequisites

To run this application, you will need Python 3.x installed on your machine along with the `pyserial` library. Tkinter is included in the standard Python library, so no separate installation is required for the GUI framework.

```bash
# Install pyserial via pip
pip install pyserial
```

---

## Installation & Usage

**Clone the repository:**
```bash
git clone https://github.com/yourusername/Power-Grid-Dashboard.git
cd Power-Grid-Dashboard
```

**Configure the Dashboard:**
Open `GUI.py` in your preferred text editor. At the top of the file, you will find the configuration block.
* To run with live hardware: Set `SIM_MODE = False`
* To run the software simulator: Set `SIM_MODE = True`

**Run the Application:**
```bash
python GUI.py
```

**Recording Data:**
Click the "Start Recording" button at the bottom of the screen. The system will generate a time-stamped CSV file (e.g., `GridData_YYYY-MM-DD_HH-MM-SS.csv`) in the same directory as the script. Click again to stop and save the file.

---

## Hardware Data Protocol (ICD)

For the dashboard to successfully parse live hardware data, the Arduino must transmit data over the Serial port (9600 baud) following a strict structural format.

### 1. Synchronization Handshake
The Arduino must send the exact string `"BEGIN"` immediately before bursting the telemetry data. This triggers the Python parser to flush its buffer and prepare for exactly 10 incoming values.

### 2. Telemetry Payload
Immediately following `"BEGIN"`, the Arduino must `Serial.println()` exactly **10 values** in the following strict order:

1. **Generator Voltage** (`float`)
2. **High Voltage Bus Voltage** (`float`)
3. **Industrial Bus Voltage** (`float`)
4. **Residential Bus Voltage** (`float`)
5. **Industrial Bus Current** (`float`)
6. **Residential Bus Current** (`float`)
7. **Generator Current** (`float`)
8. **Generator Real Power** (`float` or `"N/A"`)
9. **Industrial Real Power** (`float` or `"N/A"`)
10. **Industrial Power Factor** (`float` or `"N/A"`)

*Note: If your hardware is incapable of calculating Real Power (W) or Power Factor (PF), program the Arduino to print the string `"N/A"`. The Python parser is engineered to safely bypass the numerical formatting and display the text placeholder without crashing.*

### 3. System Alerts & Warnings
The Arduino can trigger visual warnings in the GUI's "System Alerts" terminal by sending specific strings defined in the `message_map` dictionary (e.g., `"warning1"`, `"error3"`).
* `"warning"` strings will output yellow text to the terminal.
* `"error"` strings will output red text, halt the `is_streaming` status, freeze the dashboard, and demand a hardware reset / new `"BEGIN"` handshake.

---

## Troubleshooting

* **Dashboard opens but says "Searching for Arduino...":**
  Ensure your Arduino is plugged in and recognized by your computer. Check that no other applications (like the Arduino IDE Serial Monitor) are currently occupying the COM port.
* **Values are frozen and a 5.0s timeout warning appears:**
  The Watchdog timer tripped because it hasn't received complete data. Verify that your Arduino code is looping fast enough (delays must be under 5.0 seconds total between `"BEGIN"` bursts) and that it is actually sending `"BEGIN"`.
* **Values are displaying in the wrong boxes:**
  Ensure your Arduino `Serial.println()` statements exactly match the 10-value payload sequence listed in the Data Protocol section above.
* **Tkinter fails to run on macOS:**
  Ensure your version of Python was installed with Tcl/Tk support. If using Homebrew, you may need to run `brew install python-tk@3.x`.

---

## License
This project was developed for academic purposes as part of an engineering Capstone design sequence.