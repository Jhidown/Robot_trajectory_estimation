README File — General — Version: 0.3 (2025-17-07)

This README file was generated on 2025-07-07 by Baptiste Poncet.

Last updated on: 2025-07-07

# GENERAL INFORMATION

## Dataset title:

Robot Control Interface - GUI with Tkinter, Flask API and LoRa Serial Communication

## Contact address:

baptiste.poncet@etu.unilim.fr
ENSIL-ENSCI 


# METHODOLOGICAL INFORMATION

## Environmental / experimental conditions:

- OS : Windows 11
- IDE : Visual Studio Code
- Python Version : 3.10.10
- External interaction: Serial COM port (LoRa module) and Flask server API

## Description of the sources and methods used to collect and generate the data:

- Serial data is received from a microcontroller via a LoRa module (USB virtual COM port).
- Trajectory data is derived from visual odometry (VO).
- Visual data is captured from a Intel monocular USB webcam using OpenCV/PIL and processed to infer movement.
- API commands are sent to a Flask server from an external device (e.g. smartphone).
- Movement commands are sent from the GUI to the robot via serial communication in the format: `SPEED <L> <R>`.
  - The first integer `<L>` controls the **left motor**.
  - The second integer `<R>` controls the **right motor**.
  - Positive values move the left motor forward, negative values reverse it.  
  - For the **right motor**, the direction is inverted: positive values move it **backward**, and negative values move it **forward**.
- These commands are logged in real time with timestamps and displayed in the command history window.

# INSTALLATION AND LAUNCH
## Prerequisites:

- Python 3.10.10
- Windows 11 (tested)
- Visual Studio Code (recommended)
- Available COM port for LoRa module
- Intel monocular USB webcam

## Installing dependencies:
```pip install flask numpy matplotlib Pillow pyserial```

## Setup
- Connect the LoRa module to a COM port
- Connect the USB webcam
- Check the COM port in system settings

## Launch
```python main.py```

## Usage
- The GUI interface opens automatically
- Flask server starts in the background
- Check serial connection via status indicators
- Send commands in the format: ```SPEED <L> <R>```

## Troubleshooting:

- If COM port connection fails, verify the port number and ensure the LoRa module is properly connected
- If camera is not detected, check USB connection and ensure no other applications are using the webcam
- For Flask server issues, ensure the port is not already in use by another application

# DATA AND FILE OVERVIEW
## Data processing methods:

- Theorical trajectories are calculated and updated on each command, and exported via `trajectory.py` as `.csv`
- The camera's trajectory is estimated from images taken every 8 seconds (which corresponds to 30 cm at the robot's speed).
- Visual Odometry is processed live using grayscale frame extraction, frame-to-frame pose estimation, and trajectory plotting
- Flask runs as a background thread to receive external commands securely (via token)
- The dark mode feature dynamically restyles all GUI components recursively

## Quality assurance procedures applied to the data:

- Serial connection feedback is confirmed via visual status labels
- Command acknowledgment is logged with timestamps
- GUI inputs are validated to prevent invalid commands (e.g., wrong speed format)
- Error messages are clearly displayed and tagged in the log

## Other contextual information:

**Software versions required:**

- All scripts were written and tested using Visual Studio Code on Windows 11.
- Dependencies include:
  - `flask`
  - `numpy`
  - `matplotlib`
  - `Pillow`
  - `pyserial`
  - `tkinter` (built-in) 

# DATA & FILE OVERVIEW

## File naming convention:

- `main.py`: Entry point of the application
- `main_window.py`: GUI logic (Tkinter)
- `camera_window.py`: Visual odometry and camera handling
- `flask_server.py`: API server for command input
- `serial_comm.py`: Manages COM port connection and communication
- `trajectory.py`: Data model and trajectory export
- `vision_utils.py`: Visual Odometry calculations
- `styles.py`:  Applies light/dark theme recursively to all GUI widgets

## Directory structure:
```text
README.md
interface_project\
|
|---main.py
|---gui\
|   |---__init__.py
|   |---main_window.py
|   |---camera_window.py
|   |---styles
|---communication\
|   |---__init__.py
|   |---flask_server.py
|   |---serial_comm.py
|---logic\
|   |---__init__.py
|   |---trajectory.py
|   |---vision_utils.py
|---resources\
|   |---icons\
```

