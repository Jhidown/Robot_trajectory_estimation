README File — General — Version: 0.4 (2025-24-07)

This README file was generated on 2025-07-07 by Baptiste Poncet.

Last updated on: 2025-24-07

# GENERAL INFORMATION

## Dataset title:
Robot trajectory estimation

Robot Control Interface - GUI with Tkinter, Flask API and LoRa Serial Communication
Arduino control board - LoRa Serial Communication and CANopen Communication 

## Contact address:

baptiste.poncet@etu.unilim.fr
ENSIL-ENSCI 


# METHODOLOGICAL INFORMATION

## Environmental / experimental conditions:

### Hardware :
- Arduino Uno
- Motor iSV2-CAN x2
- LoRa module E32 433T20D
- CAN controller MCP2515 + TJA1050
- Intel monocular USB webcam 
   
### Software :
- OS : Windows 11
- IDE : Visual Studio Code, Arduino IDE
- Arduino IDE Version : 1.8.18
- Python Version : 3.10.10
- External interaction: Serial COM port (LoRa module) and Flask server API
- For Flask utilisation, a Samsung S23FE with HTTP shorcut 3.32.0

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
- The Arduino board converts serial instructions into CAN instructions sent to the motors via a CANopen protocol and convert the encoders values from CANopen to serial when the robot moves 30cm (calculated for wheels with a diameter of 15 cm).
- The encoders values are in the format : `Encoder <ID> angle: <value>`.
    - `<ID>` is the CANopen ID of the register read to obtain the encoder values.
    - `<VALUES>` is the encoders values in cumulative degree.
- Encoders are read with a TPDO process.


# INSTALLATION AND LAUNCH
## Software Prerequisites:

- Python 3.10.10
- Windows 11 (tested)
- Visual Studio Code (recommended)
- Arduino IDE 1.8.18 (tested)
- Device with a flask interface (HTTP Shortcut tested)

## Installing dependencies:
- Python librairies : ```pip install flask numpy matplotlib Pillow pyserial```
- Arduino librairies :
    - SPI.h
    - mcp_can.h by coryjfowler

## Setup
- Connect a LoRa module to a COM port of the arduino board
- Connect another LoRa module to a COM port of the PC
- Connect the USB webcam to the PC which run python
- If you want to use flask server, be sure to have the computer and the phone (for example) on the same wifi, and using the same IP


## Launch
- For the PC part : ```python main.py```
- Upload the Arduino code in the arduino board

## Usage
- The GUI interface opens automatically
- Flask server starts in the background
- Check serial connection via status indicators
- Send commands in the format: ```SPEED <L> <R>```

## Troubleshooting:

- If COM port connection fails, verify the port number and ensure the LoRa module is properly connected
- If camera is not detected, check USB connection and ensure no other applications are using the webcam, be sure to change the CAMERA_INDEX in the file `vision_utils.py`
- For Flask server issues, ensure the port is not already in use by another application

# DATA AND FILE OVERVIEW
## Data processing methods:

- Theorical trajectories are calculated and updated on each command, and exported via `trajectory.py` as `.csv`
- The camera's trajectory is estimated from images taken when a message such as `Encoder ID angle ID: number` is received, or every 9 seconds.
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

- All python scripts were written and tested using Visual Studio Code on Windows 11.
- Dependencies include:
  - `flask`
  - `numpy`
  - `matplotlib`
  - `Pillow`
  - `pyserial`
  - `tkinter` (built-in)
- Arduino script was written and tested using Arduino IDE 1.8.18 on Windows 11.
- Depencies include:
  - `SPI.h`
  - `mcp_can.h by coryjfowler`

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
- `Motor_Arduino_V1_8.ino` : Arduino code for the Arduino Uno

## Directory structure:
```text
README.md
interface.gitignore
Arduino_code_robot\
|---Motor_Arduino_V1_8\
|---|---Motor_Arduino_V1_8.ino
interface_project\
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

