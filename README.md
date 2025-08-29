# Robot Trajectory Estimation and Control Interface
**Version:** 1 — **Last updated:** 2025-08-29

**Author:** Baptiste Poncet   

**Contact:** baptiste.poncet@etu.unilim.fr (ENSIL-ENSCI)

# GENERAL INFORMATION
This project implements a full robot trajectory estimation and control system. This project was carried out in two phases, a prototyping phase and a final phase.

The prototype phase uses :
 - Real-time motor control via Arduino and CANopen protocol
 - Serial LoRa-based communication
 - Visual odometry from a USB webcam
 - Extended Kalman Filter for fused trajectory estimation
 - A GUI in Python using Tkinter and a Flask API

The final phase uses :
 - Jetson Nano embedded computer
 - Real-time motor control using CANopen protocol via Waveshare USB-CAN-A module
 - VNC server
 - Visual odometry from a USB webcam
 - LiDAR
 - Onboard screen
 - ROS2 node

## Dataset Title
This dataset describes robot trajectories in a two-phase project. The prototype phase uses fused wheel and visual odometry with real-time control via a GUI (Tkinter), a Flask API, and serial LoRa communication. Motor commands are handled via CANopen using an Arduino control board. The final phase is implemented on a Jetson Nano, integrating LiDAR, ROS2, visual odometry, and wheel encoders, with a VNC server enabling remote communication.

# PROTOTYPE PHASE
## METHODOLOGICAL INFORMATION

### Environmental / experimental conditions

#### Hardware 
- Arduino Uno
- Motor iSV2-CAN x2
- LoRa module E32 433T20D
- CAN controller MCP2515 + TJA1050
- Intel monocular USB webcam 
   
#### Software 
- OS : Windows 11
- IDE : Visual Studio Code, Arduino IDE
- Arduino IDE Version : 1.8.18
- Python Version : 3.10.10
- External interaction: Serial COM port (LoRa module) and Flask server API
- For Flask utilisation, a Samsung S23FE with HTTP shortcut 3.32.0

**Software versions required:**
- All python scripts were written and tested using Visual Studio Code on Windows 11. Python 3.10.10 (via Miniconda).
- Dependencies include:
  - `flask`
  - `numpy`
  - `matplotlib`
  - `Pillow`
  - `pyserial`
  - `tkinter`
  - `filterpy`
- Arduino script was written and tested using Arduino IDE 1.8.18 on Windows 11.
- Dependencies include:
  - `SPI.h`
  - `mcp_can.h by coryjfowler`

### Description of the sources and methods used to collect and generate the data

#### Serial Communication via LoRa
- A LoRa module connected to the Arduino transmits serial data to a corresponding LoRa module connected to the PC.
- The serial data includes:
  - Robot commands (e.g. `SPEED <L> <R>`)
  - Feedback from encoders
  - Trajectory update triggers (e.g., [DIST] message)

#### Visual Odometry from USB Camera
- A monocular Intel USB webcam is connected to the PC
- Images are captured using OpenCV
- The visual odometry pipeline includes:
  - Grayscale conversion
  - Frame-to-frame pose estimation
- Triggered by:
  - Distance messages `[DIST]`
  - Periodic updates (every 9 seconds)
- The estimated camera trajectory is plotted in green

#### Extended Kalman Filter (Sensor Fusion)

- Implemented using the `filterpy` library, especially `filterpy.kalman`
- Input :
  - Odometry from encoders (wheel movement) every
  - Visual position estimates (when VO update)
- Output:
  - Fused position estimate (x, y)
  - Displayed in red on the trajectory plot
- Updates triggered by:
  - Distance messages `[DIST]`
  - Periodic updates (every 9 seconds)

#### Motor Control and CANOpen Protocol
- Serial commands `SPEED <L> <R>` are parsed by the Arduino and translated into CANopen messages
- Direction handling :
  - Left motor: Positive = forward / Negative = backward
  - Right motor: Positive = backward / Negative = forward
- CAN communication :
  - Uses MCP2515 CAN controller
- Encoders send position data via CAN :
  - Using TPDO (Transmit Process Data Object)
  - Data format: `EKF <delta_d> <delta_theta>`
   - `<delta_d>` is the distance travelled since the last transmission in meter. Distance calculated using the formula :   
      `(accumulated_delta_L + accumulated_delta_R) / 2.0`
   - `<delta_theta>` is the angle travelled since the last transmission in radians. Angle calculated using the formula :   
      `(accumulated_delta_R - accumulated_delta_L) / WHEEL_BASE` where WHEEL_BASE = 30cm
 
#### Graphical User Interface (GUI) and Command Handling
- GUI built with `tkinter`
- Flask API allows external control (e.g., via smartphone + HTTP Shortcuts)
- Flask server runs in the background on the PC
- Commands visible in log:
  - Initialization of Arduino Board
  - Error
  - `EKF <delta_d> <delta_theta>`
  - `[DIST]`
  - `SPEED <L> <R>`
  - Confirmation of Speed from Arduino
  - communication via Flask
 
## INSTALLATION AND LAUNCH
### Software Prerequisites

- Python 3.10.10
- Windows 11 (tested)
- Visual Studio Code (recommended)
- Arduino IDE 1.8.18 (tested)
- Device with a flask interface (HTTP Shortcut tested)

### Installing Dependencies
- Python libraries : ```pip install flask numpy matplotlib Pillow pyserial filterpy```
- Arduino libraries :
    - SPI.h
    - mcp_can.h by coryjfowler

### Setup Instructions
- Connect a LoRa module to a COM port of the arduino board
- Connect another LoRa module to a COM port of the PC
- Connect the USB webcam to the PC running the Python interface
- Make sure both the PC and mobile device are connected to the same Wi-Fi network to enable Flask communication


### Launch
- For the PC part : ```python main.py```
- Upload the Arduino code in the arduino board

### Usage
- The GUI interface opens automatically
- Flask server starts in the background
- Check serial connection via status indicators
- Send speed commands in the format: ```SPEED <L> <R>```

### Troubleshooting:

- If COM port connection fails, verify the port number and ensure the LoRa module is properly connected
- If camera is not detected, check USB connection and ensure no other applications are using the webcam, in the `vision_utils.py` file, adjust the CAMERA_INDEX constant
- For Flask server issues, ensure the port is not already in use by another application

## Codebase Structure & Outputs
### Outputs

- Trajectories are exported as `.csv` via `trajectory.py`
- Visual Odometry (green) and EKF (red) curves are plotted in real-time in the GUI
- Data updates are triggered by either `[DIST]` messages or time-based intervals (9 seconds)

### Code Structure Summary

- The main entry point is `main.py`, which launches the GUI and background threads
- Arduino code is located in `Arduino_code_robot/`, separated by version (V1.8 and V2.1)
- Project is modularized into:
  - `gui/`: interface
  - `communication/`: LoRa and Flask logic
  - `logic/`: trajectory, vision, EKF


### File Overview

- `main.py`: Entry point of the application
- `main_window.py`: Defines the GUI layout and event handling
- `camera_window.py`: Display the camera and trajectory plot on the GUI
- `flask_server.py`: API server for command input
- `serial_comm.py`: Manages COM port connection and communication
- `trajectory.py`: Data model and trajectory export
- `vision_utils.py`: Visual Odometry calculations
- `ekf_filter` : Extended Kalman Filter calculations
- `styles.py`:  Applies light/dark theme recursively to all GUI widgets
- `Motor_Arduino_V1_8.ino` : Arduino code estimated the camera trajectory only
- `Motor_Arduino_V2_1.ino` : Arduino code estimated the camera trajectory and the trajectory from Kalman filter, more stable than the V1_8.

### Directory structure:
```text
README.md
interface.gitignore
Arduino_code_robot\
|---Motor_Arduino_V1_8\
|---|---Motor_Arduino_V1_8.ino
|---Motor_Arduino_V2_1\
|---|---Motor_Arduino_V2_1.ino
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
|   |---ekf_filter.py
|---resources\
|   |---icons\
```

# FINAL PHASE
## METHODOLOGICAL INFORMATION

### Environmental / experimental conditions

#### Hardware 
- Jetson Nano
- Motor iSV2-CAN x2
- Screen OLED
- Waveform USB-CAN-A module
- Intel monocular USB webcam
- LiDAR
   
#### Software 
- OS : Linux Ubuntu 20.04
- External interaction: VNC Server running on the jetson nano
- ROS2 foxy

**Software versions required:**
All robot control and perception modules are implemented as ROS2 nodes.
- Python 3.8.10 (default on Ubuntu 20.04).
Python dependencies (ROS2 node development):
- `rclpy` (ROS2 Python client library)
- `opencv-python` (camera input & visual odometry)
- `filterpy`


### Description of each ROS2 node

#### Motor Node (`motor_controller`)

- **Role**: Acts as a bridge between high-level velocity commands and low-level CANOpen motor messages.  
- **Input**:  
  - Topic: `/motor_cmd` (`std_msgs/String`)  
  - Command format:  
    ```
    SPEED <right> <left>
    ```  
    where `<right>` and `<left>` are integers representing target velocities.  
- **Output**:  
  - Topic: `/can_tx` (`can_msgs/Frame`)  
  - CAN frames (SDO `TargetVelocity`) sent to the motors:  
    - ID `0x601` → right motor  
    - ID `0x602` → left motor  
- **Implementation details**:  
  - Parses and validates string commands.  
  - Converts target speeds into CANOpen SDO frames (`0x60FF:00` index).  
  - Publishes CAN frames with the correct DLC (8 bytes).  
  - Logs each transmitted command using ROS2 logger.  

#### CAN Bridge Node (`can_bridge`)

- **Role**: Provides a bidirectional bridge between ROS2 topics and the CAN bus through a USB-to-CAN serial adapter.  
- **Inputs**:  
  - Topic: `/can_tx` (`can_msgs/Frame`)  
  - Receives CAN frames from other ROS2 nodes (e.g., `motor_controller`) and forwards them to the CAN bus over the serial port.  
- **Outputs**:  
  - Topic: `/can_rx` (`can_msgs/Frame`)  
  - Publishes all CAN frames received from the serial interface to ROS2, making them available to other nodes.  
- **Implementation details**:  
  - Uses a USB serial connection (`/dev/ttyUSB0`, 115200 baud).  
  - Frames are wrapped/unwrapped with a custom protocol (`0xAA … 0x55` markers).  
  - Performs motor initialization on startup by sending a sequence of CANOpen commands (NMT + mode configuration).  
  - Periodically reads from the serial port and publishes decoded CAN frames.  
  - Logs all transmitted (`[TX]`) and received (`[RX]`) frames for debugging.  



#### Extended Kalman Filter (Sensor Fusion)

- Implemented using the `filterpy` library, especially `filterpy.kalman`
- Input :
  - Odometry from encoders (wheel movement) every
  - Visual position estimates (when VO update)
- Output:
  - Fused position estimate (x, y)
  - Displayed in red on the trajectory plot
- Updates triggered by:
  - Distance messages `[DIST]`
  - Periodic updates (every 9 seconds)

#### Motor Control and CANOpen Protocol
- Serial commands `SPEED <L> <R>` are parsed by the Arduino and translated into CANopen messages
- Direction handling :
  - Left motor: Positive = forward / Negative = backward
  - Right motor: Positive = backward / Negative = forward
- CAN communication :
  - Uses MCP2515 CAN controller
- Encoders send position data via CAN :
  - Using TPDO (Transmit Process Data Object)
  - Data format: `EKF <delta_d> <delta_theta>`
   - `<delta_d>` is the distance travelled since the last transmission in meter. Distance calculated using the formula :   
      `(accumulated_delta_L + accumulated_delta_R) / 2.0`
   - `<delta_theta>` is the angle travelled since the last transmission in radians. Angle calculated using the formula :   
      `(accumulated_delta_R - accumulated_delta_L) / WHEEL_BASE` where WHEEL_BASE = 30cm
 
#### Graphical User Interface (GUI) and Command Handling
- GUI built with `tkinter`
- Flask API allows external control (e.g., via smartphone + HTTP Shortcuts)
- Flask server runs in the background on the PC
- Commands visible in log:
  - Initialization of Arduino Board
  - Error
  - `EKF <delta_d> <delta_theta>`
  - `[DIST]`
  - `SPEED <L> <R>`
  - Confirmation of Speed from Arduino
  - communication via Flask
 
## INSTALLATION AND LAUNCH
### Software Prerequisites

- Python 3.10.10
- Windows 11 (tested)
- Visual Studio Code (recommended)
- Arduino IDE 1.8.18 (tested)
- Device with a flask interface (HTTP Shortcut tested)

### Installing Dependencies
- Python libraries : ```pip install flask numpy matplotlib Pillow pyserial filterpy```
- Arduino libraries :
    - SPI.h
    - mcp_can.h by coryjfowler

### Setup Instructions
- Connect a LoRa module to a COM port of the arduino board
- Connect another LoRa module to a COM port of the PC
- Connect the USB webcam to the PC running the Python interface
- Make sure both the PC and mobile device are connected to the same Wi-Fi network to enable Flask communication


### Launch
- For the PC part : ```python main.py```
- Upload the Arduino code in the arduino board

### Usage
- The GUI interface opens automatically
- Flask server starts in the background
- Check serial connection via status indicators
- Send speed commands in the format: ```SPEED <L> <R>```

### Troubleshooting:

- If COM port connection fails, verify the port number and ensure the LoRa module is properly connected
- If camera is not detected, check USB connection and ensure no other applications are using the webcam, in the `vision_utils.py` file, adjust the CAMERA_INDEX constant
- For Flask server issues, ensure the port is not already in use by another application

## Codebase Structure & Outputs
### Outputs

- Trajectories are exported as `.csv` via `trajectory.py`
- Visual Odometry (green) and EKF (red) curves are plotted in real-time in the GUI
- Data updates are triggered by either `[DIST]` messages or time-based intervals (9 seconds)

### Code Structure Summary

- The main entry point is `main.py`, which launches the GUI and background threads
- Arduino code is located in `Arduino_code_robot/`, separated by version (V1.8 and V2.1)
- Project is modularized into:
  - `gui/`: interface
  - `communication/`: LoRa and Flask logic
  - `logic/`: trajectory, vision, EKF


### File Overview

- `main.py`: Entry point of the application
- `main_window.py`: Defines the GUI layout and event handling
- `camera_window.py`: Display the camera and trajectory plot on the GUI
- `flask_server.py`: API server for command input
- `serial_comm.py`: Manages COM port connection and communication
- `trajectory.py`: Data model and trajectory export
- `vision_utils.py`: Visual Odometry calculations
- `ekf_filter` : Extended Kalman Filter calculations
- `styles.py`:  Applies light/dark theme recursively to all GUI widgets
- `Motor_Arduino_V1_8.ino` : Arduino code estimated the camera trajectory only
- `Motor_Arduino_V2_1.ino` : Arduino code estimated the camera trajectory and the trajectory from Kalman filter, more stable than the V1_8.

### Directory structure:
```text
README.md
interface.gitignore
Arduino_code_robot\
|---Motor_Arduino_V1_8\
|---|---Motor_Arduino_V1_8.ino
|---Motor_Arduino_V2_1\
|---|---Motor_Arduino_V2_1.ino
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
|   |---ekf_filter.py
|---resources\
|   |---icons\
```
