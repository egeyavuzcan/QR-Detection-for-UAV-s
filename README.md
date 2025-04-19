# QR-Detection-for-UAV-s

A dual-module Python toolkit for Unmanned Aerial Vehicle (UAV) operations:

1. **QR Detection Module (`QR_Algo_updated.py`)**
   - Captures live video streams (e.g., camera index or GStreamer pipeline)
   - Detects and decodes QR codes using OpenCV
   - Applies perspective transformation to unwarp the QR region
   - Displays camera feed and transformed QR view

2. **Mission Planner Module (`Qr_Mission.py`)**
   - Connects to a UAV via MAVLink (DroneKit)
   - Clears or downloads existing missions
   - Plans a square mission of configurable side length
   - Arms and takes off to specified altitude
   - Executes mission in AUTO mode, skipping or exiting on final waypoint
   - Returns to launch site (RTL) and closes connection

---
## Table of Contents
1. [Installation](#installation)
2. [Dependencies](#dependencies)
3. [Usage Examples](#usage-examples)
4. [Algorithm Details](#algorithm-details)
5. [Frameworks and Libraries](#frameworks-and-libraries)
6. [License](#license)

---
## Installation

Clone this repository and install dependencies:

```bash
git clone "https://your-repo/QR-Detection-for-UAV-s.git"
cd "QR-Detection-for-UAV-s"
pip install -r requirements.txt
```

> **Note:** If you don’t have a `requirements.txt`, install:
> ```bash
> pip install opencv-python numpy dronekit pymavlink
> ```

---
## Dependencies
- Python 3.6+
- OpenCV 
- NumPy
- DroneKit
- pymavlink

---
## Usage Examples

### 1. QR Detection Module

```bash
python QR_Algo_updated.py \
  --source "0" \
  --width 800 \
  --height 800
```
- `--source`: camera device index ("0") or GStreamer pipeline string
- `--width`, `--height`: output resolution for unwarped QR view

Press **q** to exit the video display.

### 2. Mission Planner Module

```bash
python Qr_Mission.py \
  --connection 127.0.0.1:14550 \
  --size 50 \
  --altitude 10
```
- `--connection`: MAVLink target (e.g., simulator at `127.0.0.1:14550`)
- `--size`: half side length (meters) of square mission
- `--altitude`: takeoff altitude in meters

The script will:
1. Connect to the vehicle
2. Clear/upload a square mission
3. Arm and take off
4. Fly waypoints in AUTO mode
5. RTL on completion

---
## Algorithm Details

### QRDetector (QR_Algo_updated.py)
1. **Video Capture**: Uses OpenCV `VideoCapture` for live stream input.
2. **Detection & Decoding**: Uses `cv2.QRCodeDetector.detectAndDecode` to find QR contours and extract text.
3. **Perspective Transform**:
   - Maps the 4 detected corner points to a fixed rectangle
   - Computes homography via `cv2.getPerspectiveTransform`
   - Applies warp with `cv2.warpPerspective` to unwarp QR region
4. **Display Loop**: Shows both raw camera feed and transformed view; prints decoded data.

### MissionPlanner (Qr_Mission.py)
1. **Connection**: Establishes MAVLink link via DroneKit’s `connect()`.
2. **Waypoint Computation**:
   - Computes offsets (`dNorth`, `dEast`) in meters converting to lat/lon
   - Implements small-distance approximation on a spherical Earth
3. **Mission Upload**:
   - Creates `MAV_CMD_NAV_TAKEOFF` and 4 `MAV_CMD_NAV_WAYPOINT` commands
   - Adds a final dummy waypoint as end-of-mission signal
4. **Execution**:
   - Arms and takes off in GUIDED mode
   - Switches to AUTO mode for waypoint follow
   - Monitors `vehicle.commands.next`, can skip or exit
5. **RTL & Cleanup**: Commands return-to-launch and closes the vehicle object.

---
## Frameworks and Libraries
- **OpenCV**: Real-time computer vision (QR detection, image warp)
- **NumPy**: Numerical operations on image arrays
- **DroneKit**: High-level Python API for MAVLink vehicles
- **pymavlink**: Low-level MAVLink message construction

