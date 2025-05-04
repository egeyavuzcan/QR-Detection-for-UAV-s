# 🤖 QR Detection for UAVs

A Python package for UAV QR code detection and mission planning.

---
## 📦 Installation

```bash
git clone "https://github.com/egeyavuzcan/QR-Detection-for-UAV-s.git"
cd "QR-Detection-for-UAV-s"
pip install -e .
```



---
## 📦 Dependencies
- Python 3.6+
- OpenCV 
- NumPy
- DroneKit
- pymavlink

---
## 🚀 Usage

### 🔍 QR Detection

```bash
python src/qr_algo.py --source "0" --width 800 --height 800
```
- `--source`: camera device index ("0") or GStreamer pipeline string
- `--width`, `--height`: output resolution for unwarped QR view

Press **q** to exit the video display.

### 🛫 Mission Planner

```bash
python src/qr_mission.py --connection "127.0.0.1:14550" --side-length 30 --altitude 20
```
- `--connection`: MAVLink target (e.g., simulator at `127.0.0.1:14550`)
- `--side-length`: half side length (meters) of square mission
- `--altitude`: takeoff altitude in meters

The script will:
1. 📈 Connect to the vehicle
2. 🗺️ Clear/upload a square mission
3. ✈️ Arm and take off
4. 📍 Fly waypoints in AUTO mode
5. 🛬 RTL on completion
1. Connect to the vehicle
2. Clear/upload a square mission
3. Arm and take off
4. Fly waypoints in AUTO mode
5. RTL on completion

---
## 🧠 Algorithm Overview

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
## 🧩 Frameworks and Libraries
- **OpenCV**: Real-time computer vision (QR detection, image warp)
- **NumPy**: Numerical operations on image arrays
- **DroneKit**: High-level Python API for MAVLink vehicles
- **pymavlink**: Low-level MAVLink message construction

