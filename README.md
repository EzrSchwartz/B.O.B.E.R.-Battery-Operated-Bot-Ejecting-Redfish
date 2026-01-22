# B.O.B.E.R.-Battery-Operated-Bot-Ejecting-Redfish
# Fishing Drone - Complete System

## Hardware Setup

### ESP32-S3 Connections

**Flight Controller (MSP):**
- ESP32 IO2 (TX) → FC UART RX
- ESP32 IO4 (RX) → FC UART TX
- GND → GND

**SBUS Output:**
- ESP32 IO17 → FC SBUS Input

**Camera (OV2640):**
- D0-D7: IO0, IO12, IO37, IO35, IO14, IO18, IO11, IO33
- XCLK: IO38
- PCLK: IO1
- VSYNC: IO3
- HREF: IO7
- SDA: IO8
- SCL: IO9
- PWDN: IO43
- RESET: IO44
- 3.3V & GND

**INAV FC Configuration:**
- Ports → Enable MSP on UART (115200 baud)
- Receiver → Set SBUS on appropriate UART
- Modes → Configure GPS/Waypoint mode on AUX1

## ESP32 Installation

1. Install CircuitPython 9.x on ESP32-S3
2. Copy `code.py` to CIRCUITPY drive
3. Reset ESP32

## Desktop App (Optional)
```bash
cd web_app
pip install -r requirements.txt
python app.py
```

Open: http://localhost:8000

## Usage

1. Power on drone
2. Connect to WiFi: "FishingDrone" (password: fishing123)
3. Open browser: http://192.168.4.1
4. Click map to plot drop locations
5. Click "Upload to FC" → Mission uploaded to INAV
6. Arm drone and switch to GPS/Waypoint mode
7. Mission starts automatically

## Features

- ✅ Web-based mission planning
- ✅ MSP waypoint upload to INAV
- ✅ Real-time obstacle detection (60fps)
- ✅ Autonomous collision avoidance
- ✅ SBUS override during emergencies
- ✅ Dual-core operation (vision + control)

## Safety

- Always test with props OFF first
- Verify GPS lock before flight
- Keep line of sight
- Test obstacle avoidance on ground

![Alt text for the image](BOBER!.jpeg)
