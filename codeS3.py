


# """
# Fishing Drone Autonomous System
# ESP32-S3 Feather with Camera + INAV Integration
# """

# import board
# import busio
# import wifi
# import socketpool
# import json
# import time
# import espcamera
# import _thread
# import struct
# import digitalio

# # ============================================
# # CONFIGURATION
# # ============================================

# # WiFi credentials
# WIFI_SSID = "FishingDrone"
# WIFI_PASSWORD = "fishing123"

# # Pin assignments
# class Pins:
#     # SBUS output to FC
#     SBUS_TX = board.IO17
    
#     # MSP serial to FC
#     MSP_TX = board.IO2
#     MSP_RX = board.IO4
    
#     # Camera pins
#     CAM_D0 = board.IO0
#     CAM_D1 = board.IO12
#     CAM_D2 = board.IO37
#     CAM_D3 = board.IO35
#     CAM_D4 = board.IO14
#     CAM_D5 = board.IO18
#     CAM_D6 = board.IO11
#     CAM_D7 = board.IO33
#     CAM_XCLK = board.IO38
#     CAM_PCLK = board.IO1
#     CAM_VSYNC = board.IO3
#     CAM_HREF = board.IO7
#     CAM_SDA = board.IO8
#     CAM_SCL = board.IO9
#     CAM_PWDN = board.IO43
#     CAM_RESET = board.IO44
    
#     # Status LED
#     LED = board.LED

# # ============================================
# # MSP PROTOCOL CLASS
# # ============================================

# class MSP:
#     """MSP protocol for INAV communication"""
    
#     # Command IDs
#     MSP_SET_WP = 209
#     MSP_WP_MISSION_SAVE = 46
    
#     def __init__(self, uart):
#         self.uart = uart
    
#     def _checksum(self, data):
#         """Calculate XOR checksum"""
#         csum = len(data)
#         for byte in data:
#             csum ^= byte
#         return csum
    
#     def send_command(self, cmd, payload=b''):
#         """Send MSP command"""
#         packet = bytearray([ord('$'), ord('M'), ord('<')])
#         packet.append(len(payload))
#         packet.append(cmd)
#         packet.extend(payload)
#         packet.append(self._checksum(payload) ^ cmd)
        
#         self.uart.write(packet)
    
#     def set_waypoint(self, number, lat, lon, alt, action=2):
#         """
#         Upload waypoint to INAV
#         action: 1=Waypoint, 2=PosHold, 3=RTH
#         """
#         payload = struct.pack(
#             '<BB iii hhh B',
#             number,          # WP number
#             action,          # Action type
#             int(lat * 1e7),  # Latitude
#             int(lon * 1e7),  # Longitude
#             int(alt * 100),  # Altitude (cm)
#             0, 0, 0,        # Parameters
#             0               # Flag
#         )
        
#         self.send_command(self.MSP_SET_WP, payload)
#         time.sleep(0.05)
    
#     def save_mission(self):
#         """Save mission to FC EEPROM"""
#         self.send_command(self.MSP_WP_MISSION_SAVE)
#         time.sleep(0.5)
    
#     def upload_mission(self, drops):
#         """Upload complete mission"""
#         print(f"Uploading {len(drops)} waypoints to INAV...")
        
#         # Waypoint 0: Home/RTH
#         self.set_waypoint(0, 0, 0, 0, action=1)
        
#         # Upload drops
#         for i, drop in enumerate(drops):
#             wp_num = i + 1
#             self.set_waypoint(
#                 wp_num,
#                 drop['lat'],
#                 drop['lon'],
#                 drop.get('alt', 50),
#                 action=2  # Position hold
#             )
#             print(f"  WP{wp_num}: {drop['lat']:.6f}, {drop['lon']:.6f}")
        
#         # Save to FC
#         self.save_mission()
#         print("✓ Mission uploaded!")
#         return True

# # ============================================
# # SBUS PROTOCOL CLASS
# # ============================================

# class SBUS:
#     """SBUS protocol for RC control"""
    
#     def __init__(self, tx_pin):
#         self.uart = busio.UART(
#             tx=tx_pin,
#             rx=None,
#             baudrate=100000,
#             bits=8,
#             parity=busio.UART.Parity.EVEN,
#             stop=2,
#             timeout=0
#         )
#         self.channels = [992] * 16
    
#     def set(self, channel, value):
#         """Set channel (0-15) to value (172-1811)"""
#         if 0 <= channel < 16:
#             self.channels[channel] = max(172, min(1811, value))
    
#     def send(self):
#         """Send SBUS packet"""
#         # Pack 16 channels (11 bits each)
#         bits = 0
#         for i, ch in enumerate(self.channels):
#             bits |= (ch & 0x7FF) << (i * 11)
        
#         # Build packet
#         packet = bytearray(25)
#         packet[0] = 0x0F  # Header
        
#         for i in range(22):
#             packet[i + 1] = (bits >> (i * 8)) & 0xFF
        
#         packet[23] = 0x00  # Flags
#         packet[24] = 0x00  # Footer
        
#         self.uart.write(bytes(packet))

# # ============================================
# # COLLISION DETECTOR CLASS
# # ============================================

# class CollisionDetector:
#     """Fast two-stage collision detection"""
    
#     def __init__(self, camera):
#         self.cam = camera
#         self.prev_gray = None
#         self.edge_threshold = 800
    
#     def detect(self):
#         """Returns (collision, direction, severity)"""
#         frame = self.cam.take(1)
#         if not frame:
#             return False, None, None
        
#         # Stage 1: Fast edge detection
#         edges = self._count_edges(frame)
        
#         if edges < self.edge_threshold:
#             return False, None, None
        
#         # Stage 2: Analyze direction
#         gray = self._to_gray(frame)
#         direction = self._find_direction(gray)
#         severity = "high" if edges > 1500 else "medium"
        
#         self.prev_gray = gray
#         return True, direction, severity
    
#     def _count_edges(self, frame):
#         """Stage 1: Count edges in center"""
#         count = 0
#         w, h = 160, 120
        
#         for y in range(h//4, 3*h//4, 2):
#             for x in range(w//4, 3*w//4, 2):
#                 idx = (y * w + x) * 2
#                 if idx + w*2 < len(frame):
#                     r1 = frame[idx] >> 3
#                     r2 = frame[idx + 2] >> 3
#                     r3 = frame[idx + w*2] >> 3
                    
#                     if abs(r1 - r2) > 3 or abs(r1 - r3) > 3:
#                         count += 1
        
#         return count
    
#     def _to_gray(self, frame):
#         """Convert RGB565 to grayscale"""
#         gray = bytearray(len(frame) // 2)
#         for i in range(0, len(frame), 2):
#             r = (frame[i] >> 3) & 0x1F
#             g = ((frame[i] & 0x07) << 3) | ((frame[i+1] >> 5) & 0x07)
#             b = frame[i+1] & 0x1F
#             gray[i//2] = (r * 19 + g * 38 + b * 7) >> 6
#         return gray
    
#     def _find_direction(self, gray):
#         """Stage 2: Find obstacle direction"""
#         w, h = 160, 120
#         left = center = right = 0
        
#         for y in range(h-1):
#             for x in range(w-1):
#                 idx = y * w + x
#                 if idx + w < len(gray):
#                     edge = abs(gray[idx] - gray[idx+1]) + abs(gray[idx] - gray[idx+w])
                    
#                     if edge > 20:
#                         if x < w//3:
#                             left += edge
#                         elif x > 2*w//3:
#                             right += edge
#                         else:
#                             center += edge
        
#         max_val = max(left, center, right)
#         if max_val == left:
#             return "left"
#         elif max_val == right:
#             return "right"
#         else:
#             return "front"

# # ============================================
# # WEB SERVER CLASS
# # ============================================

# class WebServer:
#     """WiFi web server for mission planning"""
    
#     def __init__(self, msp):
#         self.msp = msp
#         self.mission = {'drops': [], 'status': 'idle'}
    
#     def start(self):
#         """Start WiFi AP and HTTP server"""
#         print("\n" + "="*60)
#         print("Starting WiFi Access Point...")
        
#         wifi.radio.start_ap(WIFI_SSID, WIFI_PASSWORD)
        
#         print(f"✓ SSID: {WIFI_SSID}")
#         print(f"✓ Password: {WIFI_PASSWORD}")
#         print(f"✓ IP: {wifi.radio.ipv4_address_ap}")
#         print(f"✓ URL: http://{wifi.radio.ipv4_address_ap}")
#         print("="*60 + "\n")
        
#         pool = socketpool.SocketPool(wifi.radio)
#         server = pool.socket(pool.AF_INET, pool.SOCK_STREAM)
#         server.bind(('0.0.0.0', 80))
#         server.listen(1)
#         server.settimeout(1.0)
        
#         while True:
#             try:
#                 client, addr = server.accept()
#                 self._handle(client)
#             except OSError:
#                 pass
#             time.sleep(0.01)
    
#     def _handle(self, client):
#         """Handle HTTP request"""
#         try:
#             request = client.recv(4096).decode('utf-8')
#             if not request:
#                 return
            
#             lines = request.split('\r\n')
#             if len(lines) == 0:
#                 return
            
#             parts = lines[0].split(' ')
#             if len(parts) < 2:
#                 return
            
#             method, path = parts[0], parts[1]
            
#             if path == '/':
#                 self._serve_html(client)
#             elif path == '/api/upload':
#                 body = request.split('\r\n\r\n')[1] if '\r\n\r\n' in request else ''
#                 self._api_upload(client, body)
#             elif path == '/api/status':
#                 self._api_status(client)
#             else:
#                 client.send(b'HTTP/1.1 404 Not Found\r\n\r\n')
        
#         except Exception as e:
#             print(f"Error: {e}")
#         finally:
#             client.close()
    
#     def _serve_html(self, client):
#         """Serve web interface"""
#         html = """<!DOCTYPE html>
# <html>
# <head>
# <title>Fishing Drone</title>
# <meta name="viewport" content="width=device-width,initial-scale=1">
# <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
# <style>
# *{margin:0;padding:0;box-sizing:border-box}
# body{font-family:Arial,sans-serif}
# #map{height:70vh}
# #panel{padding:20px;background:#2c3e50;color:white}
# h1{font-size:24px;margin-bottom:15px}
# .stats{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin:15px 0}
# .stat{background:#34495e;padding:15px;border-radius:5px;text-align:center}
# .stat-value{font-size:28px;font-weight:bold}
# .stat-label{font-size:12px;opacity:0.8}
# button{width:100%;padding:15px;margin:5px 0;border:none;border-radius:5px;font-size:16px;font-weight:bold;cursor:pointer}
# .btn-success{background:#27ae60;color:white}
# .btn-danger{background:#e74c3c;color:white}
# #status{position:fixed;top:20px;right:20px;padding:15px 25px;border-radius:5px;display:none;z-index:9999}
# .status-ok{background:#27ae60;color:white}
# .status-err{background:#e74c3c;color:white}
# </style>
# </head>
# <body>
# <div id="panel">
# <h1>🎣 Fishing Drone</h1>
# <p style="margin:10px 0">Click map to add drop locations</p>
# <div class="stats">
# <div class="stat"><div class="stat-value" id="count">0</div><div class="stat-label">Drops</div></div>
# <div class="stat"><div class="stat-value" id="status-text">Ready</div><div class="stat-label">Status</div></div>
# </div>
# <button class="btn-danger" onclick="clear()">Clear All</button>
# <button class="btn-success" onclick="upload()">Upload to Drone</button>
# </div>
# <div id="map"></div>
# <div id="status"></div>
# <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
# <script>
# const map=L.map('map').setView([41.1413,-73.3579],13);
# L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);
# let drops=[],markers=[];
# map.on('click',e=>{
# drops.push({lat:e.latlng.lat,lon:e.latlng.lng,alt:50});
# const m=L.circleMarker([e.latlng.lat,e.latlng.lng],{radius:10,fillColor:'#e74c3c',color:'white',weight:3,fillOpacity:0.9}).addTo(map);
# m.bindPopup(`Drop ${drops.length}<br>${e.latlng.lat.toFixed(6)},${e.latlng.lng.toFixed(6)}`);
# markers.push(m);
# document.getElementById('count').textContent=drops.length;
# });
# function clear(){
# if(!confirm('Clear all drops?'))return;
# drops=[];
# markers.forEach(m=>map.removeLayer(m));
# markers=[];
# document.getElementById('count').textContent=0;
# }
# async function upload(){
# if(drops.length===0){
# show('Add drops first!','err');
# return;
# }
# if(!confirm(`Upload ${drops.length} waypoints?`))return;
# show('Uploading...','ok');
# try{
# const r=await fetch('/api/upload',{
# method:'POST',
# headers:{'Content-Type':'application/json'},
# body:JSON.stringify({drops})
# });
# const d=await r.json();
# if(d.success){
# show(`✓ Uploaded ${drops.length} waypoints!`,'ok');
# document.getElementById('status-text').textContent='Uploaded';
# }else{
# show('Failed: '+d.error,'err');
# }
# }catch(e){
# show('Error: '+e.message,'err');
# }
# }
# function show(msg,type){
# const s=document.getElementById('status');
# s.textContent=msg;
# s.className='status-'+type;
# s.style.display='block';
# setTimeout(()=>s.style.display='none',5000);
# }
# </script>
# </body>
# </html>"""
        
#         resp = f"HTTP/1.1 200 OK\r\nContent-Type:text/html\r\n\r\n{html}"
#         client.send(resp.encode())
    
#     def _api_upload(self, client, body):
#         """Handle mission upload"""
#         try:
#             data = json.loads(body)
#             drops = data.get('drops', [])
            
#             success = self.msp.upload_mission(drops)
            
#             if success:
#                 self.mission['drops'] = drops
#                 self.mission['status'] = 'uploaded'
#                 resp_data = json.dumps({'success': True, 'count': len(drops)})
#             else:
#                 resp_data = json.dumps({'success': False, 'error': 'Upload failed'})
            
#             resp = f"HTTP/1.1 200 OK\r\nContent-Type:application/json\r\n\r\n{resp_data}"
#             client.send(resp.encode())
            
#         except Exception as e:
#             err = json.dumps({'success': False, 'error': str(e)})
#             resp = f"HTTP/1.1 500 Error\r\nContent-Type:application/json\r\n\r\n{err}"
#             client.send(resp.encode())
    
#     def _api_status(self, client):
#         """Return status"""
#         data = json.dumps({
#             'status': self.mission['status'],
#             'drops': len(self.mission['drops'])
#         })
#         resp = f"HTTP/1.1 200 OK\r\nContent-Type:application/json\r\n\r\n{data}"
#         client.send(resp.encode())

# # ============================================
# # GLOBAL STATE
# # ============================================

# collision = False
# collision_dir = "front"

# # ============================================
# # THREAD: SBUS CONTROL (70Hz)
# # ============================================

# def sbus_thread():
#     """SBUS control loop"""
#     global collision, collision_dir
    
#     sbus = SBUS(Pins.SBUS_TX)
    
#     # Initialize channels
#     sbus.set(0, 992)  # Roll
#     sbus.set(1, 992)  # Pitch
#     sbus.set(2, 992)  # Throttle
#     sbus.set(3, 992)  # Yaw
#     sbus.set(4, 1811)  # AUX1 = GPS mode
    
#     print("SBUS control started")
    
#     while True:
#         if collision:
#             # Override for collision avoidance
#             intensity = 350
            
#             if collision_dir == "left":
#                 sbus.set(0, 992 + intensity)  # Roll right
#             elif collision_dir == "right":
#                 sbus.set(0, 992 - intensity)  # Roll left
#             else:
#                 sbus.set(1, 992 - intensity)  # Pitch back
            
#             sbus.set(4, 172)  # Stabilize mode
#         else:
#             # Normal - let INAV control
#             sbus.set(0, 992)
#             sbus.set(1, 992)
#             sbus.set(4, 1811)  # GPS mode
        
#         sbus.send()
#         time.sleep(0.014)

# # ============================================
# # THREAD: VISION (30fps)
# # ============================================

# def vision_thread():
#     """Vision processing loop"""
#     global collision, collision_dir
    
#     print("Initializing camera...")
    
#     try:
#         cam = espcamera.Camera(
#             data_pins=[Pins.CAM_D0, Pins.CAM_D1, Pins.CAM_D2, Pins.CAM_D3,
#                       Pins.CAM_D4, Pins.CAM_D5, Pins.CAM_D6, Pins.CAM_D7],
#             external_clock_pin=Pins.CAM_XCLK,
#             pixel_clock_pin=Pins.CAM_PCLK,
#             vsync_pin=Pins.CAM_VSYNC,
#             href_pin=Pins.CAM_HREF,
#             i2c=busio.I2C(scl=Pins.CAM_SCL, sda=Pins.CAM_SDA),
#             powerdown_pin=Pins.CAM_PWDN,
#             reset_pin=Pins.CAM_RESET,
#             external_clock_frequency=20_000_000,
#             framebuffer_count=2,
#             grab_mode=espcamera.GrabMode.WHEN_EMPTY
#         )
        
#         cam.pixel_format = espcamera.PixelFormat.RGB565
#         cam.frame_size = espcamera.FrameSize.QQVGA
        
#         print("✓ Camera ready")
        
#     except Exception as e:
#         print(f"✗ Camera failed: {e}")
#         return
    
#     detector = CollisionDetector(cam)
    
#     print("Vision processing started")
    
#     while True:
#         hit, direction, severity = detector.detect()
        
#         collision = hit
#         if hit:
#             collision_dir = direction
#             print(f"⚠️ COLLISION: {direction} ({severity})")
        
#         time.sleep(0.033)

# # ============================================
# # MAIN
# # ============================================

# def main():
#     """Main entry point"""
    
#     # Status LED
#     led = digitalio.DigitalInOut(Pins.LED)
#     led.direction = digitalio.Direction.OUTPUT
#     led.value = True
    
#     print("\n" + "="*60)
#     print("FISHING DRONE - AUTONOMOUS SYSTEM")
#     print("="*60 + "\n")
    
#     # Initialize MSP
#     print("Initializing MSP...")
#     try:
#         msp_uart = busio.UART(
#             tx=Pins.MSP_TX,
#             rx=Pins.MSP_RX,
#             baudrate=115200,
#             timeout=0.1
#         )
#         msp = MSP(msp_uart)
#         print("✓ MSP ready\n")
#     except Exception as e:
#         print(f"✗ MSP failed: {e}\n")
#         return
    
#     # Start threads
#     print("Starting threads...")
#     _thread.start_new_thread(sbus_thread, ())
#     time.sleep(0.5)
#     _thread.start_new_thread(vision_thread, ())
#     time.sleep(0.5)
    
#     # Run web server on main thread
#     server = WebServer(msp)
#     server.start()

# # Run
# main()


"""
Fishing Drone - Xiao ESP32S3 Sense
Backup GPS + Drop Trigger + Collision Avoidance
"""

import board
import busio
import digitalio
import pwmio
import time
import json
import math
import espcamera
from adafruit_gps import GPS
from adafruit_motor import servo

print("\n" + "="*50)
print("🎣 FISHING DRONE - BACKUP SYSTEM")
print("="*50 + "\n")

# ============================================
# CONFIGURATION
# ============================================

class Config:
    # Pins
    GPS_RX = board.IO1      # GPS TX → Xiao RX
    GPS_TX = board.IO2      # GPS RX → Xiao TX
    FC_TX = board.IO3       # → FC Serial (override/telemetry)
    SERVO_PIN = board.IO4   # → Servo signal
    
    # Drop detection
    WAYPOINT_RADIUS = 3.0   # meters - trigger radius
    DROP_COOLDOWN = 30.0    # seconds - min time between drops
    
    # Servo timing
    SERVO_RELEASE_ANGLE = 90
    SERVO_RESET_ANGLE = 0
    SERVO_RELEASE_TIME = 1.0  # seconds
    
    # Collision avoidance
    COLLISION_THRESHOLD = 800
    COLLISION_OVERRIDE_TIME = 3.0  # seconds to hold override
    
    # RC Override values (if needed)
    RC_CENTER = 1500
    RC_STOP = 1500

# ============================================
# DROP MANAGER
# ============================================

class DropManager:
    """Manages drop locations and triggers"""
    
    def __init__(self):
        self.drops = []
        self.dropped_indices = set()  # Track completed drops
        self.last_drop_time = 0
        
        self._load_drops()
    
    def _load_drops(self):
        """Load drop locations from JSON"""
        try:
            with open('/drops.json', 'r') as f:
                data = json.load(f)
                self.drops = data['drops']
                
                # Load settings
                settings = data.get('settings', {})
                Config.WAYPOINT_RADIUS = settings.get('waypoint_radius', 3.0)
                Config.SERVO_RELEASE_ANGLE = settings.get('servo_release_angle', 90)
                Config.SERVO_RESET_ANGLE = settings.get('servo_reset_angle', 0)
                Config.COLLISION_THRESHOLD = settings.get('collision_threshold', 800)
                
                print(f"✓ Loaded {len(self.drops)} drop points:")
                for i, drop in enumerate(self.drops):
                    print(f"  {i+1}. {drop['name']}: {drop['lat']:.6f}, {drop['lon']:.6f}")
                print()
                
        except Exception as e:
            print(f"✗ Failed to load drops.json: {e}\n")
            self.drops = []
    
    def check_position(self, current_lat, current_lon):
        """
        Check if at any drop point
        Returns: (should_drop, drop_index, distance) or (False, None, None)
        """
        # Cooldown check
        if time.monotonic() - self.last_drop_time < Config.DROP_COOLDOWN:
            return False, None, None
        
        # Check each drop point
        for i, drop in enumerate(self.drops):
            # Skip if already dropped here
            if i in self.dropped_indices:
                continue
            
            # Calculate distance
            distance = self._haversine(
                current_lat, current_lon,
                drop['lat'], drop['lon']
            )
            
            # Within trigger radius?
            if distance <= Config.WAYPOINT_RADIUS:
                return True, i, distance
        
        return False, None, None
    
    def mark_dropped(self, index):
        """Mark drop as completed"""
        self.dropped_indices.add(index)
        self.last_drop_time = time.monotonic()
        
        remaining = len(self.drops) - len(self.dropped_indices)
        print(f"✓ Drop complete! ({remaining} remaining)")
    
    def get_nearest_drop(self, current_lat, current_lon):
        """Get nearest undropped location and distance"""
        nearest_idx = None
        nearest_dist = float('inf')
        
        for i, drop in enumerate(self.drops):
            if i in self.dropped_indices:
                continue
            
            dist = self._haversine(
                current_lat, current_lon,
                drop['lat'], drop['lon']
            )
            
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_idx = i
        
        if nearest_idx is not None:
            return self.drops[nearest_idx], nearest_dist
        
        return None, None
    
    def all_complete(self):
        """Check if all drops completed"""
        return len(self.dropped_indices) >= len(self.drops)
    
    def _haversine(self, lat1, lon1, lat2, lon2):
        """Calculate distance in meters"""
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        a = (math.sin(dlat/2) ** 2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c

# ============================================
# BAIT RELEASE
# ============================================

class BaitReleaser:
    """Controls servo for bait release"""
    
    def __init__(self, servo_pin):
        print("Initializing servo...")
        pwm = pwmio.PWMOut(servo_pin, frequency=50)
        self.servo = servo.Servo(pwm, min_pulse=500, max_pulse=2500)
        self.reset()
        print("✓ Servo ready\n")
    
    def release(self):
        """Release bait"""
        print("  🎣 RELEASING BAIT...")
        self.servo.angle = Config.SERVO_RELEASE_ANGLE
        time.sleep(Config.SERVO_RELEASE_TIME)
        print("  ✓ Bait released")
    
    def reset(self):
        """Reset servo to closed position"""
        self.servo.angle = Config.SERVO_RESET_ANGLE
        time.sleep(0.5)

# ============================================
# COLLISION DETECTOR
# ============================================

class CollisionDetector:
    """Camera-based obstacle detection"""
    
    def __init__(self):
        print("Initializing camera...")
        try:
            self.cam = espcamera.Camera()
            self.cam.pixel_format = espcamera.PixelFormat.RGB565
            self.cam.frame_size = espcamera.FrameSize.QQVGA
            print("✓ Camera ready\n")
            self.enabled = True
        except Exception as e:
            print(f"⚠️  Camera failed: {e}")
            print("   Collision detection disabled\n")
            self.cam = None
            self.enabled = False
    
    def detect(self):
        """
        Check for obstacles
        Returns: (collision_detected, direction)
        """
        if not self.enabled or not self.cam:
            return False, None
        
        try:
            frame = self.cam.take(1)
            if not frame:
                return False, None
            
            edges = self._count_edges(frame)
            
            if edges > Config.COLLISION_THRESHOLD:
                direction = self._find_direction(frame)
                return True, direction
            
            return False, None
            
        except Exception as e:
            print(f"Camera error: {e}")
            return False, None
    
    def _count_edges(self, frame):
        """Count edges in center region"""
        count = 0
        w, h = 160, 120
        
        # Check center region only
        for y in range(h//4, 3*h//4, 2):
            for x in range(w//4, 3*w//4, 2):
                idx = (y * w + x) * 2
                if idx + w*2 < len(frame):
                    r1 = frame[idx] >> 3
                    r2 = frame[idx + 2] >> 3
                    
                    if abs(r1 - r2) > 3:
                        count += 1
        
        return count
    
    def _find_direction(self, frame):
        """Determine obstacle direction"""
        w = 160
        left = center = right = 0
        
        for y in range(40, 80, 2):
            for x in range(0, w-1, 2):
                idx = (y * w + x) * 2
                r1 = frame[idx] >> 3
                r2 = frame[idx + 2] >> 3
                edge = abs(r1 - r2)
                
                if edge > 3:
                    if x < w//3:
                        left += edge
                    elif x > 2*w//3:
                        right += edge
                    else:
                        center += edge
        
        max_val = max(left, center, right)
        if max_val == left:
            return "left"
        elif max_val == right:
            return "right"
        return "front"

# ============================================
# FC OVERRIDE
# ============================================

class FCController:
    """Send override commands to flight controller"""
    
    def __init__(self, uart):
        self.uart = uart
        self.override_active = False
        self.override_start = 0
    
    def send_stop(self):
        """Send stop/hover command"""
        # Simple protocol - adjust for your FC
        cmd = f"STOP\n"
        self.uart.write(cmd.encode())
        
        if not self.override_active:
            print("  → Sending STOP to FC")
            self.override_active = True
            self.override_start = time.monotonic()
    
    def release_override(self):
        """Release control back to FC"""
        if self.override_active:
            # Check if override time elapsed
            if time.monotonic() - self.override_start > Config.COLLISION_OVERRIDE_TIME:
                cmd = f"RESUME\n"
                self.uart.write(cmd.encode())
                print("  ← Returning control to FC")
                self.override_active = False
    
    def is_overriding(self):
        """Check if currently overriding FC"""
        return self.override_active

# ============================================
# MAIN CONTROLLER
# ============================================

class BackupController:
    """
    Backup system that monitors position and triggers drops
    Primary navigation is handled by FC
    """
    
    def __iniet__(self):
        print("Initializing backup systems...\n")
        
        # GPS
        print("1. GPS:")
        gps_uart = busio.UART(
            tx=Config.GPS_TX,
            rx=Config.GPS_RX,
            baudrate=9600,
            timeout=10
        )
        self.gps = GPS(gps_uart, debug=False)
        self.gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        self.gps.send_command(b'PMTK220,1000')  # 1Hz
        print("  ✓ GPS initialized\n")
        
        # Drop manager
        print("2. Drop Points:")
        self.drop_mgr = DropManager()
        
        # Servo
        print("3. Servo:")
        self.servo = BaitReleaser(Config.SERVO_PIN)
        
        # # Camera
        # print("4. Camera:")
        # self.collision = CollisionDetector()
        
        # FC controller
        # print("5. FC Interface:")
        # fc_uart = busio.UART(
        #     tx=Config.FC_TX,
        #     rx=None,
        #     baudrate=115200,
        #     timeout=0
        # )
        # self.fc = FCController(fc_uart)
        # print("  ✓ FC serial ready\n")
        
        # Status LED
        self.led = digitalio.DigitalInOut(board.LED)
        self.led.direction = digitalio.Direction.OUTPUT
    
    def wait_for_gps(self):
        """Wait for GPS fix"""
        print("Waiting for GPS fix...")
        
        timeout = 60  # 60 seconds
        start = time.monotonic()
        
        while not self.gps.has_fix:
            self.gps.update()
            
            # Blink LED
            self.led.value = not self.led.value
            
            # Timeout check
            if time.monotonic() - start > timeout:
                print("✗ GPS timeout!")
                return False
            
            print(".", end="")
            time.sleep(1)
        
        print("\n✓ GPS fix acquired!")
        print(f"  Position: {self.gps.latitude:.6f}, {self.gps.longitude:.6f}")
        print(f"  Altitude: {self.gps.altitude_m or 0:.1f}m")
        print(f"  Satellites: {self.gps.satellites}\n")
        
        self.led.value = True
        return True
    
    def run(self):
        """Main monitoring loop"""
        print("="*50)
        print("FC handles navigation, Xiao triggers drops")
        print("="*50 + "\n")
        
        # Wait for GPS
        if not self.wait_for_gps():
            print("Cannot run without GPS!")
            return
        
        if len(self.drop_mgr.drops) == 0:
            print("⚠️  No drops loaded - monitoring only")
        
        print("Monitoring started...\n")
        
        last_status = time.monotonic()
        
        while True:
            # Update GPS
            self.gps.update()
            
            # Check GPS fix
            if not self.gps.has_fix:
                print("⚠️  GPS fix lost!")
                time.sleep(1)
                continue
            
            # Get current position
            current_lat = self.gps.latitude
            current_lon = self.gps.longitude
            
            # Check for collision
            # collision, direction = self.collision.detect()
            
            # if collision:
            #     print(f"⚠️  OBSTACLE DETECTED: {direction.upper()}!")
            #     self.fc.send_stop()
            #     time.sleep(0.1)
            #     continue
            # else:
            #     # Release override if active
            #     self.fc.release_override()
            
            # Check if at drop point
            should_drop, drop_idx, distance = self.drop_mgr.check_position(
                current_lat, current_lon
            )
            
            if should_drop:
                drop = self.drop_mgr.drops[drop_idx]
                print(f"\n🎯 AT DROP POINT: {drop['name']}")
                print(f"   Distance: {distance:.1f}m")
                
                # Trigger drop
                self.servo.release()
                time.sleep(1)
                self.servo.reset()
                
                # Mark complete
                self.drop_mgr.mark_dropped(drop_idx)
                
                # Check if all done
                if self.drop_mgr.all_complete():
                    print("\n🎉 ALL DROPS COMPLETE!")
                    print("Monitoring continues...\n")
                
                print()
            
            # Status update (every 5 seconds)
            if time.monotonic() - last_status > 5.0:
                last_status = time.monotonic()
                
                # Get nearest drop
                nearest, dist = self.drop_mgr.get_nearest_drop(
                    current_lat, current_lon
                )
                
                if nearest:
                    print(f"Position: {current_lat:.6f}, {current_lon:.6f} | "
                          f"Next: {nearest['name']} ({dist:.0f}m) | "
                          f"Sats: {self.gps.satellites} | "
                          f"Drops: {len(self.drop_mgr.dropped_indices)}/{len(self.drop_mgr.drops)}")
                else:
                    print(f"Position: {current_lat:.6f}, {current_lon:.6f} | "
                          f"All drops complete | "
                          f"Sats: {self.gps.satellites}")
            
            time.sleep(0.2)

# ============================================
# STARTUP
# ============================================

def main():
    try:
        controller = BackupController()
        controller.run()
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Stopped by user")
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exception(e, e, e.__traceback__)

# Run
main()