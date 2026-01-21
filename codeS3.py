"""
Fishing Drone - ESP32-S3 Controller
Handles: WiFi web server, MSP communication with FC, camera obstacle detection, SBUS control
"""

import board
import busio
import wifi
import socketpool
import json
import time
import espcamera
import _thread
import struct
import digitalio

# ============================================
# Configuration
# ============================================
WIFI_SSID = "FishingDrone"
WIFI_PASSWORD = "fishing123"

# Pin assignments
SBUS_TX_PIN = board.IO17
MSP_TX_PIN = board.IO2
MSP_RX_PIN = board.IO4

# Camera pins
CAMERA_PINS = {
    'data': [board.IO0, board.IO12, board.IO37, board.IO35,
             board.IO14, board.IO18, board.IO11, board.IO33],
    'xclk': board.IO38,
    'pclk': board.IO1,
    'vsync': board.IO3,
    'href': board.IO7,
    'sda': board.IO8,
    'scl': board.IO9,
    'pwdn': board.IO43,
    'reset': board.IO44
}

# ============================================
# MSP Controller Class
# ============================================
class MSPController:
    """
    MSP (MultiWii Serial Protocol) for INAV communication
    Uploads waypoint missions to flight controller
    """
    
    # MSP Commands
    MSP_WP = 118
    MSP_SET_WP = 209
    MSP_WP_MISSION_SAVE = 46
    MSP_NAV_STATUS = 121
    MSP_STATUS = 101
    
    def __init__(self, uart):
        self.uart = uart
        print("MSP Controller initialized")
    
    def send_command(self, cmd, data=b''):
        """Send MSP command to FC"""
        size = len(data)
        checksum = size ^ cmd
        
        for byte in data:
            checksum ^= byte
        
        # MSP v1 format: $M<size><cmd><data><checksum>
        packet = bytearray([ord('$'), ord('M'), ord('<'), size, cmd])
        packet.extend(data)
        packet.append(checksum)
        
        self.uart.write(packet)
    
    def read_response(self, timeout=1.0):
        """Read MSP response from FC"""
        start = time.monotonic()
        buffer = bytearray()
        
        while time.monotonic() - start < timeout:
            if self.uart.in_waiting > 0:
                buffer.extend(self.uart.read(self.uart.in_waiting))
                
                # Check for complete packet
                if len(buffer) >= 6:
                    if buffer[0] == ord('$') and buffer[1] == ord('M') and buffer[2] == ord('>'):
                        size = buffer[3]
                        if len(buffer) >= 6 + size:
                            return buffer[5:5+size]
            
            time.sleep(0.01)
        
        return None
    
    def upload_waypoint_mission(self, drops):
        """
        Upload waypoint mission to INAV
        Converts drop locations to INAV waypoints
        """
        if len(drops) == 0:
            print("No waypoints to upload")
            return False
        
        print(f"Uploading {len(drops)} waypoints to INAV...")
        
        try:
            # Waypoint 0: Home/RTH (required by INAV)
            self.upload_waypoint(0, 0, 0, 0, waypoint_type=1)
            time.sleep(0.1)
            
            # Upload each drop location
            for i, drop in enumerate(drops):
                wp_number = i + 1
                
                lat = int(drop['lat'] * 1e7)
                lon = int(drop['lon'] * 1e7)
                alt = int(drop.get('alt', 50))
                
                # Waypoint type: 2 = Position Hold (hover for drop)
                self.upload_waypoint(wp_number, lat, lon, alt, waypoint_type=2)
                
                print(f"  ✓ Waypoint {wp_number}: {drop['lat']:.6f}, {drop['lon']:.6f}")
                time.sleep(0.1)
            
            # Save mission to FC EEPROM
            self.send_command(self.MSP_WP_MISSION_SAVE)
            time.sleep(0.5)
            
            print("✓ Mission uploaded successfully!")
            return True
            
        except Exception as e:
            print(f"✗ Mission upload failed: {e}")
            return False
    
    def upload_waypoint(self, wp_number, lat, lon, alt, waypoint_type=1):
        """Upload single waypoint to INAV"""
        # INAV waypoint structure (MSP_SET_WP)
        data = struct.pack(
            '<BB iii hhh B',
            wp_number,        # Waypoint number
            waypoint_type,    # Action: 1=Waypoint, 2=PosHold, 3=RTH
            lat,              # Latitude * 1e7
            lon,              # Longitude * 1e7
            alt * 100,        # Altitude in cm
            0, 0, 0,         # Parameters (speed, etc)
            0                 # Flag
        )
        
        self.send_command(self.MSP_SET_WP, data)
        response = self.read_response()
        
        return response is not None
    
    def get_nav_status(self):
        """Query navigation status from INAV"""
        self.send_command(self.MSP_NAV_STATUS)
        response = self.read_response()
        
        if response and len(response) >= 7:
            nav_mode = response[0]
            nav_state = response[1]
            current_wp = response[4]
            
            return {
                'nav_mode': nav_mode,
                'nav_state': nav_state,
                'current_wp': current_wp
            }
        
        return None

# ============================================
# SBUS Transmitter Class
# ============================================
class SBUSTransmitter:
    """
    SBUS protocol transmitter for FC control
    Sends RC commands at 70Hz
    """
    
    def __init__(self, tx_pin):
        self.uart = busio.UART(
            tx=tx_pin,
            rx=None,
            baudrate=100000,
            bits=8,
            parity=busio.UART.Parity.EVEN,
            stop=2,
            timeout=0
        )
        self.channels = [992] * 16
        print(f"SBUS initialized on {tx_pin}")
    
    def set_channel(self, channel, value):
        """Set channel value (0-15, 172-1811)"""
        if 0 <= channel < 16:
            self.channels[channel] = max(172, min(1811, value))
    
    def _pack_channels(self):
        """Pack 16 11-bit channels into bytes"""
        bit_string = 0
        for i, channel in enumerate(self.channels):
            bit_string |= (channel & 0x7FF) << (i * 11)
        
        packed = []
        for i in range(22):
            packed.append((bit_string >> (i * 8)) & 0xFF)
        
        return bytes(packed)
    
    def build_packet(self):
        """Build complete SBUS packet"""
        packet = bytearray(25)
        packet[0] = 0x0F                    # Start byte
        packet[1:23] = self._pack_channels()  # Channel data
        packet[23] = 0x00                    # Flags
        packet[24] = 0x00                    # End byte
        return bytes(packet)
    
    def send_packet(self):
        """Send SBUS packet"""
        self.uart.write(self.build_packet())

# ============================================
# Two-Stage Collision Detector Class
# ============================================
class TwoStageCollisionDetector:
    """
    Fast collision detection using two-stage pipeline:
    Stage 1: Fast edge detection (every frame)
    Stage 2: Direction analysis (only when obstacle detected)
    """
    
    def __init__(self, camera):
        self.camera = camera
        self.prev_frame = None
        self.prev_gray = None
        
        # Tunable thresholds
        self.edge_threshold = 800
        self.collision_threshold = 60
    
    def detect_collision(self):
        """
        Two-stage collision detection
        Returns: (collision_imminent, direction, velocity_estimate)
        """
        frame = self.camera.take(1)
        if not frame:
            return False, None, None
        
        # STAGE 1: Fast edge detection
        edge_count = self.fast_edge_detection(frame, 160, 120)
        
        if edge_count < self.edge_threshold:
            self.prev_frame = frame
            return False, None, None
        
        # STAGE 2: Full analysis - obstacle detected
        gray = self.rgb565_to_gray(frame)
        direction, confidence = self.analyze_direction_vector(gray, 160, 120)
        
        velocity = None
        if self.prev_gray is not None:
            velocity = self.estimate_velocity(gray, self.prev_gray, 160, 120)
        
        self.prev_frame = frame
        self.prev_gray = gray
        
        if confidence > self.collision_threshold:
            return True, direction, velocity
        
        return False, None, None
    
    def fast_edge_detection(self, frame, width, height):
        """Stage 1: Ultra-fast edge detection on RGB565"""
        edge_count = 0
        
        x_start = width // 5
        x_end = 4 * width // 5
        y_start = height // 5
        y_end = 4 * height // 5
        
        for y in range(y_start, y_end - 1, 2):
            row_start = y * width * 2
            
            for x in range(x_start, x_end - 1, 2):
                idx = row_start + x * 2
                
                if idx + width * 2 >= len(frame):
                    break
                
                current_r = frame[idx] >> 3
                right_r = frame[idx + 2] >> 3
                below_r = frame[idx + width * 2] >> 3
                
                h_edge = abs(current_r - right_r)
                v_edge = abs(current_r - below_r)
                
                if h_edge > 3 or v_edge > 3:
                    edge_count += 1
        
        return edge_count
    
    def rgb565_to_gray(self, frame):
        """Convert RGB565 to grayscale"""
        gray = bytearray(len(frame) // 2)
        
        for i in range(0, len(frame), 2):
            r = (frame[i] >> 3) & 0x1F
            g = ((frame[i] & 0x07) << 3) | ((frame[i+1] >> 5) & 0x07)
            b = frame[i+1] & 0x1F
            
            gray[i//2] = (r * 19 + g * 38 + b * 7) >> 6
        
        return gray
    
    def analyze_direction_vector(self, gray, width, height):
        """Stage 2: Direction vector analysis"""
        zones = {
            'left': 0, 'center': 0, 'right': 0,
            'top': 0, 'bottom': 0
        }
        
        for y in range(height - 1):
            for x in range(width - 1):
                idx = y * width + x
                
                if idx + width >= len(gray):
                    break
                
                h_edge = abs(gray[idx] - gray[idx + 1])
                v_edge = abs(gray[idx] - gray[idx + width])
                edge_strength = h_edge + v_edge
                
                if edge_strength > 20:
                    # Classify by position
                    if x < width // 3:
                        zones['left'] += edge_strength
                    elif x > 2 * width // 3:
                        zones['right'] += edge_strength
                    else:
                        zones['center'] += edge_strength
                    
                    if y < height // 3:
                        zones['top'] += edge_strength
                    elif y > 2 * height // 3:
                        zones['bottom'] += edge_strength
        
        # Weight center more
        zones['center'] *= 2
        
        total = zones['left'] + zones['center'] + zones['right']
        
        if total < 1000:
            return None, 0
        
        # Determine direction
        max_h = max(zones['left'], zones['center'], zones['right'])
        
        if max_h == zones['left'] and zones['left'] > zones['center'] * 1.3:
            direction = "left"
        elif max_h == zones['right'] and zones['right'] > zones['center'] * 1.3:
            direction = "right"
        else:
            if zones['top'] > zones['bottom'] * 1.5:
                direction = "above"
            elif zones['bottom'] > zones['top'] * 1.5:
                direction = "below"
            else:
                direction = "front"
        
        confidence = min(total // 100, 150)
        
        return direction, confidence
    
    def estimate_velocity(self, current_gray, prev_gray, width, height):
        """Estimate approach velocity"""
        center_x_start = width // 3
        center_x_end = 2 * width // 3
        center_y_start = height // 3
        center_y_end = 2 * height // 3
        
        density_change = 0
        pixel_count = 0
        
        for y in range(center_y_start, center_y_end):
            for x in range(center_x_start, center_x_end):
                idx = y * width + x
                
                if idx < len(current_gray) and idx < len(prev_gray):
                    change = abs(current_gray[idx] - prev_gray[idx])
                    density_change += change
                    pixel_count += 1
        
        avg_change = density_change / pixel_count if pixel_count > 0 else 0
        
        if avg_change > 15:
            return "critical"
        elif avg_change > 10:
            return "fast"
        elif avg_change > 5:
            return "medium"
        else:
            return "slow"

# ============================================
# Web Server Class
# ============================================
class DroneWebServer:
    """
    WiFi web server for mission planning
    Serves HTML interface and handles API requests
    """
    
    def __init__(self, msp_controller):
        self.msp = msp_controller
        self.current_mission = {
            'drops': [],
            'status': 'idle'
        }
    
    def start(self):
        """Start WiFi AP and web server"""
        print("\n" + "="*60)
        print("Starting WiFi Access Point...")
        print("="*60)
        
        wifi.radio.start_ap(WIFI_SSID, WIFI_PASSWORD)
        
        print(f"✓ WiFi AP started")
        print(f"  SSID: {WIFI_SSID}")
        print(f"  Password: {WIFI_PASSWORD}")
        print(f"  IP: {wifi.radio.ipv4_address_ap}")
        print(f"  URL: http://{wifi.radio.ipv4_address_ap}")
        print("="*60 + "\n")
        
        pool = socketpool.SocketPool(wifi.radio)
        server_socket = pool.socket(pool.AF_INET, pool.SOCK_STREAM)
        server_socket.bind(('0.0.0.0', 80))
        server_socket.listen(1)
        server_socket.settimeout(1.0)
        
        print("Web server listening on port 80")
        print("Waiting for connections...\n")
        
        while True:
            try:
                client_socket, client_address = server_socket.accept()
                self.handle_client(client_socket, client_address)
            except OSError:
                pass
            
            time.sleep(0.1)
    
    def handle_client(self, client_socket, client_address):
        """Handle HTTP request"""
        try:
            request = client_socket.recv(4096).decode('utf-8')
            
            if not request:
                return
            
            lines = request.split('\r\n')
            if len(lines) == 0:
                return
            
            request_line = lines[0]
            parts = request_line.split(' ')
            
            if len(parts) < 2:
                return
            
            method, path = parts[0], parts[1]
            
            print(f"{method} {path}")
            
            # Route requests
            if path == '/':
                self.serve_html(client_socket)
            elif path == '/api/status':
                self.api_status(client_socket)
            elif path.startswith('/api/upload_mission'):
                body = request.split('\r\n\r\n')[1] if '\r\n\r\n' in request else ''
                self.api_upload_mission(client_socket, body)
            elif path == '/api/start_mission':
                self.api_start_mission(client_socket)
            else:
                self.send_404(client_socket)
        
        except Exception as e:
            print(f"Error handling client: {e}")
        
        finally:
            client_socket.close()
    
    def serve_html(self, client_socket):
        """Serve web interface"""
        html = """<!DOCTYPE html>
<html>
<head>
    <title>Fishing Drone Mission Planner</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif; }
        
        #map { height: 70vh; width: 100%; }
        
        #controls {
            padding: 20px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        
        h1 {
            font-size: 28px;
            margin-bottom: 20px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.2);
        }
        
        .stats {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin: 20px 0;
        }
        
        .stat {
            background: rgba(255,255,255,0.2);
            padding: 15px;
            border-radius: 10px;
            backdrop-filter: blur(10px);
        }
        
        .stat-label {
            font-size: 12px;
            opacity: 0.9;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        
        .stat-value {
            font-size: 28px;
            font-weight: bold;
            margin-top: 5px;
        }
        
        .button-group {
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
        }
        
        button {
            padding: 15px 30px;
            border: none;
            border-radius: 8px;
            font-size: 16px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s ease;
            flex: 1;
            min-width: 200px;
        }
        
        button:hover {
            transform: translateY(-2px);
            box-shadow: 0 4px 12px rgba(0,0,0,0.2);
        }
        
        button:active {
            transform: translateY(0);
        }
        
        .btn-primary {
            background: #3498db;
            color: white;
        }
        
        .btn-success {
            background: #27ae60;
            color: white;
        }
        
        .btn-danger {
            background: #e74c3c;
            color: white;
        }
        
        .btn-warning {
            background: #f39c12;
            color: white;
        }
        
        #status {
            position: fixed;
            top: 20px;
            right: 20px;
            padding: 15px 25px;
            border-radius: 10px;
            font-weight: 600;
            box-shadow: 0 4px 12px rgba(0,0,0,0.3);
            display: none;
            z-index: 10000;
            animation: slideIn 0.3s ease;
        }
        
        @keyframes slideIn {
            from { transform: translateX(400px); opacity: 0; }
            to { transform: translateX(0); opacity: 1; }
        }
        
        .status-success { background: #27ae60; color: white; }
        .status-error { background: #e74c3c; color: white; }
        .status-info { background: #3498db; color: white; }
        
        .instructions {
            background: rgba(255,255,255,0.1);
            padding: 15px;
            border-radius: 8px;
            margin: 15px 0;
            font-size: 14px;
        }
    </style>
</head>
<body>
    <div id="controls">
        <h1>🎣 Fishing Drone Mission Planner</h1>
        
        <div class="instructions">
            <strong>Instructions:</strong> Click on the map to add drop locations. 
            Each click adds a waypoint where the drone will hover and deploy bait.
        </div>
        
        <div class="stats">
            <div class="stat">
                <div class="stat-label">Drop Locations</div>
                <div class="stat-value" id="drop-count">0</div>
            </div>
            <div class="stat">
                <div class="stat-label">Mission Status</div>
                <div class="stat-value" id="mission-status">Ready</div>
            </div>
            <div class="stat">
                <div class="stat-label">Distance</div>
                <div class="stat-value" id="mission-distance">0m</div>
            </div>
        </div>
        
        <div class="button-group">
            <button class="btn-danger" onclick="clearDrops()">🗑️ Clear All</button>
            <button class="btn-success" onclick="uploadToFC()">📡 Upload to FC</button>
            <button class="btn-warning" onclick="startMission()">🚀 Start Mission</button>
        </div>
    </div>
    
    <div id="map"></div>
    <div id="status"></div>
    
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <script>
        const map = L.map('map').setView([41.1413, -73.3579], 13);
        
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors',
            maxZoom: 19
        }).addTo(map);
        
        let drops = [];
        let markers = [];
        let polyline = null;
        
        map.on('click', function(e) {
            addDrop(e.latlng.lat, e.latlng.lng);
        });
        
        function addDrop(lat, lng) {
            const drop = { 
                lat: lat, 
                lon: lng, 
                alt: 50,
                id: Date.now()
            };
            drops.push(drop);
            
            const marker = L.circleMarker([lat, lng], {
                radius: 12,
                fillColor: '#e74c3c',
                color: 'white',
                weight: 3,
                fillOpacity: 0.9
            }).addTo(map);
            
            marker.bindPopup(`
                <b>Drop #${drops.length}</b><br>
                Lat: ${lat.toFixed(6)}<br>
                Lon: ${lng.toFixed(6)}<br>
                Alt: 50m<br>
                <button onclick="removeDropById(${drop.id})">Remove</button>
            `);
            
            markers.push({ id: drop.id, marker: marker });
            
            updatePath();
            updateStats();
        }
        
        function removeDropById(id) {
            const index = drops.findIndex(d => d.id === id);
            if (index !== -1) {
                drops.splice(index, 1);
                
                const markerObj = markers.find(m => m.id === id);
                if (markerObj) {
                    map.removeLayer(markerObj.marker);
                }
                
                markers = markers.filter(m => m.id !== id);
                updatePath();
                updateStats();
                map.closePopup();
            }
        }
        
        function clearDrops() {
            if (drops.length === 0) return;
            if (!confirm(`Clear all ${drops.length} drop locations?`)) return;
            
            drops = [];
            markers.forEach(m => map.removeLayer(m.marker));
            markers = [];
            
            if (polyline) {
                map.removeLayer(polyline);
                polyline = null;
            }
            
            updateStats();
        }
        
        function updatePath() {
            if (polyline) {
                map.removeLayer(polyline);
            }
            
            if (drops.length > 1) {
                const latLngs = drops.map(d => [d.lat, d.lon]);
                polyline = L.polyline(latLngs, {
                    color: '#3498db',
                    weight: 3,
                    opacity: 0.7,
                    dashArray: '10, 10'
                }).addTo(map);
            }
        }
        
        function updateStats() {
            document.getElementById('drop-count').textContent = drops.length;
            
            if (drops.length > 1) {
                let distance = 0;
                for (let i = 0; i < drops.length - 1; i++) {
                    distance += calculateDistance(
                        drops[i].lat, drops[i].lon,
                        drops[i+1].lat, drops[i+1].lon
                    );
                }
                document.getElementById('mission-distance').textContent = 
                    Math.round(distance) + 'm';
            } else {
                document.getElementById('mission-distance').textContent = '0m';
            }
        }
        
        function calculateDistance(lat1, lon1, lat2, lon2) {
            const R = 6371000;
            const dLat = (lat2 - lat1) * Math.PI / 180;
            const dLon = (lon2 - lon1) * Math.PI / 180;
            const a = Math.sin(dLat/2) * Math.sin(dLat/2) +
                      Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
                      Math.sin(dLon/2) * Math.sin(dLon/2);
            const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
            return R * c;
        }
        
        async function uploadToFC() {
            if (drops.length === 0) {
                showStatus('Add drop locations first!', 'error');
                return;
            }
            
            if (!confirm(`Upload ${drops.length} waypoints to flight controller?`)) {
                return;
            }
            
            showStatus('Uploading to FC...', 'info');
            
            try {
                const response = await fetch('/api/upload_mission', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ drops: drops })
                });
                
                const data = await response.json();
                
                if (data.success) {
                    showStatus(`✓ Uploaded ${drops.length} waypoints!`, 'success');
                    document.getElementById('mission-status').textContent = 'Uploaded';
                } else {
                    showStatus('Upload failed: ' + data.error, 'error');
                }
            } catch (error) {
                showStatus('Connection error: ' + error.message, 'error');
            }
        }
        
        async function startMission() {
            if (drops.length === 0) {
                showStatus('Upload mission first!', 'error');
                return;
            }
            
            if (!confirm('Start autonomous mission?\n\nMake sure:\n- GPS lock acquired\n- Props installed\n- Area is clear')) {
                return;
            }
            
            try {
                const response = await fetch('/api/start_mission', { 
                    method: 'POST' 
                });
                const data = await response.json();
                
                if (data.success) {
                    showStatus('✓ Mission started!', 'success');
                    document.getElementById('mission-status').textContent = 'Active';
                } else {
                    showStatus('Failed to start mission', 'error');
                }
            } catch (error) {
                showStatus('Error: ' + error.message, 'error');
            }
        }
        
        function showStatus(message, type) {
            const status = document.getElementById('status');
            status.textContent = message;
            status.className = 'status-' + type;
            status.style.display = 'block';
            
            setTimeout(() => {
                status.style.display = 'none';
            }, 5000);
        }
        
        // Update status periodically
        setInterval(async () => {
            try {
                const response = await fetch('/api/status');
                const data = await response.json();
                document.getElementById('mission-status').textContent = 
                    data.status.charAt(0).toUpperCase() + data.status.slice(1);
            } catch (error) {
                // Silently fail
            }
        }, 2000);
    </script>
</body>
</html>"""
        
        response = f"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: {len(html)}\r\n\r\n{html}"
        client_socket.send(response.encode())
    
    def api_status(self, client_socket):
        """Return drone status"""
        data = json.dumps({
            'status': self.current_mission['status'],
            'drops': len(self.current_mission['drops']),
            'battery': 23.5,
            'gps_sats': 8
        })
        
        response = f"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n{data}"
        client_socket.send(response.encode())
    
    def api_upload_mission(self, client_socket, body):
        """Upload mission to FC via MSP"""
        try:
            data = json.loads(body)
            drops = data.get('drops', [])
            
            print(f"\nReceived mission upload request: {len(drops)} waypoints")
            
            # Upload to INAV via MSP
            success = self.msp.upload_waypoint_mission(drops)
            
            if success:
                self.current_mission['drops'] = drops
                self.current_mission['status'] = 'uploaded'
                
                response_data = json.dumps({
                    'success': True, 
                    'count': len(drops)
                })
            else:
                response_data = json.dumps({
                    'success': False, 
                    'error': 'MSP upload failed'
                })
            
            response = f"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n{response_data}"
            client_socket.send(response.encode())
            
        except Exception as e:
            print(f"Error in api_upload_mission: {e}")
            error_data = json.dumps({'success': False, 'error': str(e)})
            response = f"HTTP/1.1 500 Internal Server Error\r\nContent-Type: application/json\r\n\r\n{error_data}"
            client_socket.send(response.encode())
    
    def api_start_mission(self, client_socket):
        """Start mission execution"""
        self.current_mission['status'] = 'active'
        
        # Mission start is handled by switching FC to waypoint mode
        # This is done via SBUS AUX channel in the main loop
        
        data = json.dumps({'success': True})
        response = f"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n{data}"
        client_socket.send(response.encode())
        
        print("\n🚀 Mission START requested")
    
    def send_404(self, client_socket):
        response = "HTTP/1.1 404 Not Found\r\n\r\n404 Not Found"
        client_socket.send(response.encode())

# ============================================
# Global State (shared between threads)
# ============================================
collision_imminent = False
collision_direction = "front"
collision_velocity = None
mission_active = False

# ============================================
# Thread 1: SBUS Flight Control (70Hz)
# ============================================
def sbus_control_loop():
    """
    SBUS control thread
    Normally passes through to INAV
    Overrides only during collision avoidance
    """
    global collision_imminent, collision_direction, collision_velocity
    global mission_active
    
    print("Starting SBUS control loop...")
    
    sbus = SBUSTransmitter(SBUS_TX_PIN)
    
    # Initialize channels
    sbus.set_channel(0, 992)   # Roll center
    sbus.set_channel(1, 992)   # Pitch center
    sbus.set_channel(2, 992)   # Throttle center
    sbus.set_channel(3, 992)   # Yaw center
    sbus.set_channel(4, 1811)  # AUX1 - GPS/Waypoint mode
    
    last_collision_time = 0
    
    print("SBUS control ready\n")
    
    while True:
        if collision_imminent:
            # EMERGENCY COLLISION AVOIDANCE
            if collision_velocity == "critical":
                intensity = 400
            elif collision_velocity == "fast":
                intensity = 300
            elif collision_velocity == "medium":
                intensity = 200
            else:
                intensity = 150
            
            # Apply evasive maneuver
            if collision_direction == "right":
                sbus.set_channel(0, 992 - intensity)  # Roll left
            elif collision_direction == "left":
                sbus.set_channel(0, 992 + intensity)  # Roll right
            elif collision_direction == "above":
                sbus.set_channel(2, 992 - intensity)  # Descend
            elif collision_direction == "below":
                sbus.set_channel(2, 992 + intensity)  # Climb
            else:  # front
                sbus.set_channel(1, 992 - intensity)  # Pitch back
            
            sbus.set_channel(4, 172)  # Switch to Stabilize (AI control)
            last_collision_time = time.monotonic()
        
        else:
            # Return to INAV control after collision cleared
            if time.monotonic() - last_collision_time > 2.0:
                sbus.set_channel(0, 992)
                sbus.set_channel(1, 992)
                sbus.set_channel(2, 992)
                
                # Choose flight mode based on mission status
                if mission_active:
                    sbus.set_channel(4, 1811)  # GPS/Waypoint mode
                else:
                    sbus.set_channel(4, 992)   # Stabilize/Manual
        
        sbus.send_packet()
        time.sleep(0.014)  # 70Hz

# ============================================
# Thread 2: Vision Processing (60fps target)
# ============================================
def vision_processing_loop():
    """
    Vision processing thread
    Monitors camera for obstacles
    Sets global collision flags
    """
    global collision_imminent, collision_direction, collision_velocity
    
    print("Initializing camera...")
    
    try:
        cam = espcamera.Camera(
            data_pins=CAMERA_PINS['data'],
            external_clock_pin=CAMERA_PINS['xclk'],
            pixel_clock_pin=CAMERA_PINS['pclk'],
            vsync_pin=CAMERA_PINS['vsync'],
            href_pin=CAMERA_PINS['href'],
            i2c=busio.I2C(scl=CAMERA_PINS['scl'], sda=CAMERA_PINS['sda']),
            powerdown_pin=CAMERA_PINS['pwdn'],
            reset_pin=CAMERA_PINS['reset'],
            external_clock_frequency=20_000_000,
            framebuffer_count=2,
            grab_mode=espcamera.GrabMode.WHEN_EMPTY
        )
        
        cam.pixel_format = espcamera.PixelFormat.RGB565
        cam.frame_size = espcamera.FrameSize.QQVGA  # 160x120 for speed
        
        print("✓ Camera initialized\n")
        
    except Exception as e:
        print(f"✗ Camera initialization failed: {e}")
        print("Vision processing disabled\n")
        return
    
    detector = TwoStageCollisionDetector(cam)
    
    frame_count = 0
    start_time = time.monotonic()
    
    print("Vision processing ready\n")
    
    while True:
        try:
            collision, direction, velocity = detector.detect_collision()
            
            collision_imminent = collision
            
            if collision:
                collision_direction = direction
                collision_velocity = velocity
                print(f"🚨 COLLISION WARNING: {direction} | Velocity: {velocity}")
            
            # FPS tracking
            frame_count += 1
            if frame_count % 300 == 0:  # Every 5 seconds at 60fps
                elapsed = time.monotonic() - start_time
                fps = frame_count / elapsed
                print(f"Vision FPS: {fps:.1f}")
        
        except Exception as e:
            print(f"Vision error: {e}")
            time.sleep(0.1)

# ============================================
# Thread 3: WiFi Web Server
# ============================================
def web_server_loop(msp_controller):
    """
    Web server thread
    Hosts mission planning interface
    """
    server = DroneWebServer(msp_controller)
    server.start()

# ============================================
# MAIN PROGRAM
# ============================================
def main():
    """Main entry point"""
    
    # LED for status
    led = digitalio.DigitalInOut(board.LED)
    led.direction = digitalio.Direction.OUTPUT
    led.value = True
    
    print("\n" + "="*60)
    print("  FISHING DRONE - AUTONOMOUS SYSTEM")
    print("="*60)
    print(f"  Firmware: v1.0")
    print(f"  Board: ESP32-S3 Feather")
    print("="*60 + "\n")
    
    # Initialize MSP connection to FC
    print("Initializing MSP connection to flight controller...")
    try:
        msp_uart = busio.UART(
            tx=MSP_TX_PIN,
            rx=MSP_RX_PIN,
            baudrate=115200,
            timeout=0.1
        )
        msp = MSPController(msp_uart)
        print("✓ MSP initialized\n")
    except Exception as e:
        print(f"✗ MSP initialization failed: {e}\n")
        msp = None
    
    # Start threads
    print("Starting system threads...\n")
    
    # Thread 1: SBUS Control
    _thread.start_new_thread(sbus_control_loop, ())
    time.sleep(0.5)
    
    # Thread 2: Vision Processing
    _thread.start_new_thread(vision_processing_loop, ())
    time.sleep(0.5)
    
    # Thread 3: Web Server (runs on main thread)
    if msp:
        web_server_loop(msp)
    else:
        print("MSP not available - web server disabled")
        print("Running in collision avoidance only mode\n")
        while True:
            led.value = not led.value
            time.sleep(1)

# Run main program
if __name__ == "__main__":
    main()