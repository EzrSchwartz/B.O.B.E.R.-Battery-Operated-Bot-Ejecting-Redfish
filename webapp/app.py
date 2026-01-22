"""
Fishing Drone Desktop Mission Planner
With XML, QGC, and INAV format support
"""

from flask import Flask, render_template, request, jsonify
from flask_cors import CORS
import json
import os
import re
import xml.etree.ElementTree as ET
from datetime import datetime

app = Flask(__name__)
CORS(app)

MISSIONS_DIR = 'missions'
os.makedirs(MISSIONS_DIR, exist_ok=True)

class MissionParser:
    """Enhanced mission file parser supporting multiple formats"""
    
    @staticmethod
    def parse(filepath):
        """Try multiple parsing strategies"""
        
        with open(filepath, 'r') as f:
            content = f.read()
        
        print("\n" + "="*70)
        print(f"PARSING FILE: {filepath}")
        print("="*70)
        
        # Show raw content
        lines = content.split('\n')
        print(f"Total lines: {len(lines)}")
        print(f"\nFirst 5 lines:")
        for i, line in enumerate(lines[:5], 1):
            print(f"{i:2d}: {line[:80]}")
        
        # Detect format
        if content.strip().startswith('<?xml'):
            print("\nDetected: XML format")
            waypoints = MissionParser._parse_xml(content)
        else:
            print("\nDetected: Text format (QGC/INAV)")
            # Try different text-based strategies
            strategies = [
                MissionParser._parse_qgc,
                MissionParser._parse_inav,
                MissionParser._parse_numeric
            ]
            
            waypoints = []
            for strategy in strategies:
                waypoints = strategy(content)
                if waypoints:
                    print(f"✓ SUCCESS with {strategy.__name__}")
                    break
        
        if waypoints:
            print(f"Found {len(waypoints)} waypoints")
            for wp in waypoints[:3]:
                print(f"  WP{wp['index']}: {wp['lat']:.6f}, {wp['lon']:.6f}, {wp['alt']}m")
            if len(waypoints) > 3:
                print(f"  ... and {len(waypoints)-3} more")
        else:
            print("✗ No waypoints found")
        
        print("="*70 + "\n")
        return waypoints
    
    @staticmethod
    def _parse_xml(content):
        """Parse XML mission format (INAV Configurator / mwp)"""
        waypoints = []
        
        try:
            root = ET.fromstring(content)
            
            # Find all missionitem elements
            for item in root.findall('.//missionitem'):
                try:
                    no = int(item.get('no', 0))
                    lat = float(item.get('lat', 0))
                    lon = float(item.get('lon', 0))
                    alt = float(item.get('alt', 0))
                    action = item.get('action', 'WAYPOINT')
                    
                    # Validate coordinates
                    if -90 <= lat <= 90 and -180 <= lon <= 180:
                        waypoints.append({
                            'index': no,
                            'lat': lat,
                            'lon': lon,
                            'alt': alt,
                            'action': action
                        })
                
                except (ValueError, AttributeError) as e:
                    print(f"  Warning: Could not parse missionitem - {e}")
                    continue
            
            # Sort by index
            waypoints.sort(key=lambda x: x['index'])
            
        except ET.ParseError as e:
            print(f"  XML Parse Error: {e}")
            return []
        
        return waypoints
    
    @staticmethod
    def _parse_qgc(content):
        """Parse QGroundControl text format"""
        waypoints = []
        
        for line in content.split('\n'):
            line = line.strip()
            
            if not line or line.startswith('#') or 'QGC' in line:
                continue
            
            # Try tab-separated
            parts = line.split('\t')
            
            # Fallback to space-separated
            if len(parts) < 12:
                parts = line.split()
            
            if len(parts) >= 12:
                try:
                    idx = int(float(parts[0]))
                    lat = float(parts[8])
                    lon = float(parts[9])
                    alt = float(parts[10])
                    
                    if -90 <= lat <= 90 and -180 <= lon <= 180:
                        if idx > 0:
                            waypoints.append({
                                'index': idx,
                                'lat': lat,
                                'lon': lon,
                                'alt': alt
                            })
                except (ValueError, IndexError):
                    continue
        
        return waypoints
    
    @staticmethod
    def _parse_inav(content):
        """Parse INAV native format"""
        waypoints = []
        
        for line in content.split('\n'):
            line = line.strip()
            
            if not line or line.startswith('#'):
                continue
            
            parts = line.split()
            
            if len(parts) >= 10:
                try:
                    for i in range(len(parts) - 2):
                        lat = float(parts[i])
                        lon = float(parts[i+1])
                        
                        if -90 <= lat <= 90 and -180 <= lon <= 180:
                            alt = 50
                            if i+2 < len(parts):
                                try:
                                    alt = float(parts[i+2])
                                except:
                                    pass
                            
                            waypoints.append({
                                'index': len(waypoints) + 1,
                                'lat': lat,
                                'lon': lon,
                                'alt': alt
                            })
                            break
                except (ValueError, IndexError):
                    continue
        
        return waypoints
    
    @staticmethod
    def _parse_numeric(content):
        """Parse by finding coordinate patterns"""
        waypoints = []
        
        for line in content.split('\n'):
            line = line.strip()
            
            if not line or line.startswith('#') or 'QGC' in line:
                continue
            
            numbers = []
            for part in re.split(r'[,\s\t]+', line):
                try:
                    numbers.append(float(part))
                except:
                    pass
            
            if len(numbers) >= 3:
                for i in range(len(numbers) - 2):
                    lat = numbers[i]
                    lon = numbers[i+1]
                    alt = numbers[i+2]
                    
                    if (-90 <= lat <= 90 and -180 <= lon <= 180 and 
                        0 <= alt <= 1000):
                        
                        if not any(w['lat'] == lat and w['lon'] == lon 
                                 for w in waypoints):
                            waypoints.append({
                                'index': len(waypoints) + 1,
                                'lat': lat,
                                'lon': lon,
                                'alt': alt
                            })
                            break
        
        return waypoints

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/upload', methods=['POST'])
def upload():
    """Upload mission file"""
    if 'file' not in request.files:
        return jsonify({'error': 'No file'}), 400
    
    file = request.files['file']
    
    if not file.filename:
        return jsonify({'error': 'No file selected'}), 400
    
    # Save file
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    
    # Preserve original extension
    original_ext = os.path.splitext(file.filename)[1]
    filename = f"mission_{timestamp}{original_ext}"
    filepath = os.path.join(MISSIONS_DIR, filename)
    
    file.save(filepath)
    
    # Parse
    waypoints = MissionParser.parse(filepath)
    
    return jsonify({
        'success': True,
        'waypoints': waypoints,
        'count': len(waypoints),
        'filename': filename
    })

@app.route('/api/inspect', methods=['POST'])
def inspect():
    """Inspect file for debugging"""
    if 'file' not in request.files:
        return jsonify({'error': 'No file'}), 400
    
    file = request.files['file']
    content = file.read().decode('utf-8', errors='ignore')
    lines = content.split('\n')
    
    # Detect format
    is_xml = content.strip().startswith('<?xml')
    
    # Analyze first data line
    sample = None
    if not is_xml:
        for line in lines:
            line = line.strip()
            if line and not line.startswith('#') and 'QGC' not in line:
                sample = {
                    'original': line,
                    'length': len(line),
                    'tab_count': line.count('\t'),
                    'space_count': line.count(' '),
                    'comma_count': line.count(','),
                    'parts_tab': line.split('\t'),
                    'parts_space': line.split(),
                    'parts_comma': line.split(',')
                }
                break
    
    return jsonify({
        'total_lines': len(lines),
        'first_10': lines[:10],
        'format': 'XML' if is_xml else 'Text',
        'has_tabs': '\t' in content,
        'has_commas': ',' in content,
        'sample_line': sample
    })

@app.route('/api/save', methods=['POST'])
def save():
    """Save mission"""
    data = request.json
    filename = f"mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    filepath = os.path.join(MISSIONS_DIR, filename)
    
    with open(filepath, 'w') as f:
        json.dump(data, f, indent=2)
    
    return jsonify({'success': True, 'filename': filename})

@app.route('/api/test_parse', methods=['POST'])
def test_parse():
    """Test parsing with sample data"""
    data = request.json
    text = data.get('text', '')
    
    # Save to temp file
    temp_path = os.path.join(MISSIONS_DIR, 'temp_test.mission')
    with open(temp_path, 'w') as f:
        f.write(text)
    
    # Parse
    waypoints = MissionParser.parse(temp_path)
    
    return jsonify({
        'waypoints': waypoints,
        'count': len(waypoints)
    })

if __name__ == '__main__':
    print("\n" + "="*70)
    print("  Fishing Drone Desktop Mission Planner")
    print("="*70)
    print("  Supported formats:")
    print("    - INAV XML (.mission)")
    print("    - QGroundControl text (.mission, .waypoints)")
    print("    - INAV text format")
    print("="*70)
    print("  Server: http://localhost:5000")
    print("  Storage: ./missions/")
    print("="*70)
    print("\n  Ready for connections...\n")
    
    app.run(host='0.0.0.0', port=8000, debug=True)