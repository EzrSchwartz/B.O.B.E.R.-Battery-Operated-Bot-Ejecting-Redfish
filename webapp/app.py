"""
Flask Web Server for Fishing Drone Mission Planning
Optional desktop interface - can plan missions offline then send to ESP32
"""

from flask import Flask, render_template, request, jsonify, send_from_directory
from flask_cors import CORS
import json
import os
from datetime import datetime

app = Flask(__name__)
CORS(app)

# Storage directories
MISSIONS_DIR = 'missions'
os.makedirs(MISSIONS_DIR, exist_ok=True)

class INAVMissionParser:
    """Parse INAV .mission files"""
    
    @staticmethod
    def parse_mission_file(filepath):
        """
        Parse INAV mission file
        Returns list of waypoints
        """
        waypoints = []
        
        with open(filepath, 'r') as f:
            lines = f.readlines()
        
        for line in lines:
            line = line.strip()
            
            # Skip comments
            if line.startswith('#') or line.startswith('QGC') or not line:
                continue
            
            # Parse waypoint line
            parts = line.split('\t')
            
            if len(parts) >= 12:
                try:
                    wp = {
                        'index': int(parts[0]),
                        'lat': float(parts[8]),
                        'lon': float(parts[9]),
                        'alt': float(parts[10]),
                        'type': 'waypoint'
                    }
                    waypoints.append(wp)
                except ValueError:
                    continue
        
        return waypoints

@app.route('/')
def index():
    """Serve main interface"""
    return render_template('index.html')

@app.route('/api/upload_mission', methods=['POST'])
def upload_mission():
    """Upload and parse INAV mission file"""
    if 'file' not in request.files:
        return jsonify({'error': 'No file provided'}), 400
    
    file = request.files['file']
    
    if file.filename == '':
        return jsonify({'error': 'No file selected'}), 400
    
    # Save file
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f"mission_{timestamp}.mission"
    filepath = os.path.join(MISSIONS_DIR, filename)
    file.save(filepath)
    
    # Parse mission
    try:
        waypoints = INAVMissionParser.parse_mission_file(filepath)
        
        return jsonify({
            'success': True,
            'filename': filename,
            'waypoints': waypoints,
            'count': len(waypoints)
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/save_mission', methods=['POST'])
def save_mission():
    """Save drop locations to file"""
    data = request.json
    drops = data.get('drops', [])
    mission_name = data.get('mission_name', 'fishing_mission')
    
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f"{mission_name}_{timestamp}.json"
    filepath = os.path.join(MISSIONS_DIR, filename)
    
    with open(filepath, 'w') as f:
        json.dump({
            'mission_name': mission_name,
            'timestamp': timestamp,
            'drops': drops
        }, f, indent=2)
    
    return jsonify({
        'success': True,
        'filename': filename,
        'count': len(drops)
    })

@app.route('/api/missions', methods=['GET'])
def get_missions():
    """List all saved missions"""
    missions = []
    
    for filename in os.listdir(MISSIONS_DIR):
        if filename.endswith('.json'):
            filepath = os.path.join(MISSIONS_DIR, filename)
            try:
                with open(filepath, 'r') as f:
                    data = json.load(f)
                    missions.append({
                        'filename': filename,
                        'mission_name': data.get('mission_name'),
                        'timestamp': data.get('timestamp'),
                        'drop_count': len(data.get('drops', []))
                    })
            except:
                continue
    
    return jsonify({'missions': missions})

@app.route('/api/mission/<filename>', methods=['GET'])
def get_mission(filename):
    """Load specific mission"""
    filepath = os.path.join(MISSIONS_DIR, filename)
    
    if not os.path.exists(filepath):
        return jsonify({'error': 'Mission not found'}), 404
    
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    return jsonify(data)

if __name__ == '__main__':
    print("="*60)
    print("  Fishing Drone Desktop Mission Planner")
    print("="*60)
    print("  Server: http://localhost:5000")
    print("  Storage: ./missions/")
    print("="*60)
    app.run(host='0.0.0.0', port=5000, debug=True)