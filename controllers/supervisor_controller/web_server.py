"""
Flask web server for visualizing supervisor controller data in real-time.
Run this alongside supervisor_controller.py to view live robot and ball positions.
"""

from flask import Flask, render_template, jsonify
import os
import json
import math

app = Flask(__name__, template_folder=os.path.join(os.path.dirname(__file__), 'templates'))

# Base path for reading data files
DATA_DIR = os.path.dirname(__file__)

def read_ball_positions():
    """Read ball positions from ball_position.txt"""
    file_path = os.path.join(DATA_DIR, "ball_position.txt")
    balls = []
    try:
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        # Parse: (x, y, type)
                        import re
                        match = re.match(r'\(([-+]?[\d.]+),\s*([-+]?[\d.]+),\s*(\w+)\)', line)
                        if match:
                            x, y, ball_type = float(match.group(1)), float(match.group(2)), match.group(3)
                            balls.append({"x": x, "y": y, "type": ball_type})
                    except Exception:
                        pass
    except Exception as e:
        print(f"Error reading balls: {e}")
    return balls

def read_robot_position():
    """Read main robot current position"""
    file_path = os.path.join(DATA_DIR, "current_position.txt")
    try:
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                line = f.read().strip()
                if line:
                    import re
                    match = re.match(r'\(([-+]?[\d.]+),\s*([-+]?[\d.]+)\)', line)
                    if match:
                        x, y = float(match.group(1)), float(match.group(2))
                        return {"x": x, "y": y}
    except Exception as e:
        print(f"Error reading robot position: {e}")
    return {"x": 0, "y": 0}

def read_obstacle_position():
    """Try to read obstacle robot position if it exists"""
    # Obstacle robot position would need to be written to a file
    # For now, we'll return None or read from a placeholder
    return None

def read_webots_time():
    """Read current Webots simulation time"""
    file_path = os.path.join(DATA_DIR, "time.txt")
    try:
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                line = f.read().strip()
                if line:
                    return float(line)
    except Exception as e:
        print(f"Error reading time: {e}")
    return 0.0

def read_waypoint_status():
    """Read current waypoint status"""
    file_path = os.path.join(DATA_DIR, "waypoint_status.txt")
    try:
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                line = f.read().strip()
                return line if line else "idle"
    except Exception:
        pass
    return "idle"

def read_scores():
    """Read score information if available"""
    # This would need to be written to a file by supervisor_controller.py
    return {
        "score": 0,
        "ping_hit": 0,
        "steel_hit": 0,
        "steel_stored": 0,
        "ping_stored": 0
    }

@app.route('/')
def index():
    """Serve the main visualization page"""
    return render_template('visualization.html')

@app.route('/api/data')
def get_data():
    """API endpoint to get all current data"""
    data = {
        "balls": read_ball_positions(),
        "robot": read_robot_position(),
        "obstacle": read_obstacle_position(),
        "time": read_webots_time(),
        "status": read_waypoint_status(),
        "scores": read_scores(),
        "arena": {
            "x_min": -0.86,
            "x_max": 0.86,
            "y_min": -0.86,
            "y_max": 0.86
        }
    }
    return jsonify(data)

@app.route('/api/balls')
def get_balls():
    """API endpoint for balls only"""
    return jsonify(read_ball_positions())

@app.route('/api/robot')
def get_robot():
    """API endpoint for robot position"""
    return jsonify(read_robot_position())

@app.route('/api/time')
def get_time():
    """API endpoint for simulation time"""
    return jsonify({"time": read_webots_time()})

if __name__ == '__main__':
    print("Starting Flask web server on http://localhost:9999")
    print(f"Data directory: {DATA_DIR}")
    app.run(debug=False, host='localhost', port=9999, threaded=True)
