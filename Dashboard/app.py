from flask import Flask, render_template, send_from_directory
from flask_socketio import SocketIO
import serial
import threading
from datetime import datetime
import time

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*")

SERIAL_PORT = '/dev/cu.usbmodem11201'  # Change this to match your system
BAUD_RATE = 115200
current_data = {
    'timestamp': 0,
    'temperature': 0, 'pressure': 0, 'humidity': 0, 'altitude': 0,
    'orientation': [0, 0, 0], 'acceleration': [0, 0, 0],
    'gps': {'lat': 0, 'lon': 0, 'satellites': 0, 'valid': False}
}

def parse_csv(line):
    try:
        parts = line.decode().strip().split(',')
        if len(parts) != 16:
            return None
            
        return {
            'timestamp': int(parts[0]),
            'state': int(parts[1]),
            'lat': float(parts[2]),
            'lon': float(parts[3]),
            'satellites': int(parts[4]),
            'gps_valid': bool(int(parts[5])),
            'altitude': float(parts[6]),
            'temperature': float(parts[7]),
            'pressure': float(parts[8]),
            'humidity': float(parts[9]),
            'roll': float(parts[10]),
            'pitch': float(parts[11]),
            'yaw': float(parts[12]),
            'accX': float(parts[13]),
            'accY': float(parts[14]),
            'accZ': float(parts[15])
        }
    except Exception as e:
        print(f"Parse error: {e}")
        return None

def serial_reader():
    while True:
        try:
            with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
                print(f"Connected to {SERIAL_PORT}")
                while True:
                    line = ser.readline()
                    if line:
                        parsed = parse_csv(line)
                        if parsed:
                            current_data.update({
                                'temperature': parsed['temperature'],
                                'pressure': parsed['pressure'],
                                'humidity': parsed['humidity'],
                                'altitude': parsed['altitude'],
                                'orientation': [parsed['roll'], parsed['pitch'], parsed['yaw']],
                                'acceleration': [parsed['accX'], parsed['accY'], parsed['accZ']],
                                'gps': {
                                    'lat': parsed['lat'],
                                    'lon': parsed['lon'],
                                    'satellites': parsed['satellites'],
                                    'valid': parsed['gps_valid']
                                }
                            })
                            socketio.emit('update', current_data)
        except Exception as e:
            print(f"Serial error: {e}, retrying in 5 seconds...")
            time.sleep(5)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/models/<path:path>')
def send_model(path):
    return send_from_directory('models', path)

if __name__ == '__main__':
    thread = threading.Thread(target=serial_reader)
    thread.daemon = True
    thread.start()
    socketio.run(app, host='0.0.0.0', port=3663, debug=True)