import cv2
import os
import numpy as np
from flask import Flask, Response

# Initialize Flask app
app = Flask(__name__)

# Define hardware paths based on physical USB ports
LEFT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
RIGHT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0"

# Resolve absolute paths
real_left_path = os.path.realpath(LEFT_CAM_PATH)
real_right_path = os.path.realpath(RIGHT_CAM_PATH)

# Open cameras
cap_l = cv2.VideoCapture(real_left_path, cv2.CAP_V4L2)
cap_r = cv2.VideoCapture(real_right_path, cv2.CAP_V4L2)

# Configure MJPG and resolution to save USB bandwidth
cap_l.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap_l.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap_l.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

cap_r.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap_r.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap_r.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Generator for a single combined video frame
def generate_combined_frames():
    while True:
        # Read sequentially from both cameras
        success_l, frame_l = cap_l.read()
        success_r, frame_r = cap_r.read()
        
        if not success_l or not success_r:
            break
        
        # Combine frames side-by-side horizontally (1280x480)
        combined_frame = np.hstack((frame_l, frame_r))
        
        # Encode the combined frame to JPEG
        ret, buffer = cv2.imencode('.jpg', combined_frame)
        frame_bytes = buffer.tobytes()
        
        # Yield multipart HTTP response
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# Main route
@app.route('/')
def index():
    # HTML template expecting only ONE video stream
    html_content = """
    <html>
        <head><title>Pi Stereo Stream</title></head>
        <body style="text-align: center; background-color: #222; color: white; font-family: sans-serif;">
            <h2>Stereo Cameras Live Feed (No Threads)</h2>
            <img src="/video_feed" width="1280" height="480" style="border: 2px solid white; max-width: 100%;"/>
        </body>
    </html>
    """
    return html_content

# Single stream route
@app.route('/video_feed')
def video_feed():
    return Response(generate_combined_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Start server with threading explicitly DISABLED
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=False)