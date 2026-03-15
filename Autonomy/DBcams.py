import cv2
import os
import numpy as np
from flask import Flask, Response

# Initialize Flask app
app = Flask(__name__)

# Define hardware paths based on physical USB ports
LEFT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
RIGHT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0"

# Resolve absolute paths
real_left_path = os.path.realpath(LEFT_CAM_PATH)
real_right_path = os.path.realpath(RIGHT_CAM_PATH)

# Open cameras
cap_l = cv2.VideoCapture(real_left_path, cv2.CAP_V4L2)
cap_r = cv2.VideoCapture(real_right_path, cv2.CAP_V4L2)

# Configure cameras to minimize USB bandwidth
for cap in [cap_l, cap_r]:
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # Limit buffer size to prevent stale frames causing timeouts
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# Generate a continuous combined frame stream
def generate_combined_frames():
    # Create a blank fallback frame
    blank_frame = np.zeros((480, 640, 3), dtype=np.uint8)

    while True:
        # Read from left camera
        success_l, frame_l = cap_l.read()
        if not success_l or frame_l is None:
            frame_l = blank_frame.copy()
            cv2.putText(frame_l, "LEFT: NO SIGNAL", (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        # Read from right camera
        success_r, frame_r = cap_r.read()
        if not success_r or frame_r is None:
            frame_r = blank_frame.copy()
            cv2.putText(frame_r, "RIGHT: NO SIGNAL", (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        # Combine frames horizontally
        combined_frame = np.hstack((frame_l, frame_r))
        
        # Encode to JPEG
        ret, buffer = cv2.imencode('.jpg', combined_frame)
        frame_bytes = buffer.tobytes()
        
        # Yield stream
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# Main web route
@app.route('/')
def index():
    html_content = """
    <html>
        <head><title>Pi Stereo Stream Debug</title></head>
        <body style="text-align: center; background-color: #222; color: white; font-family: sans-serif;">
            <h2>Stereo Cameras Live Feed (Debug Mode)</h2>
            <img src="/video_feed" width="1280" height="480" style="border: 2px solid white; max-width: 100%;"/>
        </body>
    </html>
    """
    return html_content

# Video feed route
@app.route('/video_feed')
def video_feed():
    return Response(generate_combined_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Threading explicitly disabled
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=False)