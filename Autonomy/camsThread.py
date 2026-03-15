import cv2
import os
from flask import Flask, Response

# Initialize Flask app
app = Flask(__name__)

# Define hardware paths based on physical USB ports
LEFT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
RIGHT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0"

# Resolve the absolute paths to their actual /dev/videoX mappings
real_left_path = os.path.realpath(LEFT_CAM_PATH)
real_right_path = os.path.realpath(RIGHT_CAM_PATH)

print(f"[DEBUG] Resolved Left Camera Path: {real_left_path}")
print(f"[DEBUG] Resolved Right Camera Path: {real_right_path}")

# Open cameras directly using their resolved hardware paths
cap_l = cv2.VideoCapture(real_left_path, cv2.CAP_V4L2)
cap_r = cv2.VideoCapture(real_right_path, cv2.CAP_V4L2)

# Verify if cameras are successfully opened
print(f"[DEBUG] Left camera opened: {cap_l.isOpened()}")
print(f"[DEBUG] Right camera opened: {cap_r.isOpened()}")

# Configure MJPG and resolution to save USB bandwidth
cap_l.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap_l.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap_l.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

cap_r.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap_r.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap_r.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Generator for video frames
def generate_frames(camera):
    while True:
        success, frame = camera.read()
        if not success:
            break
        
        # Encode frame to JPEG format
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        
        # Yield multipart HTTP response for the browser
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# Main route
@app.route('/')
def index():
    # HTML template for dual stream display
    html_content = """
    <html>
        <head><title>Pi Stereo Stream</title></head>
        <body style="text-align: center; background-color: #222; color: white; font-family: sans-serif;">
            <h2>Stereo Cameras Live Feed</h2>
            <div style="display: flex; justify-content: center; gap: 20px;">
                <div>
                    <h3>Left Camera</h3>
                    <img src="/video_left" width="640" height="480" style="border: 2px solid white;"/>
                </div>
                <div>
                    <h3>Right Camera</h3>
                    <img src="/video_right" width="640" height="480" style="border: 2px solid white;"/>
                </div>
            </div>
        </body>
    </html>
    """
    return html_content

# Left stream route (fixed typo here)
@app.route('/video_left')
def video_left():
    return Response(generate_frames(cap_l), mimetype='multipart/x-mixed-replace; boundary=frame')

# Right stream route
@app.route('/video_right')
def video_right():
    return Response(generate_frames(cap_r), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Start server with threading enabled for the web server only
    # This is required so Flask can serve two video streams at the exact same time
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)