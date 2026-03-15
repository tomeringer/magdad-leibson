import cv2
from flask import Flask, Response

# Initialize Flask app
app = Flask(__name__)

# Open cameras using V4L2 for Raspberry Pi
# Indexes are typically 0 and 2 on Linux, change if necessary
cap_l = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap_r = cv2.VideoCapture(2, cv2.CAP_V4L2)

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

# Left stream route
@app.route('/video_lefst')
def video_left():
    return Response(generate_frames(cap_l), mimetype='multipart/x-mixed-replace; boundary=frame')

# Right stream route
@app.route('/video_right')
def video_right():
    return Response(generate_frames(cap_r), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Start server on all network interfaces
    app.run(host='0.0.0.0', port=5000, debug=False)