#!/usr/bin/env python
# THIS GUY IS A FUCKING GENIUS.
# https://blog.miguelgrinberg.com/post/flask-video-streaming-revisited
from importlib import import_module
import os
from flask import Flask, render_template, Response, jsonify, json
from predict import Predictor

# import camera driver
if os.environ.get('CAMERA'):
    Camera = import_module('cameras.camera_' + os.environ['CAMERA']).Camera
else:
    # from camera import Camera
    from cameras.camera_opencv import Camera

# Raspberry Pi camera module (requires picamera package)
# from camera_pi import Camera

app = Flask(__name__)


@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')


def gen(camera):
    """Video streaming generator function."""
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed', defaults={'source': 0})
@app.route('/video_feed/<int:source>')
def video_feed(source):
    """Video streaming route. Put this in the src attribute of an img tag."""
    Camera.set_video_source(source)
    camera = Camera()
    return Response(gen(camera),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/save_image')
def save_image():
    camera = Camera()
    image_name = camera.save_image()
    data = Predictor.predict(image_name)
    response = app.response_class(
        response=json.dumps(data),
        status=200,
        mimetype='application/json'
    )
    return response

if __name__ == '__main__':
    app.run(host='0.0.0.0', threaded=True, debug=False)
