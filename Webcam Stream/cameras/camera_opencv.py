import cv2
import tensorflow as tf
import numpy as np

from node_lookup import NodeLookup
from cameras.base_camera import BaseCamera


class Camera(BaseCamera):
    video_source = 0
    image = None

    @staticmethod
    def set_video_source(source):
        Camera.video_source = source

    @classmethod
    def save_image(cls):
        image_name = 'out.jpg'
        out = cv2.imwrite(image_name, cls.image)
        return image_name

    @staticmethod
    def frames():
        print('Frames: ', Camera.video_source)
        camera = cv2.VideoCapture(Camera.video_source)
        if not camera.isOpened():
            raise RuntimeError('Could not start camera.')

        while True:
            # read current frame
            _, img = camera.read()

            # encode as a jpeg image and return it
            Camera.image = img
            yield cv2.imencode('.jpg', img)[1].tobytes()
