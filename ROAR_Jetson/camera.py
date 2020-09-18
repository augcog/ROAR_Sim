import os
import time
import numpy as np
import cv2
from PIL import Image
import glob
import logging
import pyrealsense2 as rs


class BaseCamera:
    def run_threaded(self):
        return self.frame


class CSICamera(BaseCamera):
    '''
    Camera for Jetson Nano IMX219 based camera
    Credit: https://github.com/feicccccccc/donkeycar/blob/dev/donkeycar/parts/camera.py
    gstreamer init string from https://github.com/NVIDIA-AI-IOT/jetbot/blob/master/jetbot/camera.py
    '''

    def gstreamer_pipeline(self, capture_width=3280, capture_height=2464, output_width=224, output_height=224,
                           framerate=21, flip_method=0):
        return 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv flip-method=%d ! nvvidconv ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink' % (
            capture_width, capture_height, framerate, flip_method, output_width, output_height)

    def gstreamer_pipelineout(self, output_width=1280, output_height=720, framerate=21, client_ip='127.0.0.1'):
        return 'appsrc ! videoconvert ! video/x-raw, format=(string)BGRx, width=%d, height=%d, framerate=(fraction)%d/1 ! videoconvert ! video/x-raw, format=(string)I420 ! omxh264enc tune=zerolatency bitrate=8000000 speed-preset=ultrafast ! video/x-h264, stream-format=byte-stream ! rtph264pay mtu=1400 ! udpsink host=%s port=5000 sync=false async=false' % (
        output_width, output_height, framerate, client_ip)

    def __init__(self, image_w=160, image_h=120, image_d=3, capture_width=640, capture_height=480, framerate=60,
                 gstreamer_flip=0, client_ip='127.0.0.1'):
        '''
        gstreamer_flip = 0 - no flip
        gstreamer_flip = 1 - rotate CCW 90
        gstreamer_flip = 2 - flip vertically
        gstreamer_flip = 3 - rotate CW 90
        '''
        self.w = image_w
        self.h = image_h
        self.running = True
        self.frame = None
        self.flip_method = gstreamer_flip
        self.capture_width = capture_width
        self.capture_height = capture_height
        self.framerate = framerate
        self.client_ip = client_ip

    def init_camera(self):
        import cv2

        # initialize the camera and stream
        self.camera = cv2.VideoCapture(
            self.gstreamer_pipeline(
                capture_width=self.capture_width,
                capture_height=self.capture_height,
                output_width=self.w,
                output_height=self.h,
                framerate=self.framerate,
                flip_method=self.flip_method),
            cv2.CAP_GSTREAMER)
        self.out_send = cv2.VideoWriter(self.gstreamer_pipelineout(
            output_width=self.w,
            output_height=self.h,
            framerate=self.framerate,
            client_ip=self.client_ip), cv2.CAP_GSTREAMER, 0, self.framerate, (self.w, self.h), True)

        self.poll_camera()
        print('CSICamera loaded.. .warming camera')
        time.sleep(2)

    def update(self):
        self.init_camera()
        while self.running:
            self.poll_camera()

    def poll_camera(self):
        import cv2
        self.ret, frame = self.camera.read()
        self.out_send.write(frame)
        self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def run(self):
        self.poll_camera()
        return self.frame

    def run_threaded(self):
        return self.frame

    def shutdown(self):
        self.running = False
        print('stoping CSICamera')
        time.sleep(.5)
        del (self.camera)
        self.out_send.release()


class RS_D435i(object):
    '''
    Intel RealSense depth camera D435i combines the robust depth sensing capabilities of the D435 with the addition of an inertial measurement unit (IMU).
    ref: https://www.intelrealsense.com/depth-camera-d435i/
    '''

    def gstreamer_pipelineout(self, output_width=1280, output_height=720, framerate=21, client_ip='127.0.0.1'):
        return 'appsrc ! videoconvert ! video/x-raw, format=(string)BGRx, width=%d, height=%d, framerate=(fraction)%d/1 ! videoconvert ! video/x-raw, format=(string)I420 ! omxh264enc tune=zerolatency bitrate=8000000 speed-preset=ultrafast ! video/x-h264, stream-format=byte-stream ! rtph264pay mtu=1400 ! udpsink host=%s port=5001 sync=false async=false' % (
        output_width, output_height, framerate, client_ip)

    def __init__(self, image_w=640, image_h=480, image_d=3, image_output=True, framerate=30, client_ip='127.0.0.1'):
        # Using the image_output will grab two image streams from the fisheye cameras but return only one.
        # This can be a bit much for USB2, but you can try it. Docs recommend USB3 connection for this.
        self.image_output = image_output
        self.client_ip = client_ip
        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.gyro)
        cfg.enable_stream(rs.stream.accel)

        if self.image_output:
            cfg.enable_stream(rs.stream.color, image_w, image_h, rs.format.bgr8, framerate)  # color camera

            self.out_send = cv2.VideoWriter(self.gstreamer_pipelineout(
                output_width=image_w,
                output_height=image_h,
                framerate=framerate,
                client_ip=self.client_ip), cv2.CAP_GSTREAMER, 0, framerate, (image_w, image_h), True)

            cfg.enable_stream(rs.stream.depth, image_w, image_h, rs.format.z16, framerate)  # depth camera

        # Start streaming with requested config
        self.pipe.start(cfg)
        self.running = True

        zero_vec = (0.0, 0.0, 0.0)
        self.gyro = zero_vec
        self.acc = zero_vec
        self.img = None
        self.dimg = None
        self.logger = logging.getLogger("Intel RealSense D435i")
        self.logger.info("Camera Initiated")

    def poll(self):
        try:
            frames = self.pipe.wait_for_frames()
        except Exception as e:
            logging.error(e)
            return

        if self.image_output:
            color_frame = frames.get_color_frame()

            depth_frame = frames.get_depth_frame()
            self.img = np.asanyarray(color_frame.get_data())
            self.dimg = np.asanyarray(depth_frame.get_data())
            self.out_send.write(self.img)

        # Fetch IMU frame
        accel = frames.first_or_default(rs.stream.accel)
        gyro = frames.first_or_default(rs.stream.gyro)
        if accel and gyro:
            self.acc = accel.as_motion_frame().get_motion_data()
            self.gyro = gyro.as_motion_frame().get_motion_data()
            # print('realsense accel(%f, %f, %f)' % (self.acc.x, self.acc.y, self.acc.z))
            # print('realsense gyro(%f, %f, %f)' % (self.gyro.x, self.gyro.y, self.gyro.z))

    def update(self):
        while self.running:
            self.poll()

    def run_threaded(self):

        try:
            self.poll()
            return self.img, self.dimg
        except KeyboardInterrupt as e:
            raise e
        except Exception as e:
            return None, None

    def run(self):
        self.poll()
        return self.run_threaded()

    def shutdown(self):
        self.logger.debug("Shutting Down")
        self.running = False
        time.sleep(0.1)
        self.pipe.stop()
