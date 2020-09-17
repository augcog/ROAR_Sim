# """ 
# My CAR CONFIG 

# This file is read by your car application's manage.py script to change the car
# performance

# If desired, all config overrides can be specified here. 
# The update operation will not touch this file.
# """

# VEHICLE
DRIVE_LOOP_HZ = 20
MAX_LOOPS = None

# CAMERA
IMAGE_W = 1280
IMAGE_H = 720
IMAGE_DEPTH = 3         # default RGB=3, make 1 for mono
CAMERA_FRAMERATE = 30
CLIENT_IP="192.168.50.205" #for VR
CSIC_CAM_GSTREAMER_FLIP_PARM=0
