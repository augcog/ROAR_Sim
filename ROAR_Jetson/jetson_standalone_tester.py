#!/usr/bin/env python3
import os
import time
import argparse
import numpy as np

from config import load_config
import vehicle
from camera import RS_D435i, CSICamera
from jetson_cmd_sender import JetsonCommandSender as Sender
from receiver import Receiver
from controller import NaiveController

def drive(cfg, client_ip=None, to_control=False):

    #Initialize car
    V = vehicle.Vehicle()
    client_ip = client_ip or cfg.CLIENT_IP

    # camA = CSICamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, framerate=cfg.CAMERA_FRAMERATE, gstreamer_flip=cfg.CSIC_CAM_GSTREAMER_FLIP_PARM,client_ip=client_ip)
    # camB = RS_D435i(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH, framerate=cfg.CAMERA_FRAMERATE,client_ip=client_ip)

    #V.add(camA, outputs=['cam/image_array_a'], threaded=True)
    #V.add(camB, outputs=['cam/image_array_b', 'cam/image_array_c'], threaded=True)

    if to_control:
        V.add(NaiveController(), outputs=['throttle', 'steering'], threaded=True)
        V.add(Sender(), inputs=['throttle', 'steering'], threaded=True)
    else:
        V.add(Receiver(client_ip), threaded=True)

    #run the vehicle for 20 seconds
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ, 
            max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    cfg = load_config('myconfig.py')

    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', required=False, default=None, type=str)
    parser.add_argument('-c', '--control', required=False, action='store_true')
    args = parser.parse_args()
    print(args.control)
    drive(cfg, args.ip, args.control)
