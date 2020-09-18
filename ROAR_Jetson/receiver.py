import socket
import serial
import struct

MOTOR_MAX = 1750;
MOTOR_MIN = 800;
MOTOR_NEUTRAL = 1500;
THETA_MAX = 3000;
THETA_MIN = 0;

COMMAND_THROTTLE = 0
COMMAND_STEERING = 1
UDP_PORT = 7788

class Receiver:
    def __init__(self, client_ip):
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.old_steering = 0.0
        self.old_throttle = 0.0
        self.new_steering = 0.0
        self.new_throttle = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_ip = client_ip

    def update(self):
        while True:
            vel_wheel = self.ser.readline()
            vel_wheel = str(vel_wheel)
            vel_wheel = vel_wheel[2:][:-5]
            vel_wheel = vel_wheel.split()
            try:
                throttle, steering = vel_wheel
                throttle = float(throttle)
                steering = float(steering)
            except:
                continue
            if self.new_throttle >= MOTOR_NEUTRAL:
                self.new_throttle = float(throttle - MOTOR_NEUTRAL) / (MOTOR_MAX - MOTOR_NEUTRAL)
            else:
                self.new_throttle = float(throttle - MOTOR_NEUTRAL) / (MOTOR_NEUTRAL - MOTOR_MIN)
            self.new_throttle = max(-1, self.new_throttle)
            self.new_throttle = min(1, self.new_throttle)
            self.new_steering = float(steering - THETA_MIN) / (THETA_MAX - THETA_MIN) * 2 - 1
            self.new_steering = max(-1, self.new_steering)
            self.new_steering = min(1, self.new_steering)

    def run_threaded(self, **args):
        if (self.new_throttle != self.old_throttle):
            msg = struct.pack('>Ii', COMMAND_THROTTLE, int(self.new_throttle * 32767))
            self.sock.sendto(msg, (self.client_ip, UDP_PORT))
            self.old_throttle = self.new_throttle
        if (self.new_steering != self.old_steering):
            msg = struct.pack('>Ii', COMMAND_STEERING, int(self.new_steering * 32767))
            self.sock.sendto(msg, (self.client_ip, UDP_PORT))
            self.old_steering = self.new_steering
