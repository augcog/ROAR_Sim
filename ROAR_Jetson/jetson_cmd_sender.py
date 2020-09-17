import serial
import logging

MOTOR_MAX = 1750
MOTOR_MIN = 800
MOTOR_NEUTRAL = 1500
THETA_MAX = 3000
THETA_MIN = 0


class JetsonCommandSender:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1, writeTimeout=1)
        self.prev_throttle = 1500  # record previous throttle, set to neutral initially 
        self.prev_steering = 1500  # record previous steering, set to neutral initially
        self.logger = logging.getLogger("Jetson CMD Sender")

    def update(self):
        pass

    def run_threaded(self, throttle, steering, **args):
        if throttle >= 0:
            throttle_send = int(MOTOR_NEUTRAL + (MOTOR_MAX - MOTOR_NEUTRAL) * throttle)
        else:
            throttle_send = int(MOTOR_NEUTRAL + (MOTOR_NEUTRAL - MOTOR_MIN) * throttle)
        steering_send = int(THETA_MIN + (steering / 2 + 0.5) * (THETA_MAX - THETA_MIN))
        # only send new msg when throttle or steering changes
        # if self.prev_throttle != throttle_send or self.prev_steering != steering_send:
        serial_msg ='& {} {}'.format(throttle_send, steering_send)
        self.logger.debug(f"Sending [{serial_msg}]")
        self.ser.write(serial_msg.encode('ascii'))
        self.prev_throttle = throttle_send
        self.prev_steering = steering_send
        try:
            self.ser.read(100)
        except KeyboardInterrupt as e:
            self.logger.debug("Interrupted Using Keyboard")
            exit(0)
        except Exception as e:
            self.logger.error(f"Something bad happened {e}")

    def shutdown(self):
        print('sender shutdown...')
        for i in range(5):
            self.ser.write('& 1500 1500\n'.encode('ascii'))
            self.ser.read(100)
