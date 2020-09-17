class Controller:
    def __init__(self):
        self.new_steering = 0.0
        self.new_throttle = 0.0

    def update(self):
        # update throttle and steering
        pass

    def run_threaded(self, **args):
        return self.new_throttle, self.new_steering

class NaiveController(Controller):
    def __init__(self):
        Controller.__init__(self)

    def update(self):
        while True:
            try:
                x, y = input().split()
                self.new_throttle, self.new_steering = float(x), float(y)
            except:
                pass
