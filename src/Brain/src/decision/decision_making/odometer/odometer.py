import time

class Odometer:

    def __init__(self):
        self.distance = 0
        self.last_time = 0

    def update(self, speed):
        current_time = time.time()
        self.distance += speed * (current_time - self.last_time)
        self.last_time = current_time