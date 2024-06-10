from time import sleep
from math import sqrt

def smooth_func(from_, to_, func_, time_, resolution=100):
    v = from_
    dv = (to_ - from_) / resolution
    
    dt = time_ / resolution

    func_(from_)
    for i in range(resolution):
        sleep(dt)
        func_(v)
        v += dv
    func_(to_)

def clamp(min_, max_, value):
    if value < min_: return min_
    if value > max_: return max_
    return value

def sqrt_(value):
    if value <= 0:
        return 0
    return sqrt(value)

def sign(value):
    if value > 0: return 1
    if value < 0: return -1
    if value == 0: return 0

if __name__ == "__main__":
    smooth_func(1, 0, lambda x: print(x), 5)