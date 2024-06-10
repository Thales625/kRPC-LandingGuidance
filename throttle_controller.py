from time import time

from sys import path
path.append("C:/Users/thale/Dropbox/Codes/Python/kRPC/Modules")
from PID import PIDController

class ThrottleControl:
    def __init__(self, ut_func=time) -> None:
        self.ut  = ut_func
        self.pid = PIDController()
        self.pid.limit_output(0, 1)
    
    def linear_control(self, dv, a_eng, a_g, factor=5):
        return (dv*factor + a_g) / a_eng

    def pid_control(self, dv, a_eng):
        return self.pid(-dv/a_eng, 0, self.ut())