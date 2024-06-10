from numpy import interp, linspace
from scipy.integrate import odeint

class LandingCalculator:
    def __init__(self, v0, body):
        self.GM      = body.gravitational_parameter
        self.r_body  = body.equatorial_radius
        self.has_atm = body.has_atmosphere

        if self.has_atm:
            self.rho = body.density_at
            self.p   = body.pressure_at
            self.p0  = 1 / 101325

        self.v0 = v0

        self.resolution = 5000
        self.h_eval = None
        self.v_h_target = None
        self.calculated = False

    def dVdh(self, h, v, m, thrust_func):
        if v == 0:
            return 0 # dvdh
        else:
            a_g = self.GM / (self.r_body + h)**2

            if self.has_atm:
                a_eng = thrust_func(self.p(h) * self.p0) / m
                a_drag = self.k * self.rho(h) * v*abs(v) / m
                dvdt = a_eng - a_drag - a_g # dvdt with drag
            else:
                a_eng = thrust_func(0) / m
                dvdt = a_eng - a_g # dvdt without drag

            return dvdt / v

    def calculate(self, h_f, mass, area_cd, thrust_func, plot_graph=False): # final altitude
        self.k = area_cd * 0.5

        self.h_eval = linspace(0, h_f, self.resolution)

        self.v_h_target = [i[0] for i in odeint(self.dVdh, t=self.h_eval, y0=[self.v0 if self.v0 != 0 else 1e-3], args=(mass, thrust_func), tfirst=True)]

        self.calculated = True

    def get_v_target(self, h):
        if not self.calculated:
            print("V_target não foi calculado!")
            return -1
        
        return interp(h, self.h_eval, self.v_h_target)