import numpy as np

class Simulation():
    def __init__(self, r, mass, thrust, final_altitude, final_speed, body, Cd=0.4) -> None:
        self.dt = 0.01
        self.S = np.array([0])
        self.V = np.array([final_speed])

        # Rocket Proprieties
        self.r = r # Radius
        self.A = np.pi * self.r**2 # Area
        self.Cd = Cd # Drag Coefficient
        self.m = mass # Initial Mass
        self.thrust = thrust

        # Body Proprieties
        self.body = body
        self.surface_gravity = self.body.surface_gravity # Surface Gravity
        self.gm = self.body.gravitational_parameter # Gravitational Parameter
        self.rb = self.body.equatorial_radius
        self.atm = self.body.has_atmosphere
        self.final_altitude = final_altitude

        # Thrust limit
        self.aeng = (self.thrust/self.m)
        if self.surface_gravity+2 > self.aeng:
            self.aeng = self.surface_gravity+2
        elif self.aeng > self.surface_gravity*4:
            self.aeng = self.surface_gravity*4

        # CALCULE
        while self.S[-1] < self.final_altitude: # Talvez usar Runge Kutta
            self.V = np.append(self.V, self.V[-1] - self.dVdt())
            self.S = np.append(self.S, self.S[-1] - self.V[-1] * self.dt)

    def dVdt(self):
        accel_net = self.aeng - self.ag(self.S[-1])

        if self.atm: # Drag Calculation
            accel_net += (self.body.density_at(float(self.S[-1]))*self.Cd*self.A*self.V[-1]**2) / (2*self.m)

        return accel_net * self.dt

    def ag(self, alt):
        return self.gm / (self.rb+alt)**2

    def get_speed(self, alt):
        return np.interp(alt, self.S, self.V)