import krpc
from time import sleep
from math import radians, sin, sqrt
from Simulation import Simulation
import numpy as np


class LandingGuidance:
    def __init__(self, vessel=None):
        self.conn = krpc.connect('LandingGuidance')
        self.space_center = self.conn.space_center
        self.vessel = self.space_center.active_vessel if vessel == None else vessel
        self.body = self.vessel.orbit.body
        self.body_ref = self.body.reference_frame
        self.surface_ref = self.vessel.surface_reference_frame
        self.flight = self.vessel.flight(self.body_ref)

        # Check Target
        self.target = self.space_center.target_vessel
        if self.target is None:
            print('Selecione um alvo!')
            exit()

        # Streams
        self.mass = self.conn.add_stream(getattr, self.vessel, "mass")
        self.velocity = self.conn.add_stream(getattr, self.flight, "velocity")
        self.vertical_speed = self.conn.add_stream(getattr, self.flight, "vertical_speed")
        self.mag_speed = self.conn.add_stream(getattr, self.flight, "speed")
        self.surface_altitude = self.conn.add_stream(getattr, self.flight, "surface_altitude")
        self.pitch = self.conn.add_stream(getattr, self.vessel.flight(self.surface_ref), "pitch")

        # Propriedade Computacional
        self.eng_threshold = 1
        self.final_speed = -2
        self.hover_altitude = 30
        self.final_burn = False
        self.accelerating = False

        # Initializing
        self.vessel.control.throttle = 0
        self.vessel.control.brakes = True
        self.vessel.control.rcs = True
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.target_roll = -90
        self.vessel.auto_pilot.reference_frame = self.body_ref

        while self.vertical_speed() > 0 or self.altitude() > 8000: # WAIT
            self.aim_vessel(self.vertical_speed())

        # Propriedades do Corpo
        self.surface_gravity = self.body.surface_gravity

        # Propriedade do Foguete
        self.gears_delay = 4 /2
        
        # Thrust de acordo com ângulo de montagem do motor
        self.thrust = 0
        for engine in self.vessel.parts.engines:
            if engine.active:
                self.thrust += engine.available_thrust * engine.part.direction(self.vessel.reference_frame)[1]

        # Simulation
        self.simulation = Simulation(self.rocket_radius(), self.mass(), self.thrust*self.eng_threshold, self.altitude(), self.final_speed, self.body)

        while True:
            sleep(0.01)
            if self.vessel.situation == self.vessel.situation.landed or self.vessel.situation == self.vessel.situation.splashed:
                self.vessel.control.throttle = 0
                self.vessel.control.brakes = False
                print(f'{self.vessel.name} Pousou!')
                self.vessel.auto_pilot.disengage()
                self.vessel.control.sas = True
                sleep(0.1)
                try:
                    self.vessel.control.sas_mode = self.vessel.control.sas_mode.radial
                except:
                    self.vessel.auto_pilot.engage()
                    self.vessel.auto_pilot.target_direction = self.space_center.transform_direction((1, 0, 0), self.surface_ref, self.body_ref)
                    self.vessel.control.rcs = True
                    sleep(4)
                    self.vessel.control.rcs = False
                    self.vessel.auto_pilot.disengage()
                break
            
            vert_speed = self.vertical_speed()
            aeng = self.vessel.available_thrust/self.vessel.mass
            pitch = self.pitch()

            self.aim_vessel(vert_speed)

            if vert_speed < 0 and pitch > 0:
                if self.final_burn:
                    self.vessel.gear = True
                    self.vessel.control.throttle = self.throttle_control(self.final_speed - vert_speed, pitch, 5)
                else:
                    alt = self.altitude()

                    if alt <= self.hover_altitude+2:
                        self.final_burn = True
                    elif self.time_fall(-.5 * self.surface_gravity, vert_speed, alt) <= self.gears_delay:
                        self.vessel.control.gear = True

                    target_speed = self.simulation.get_speed(alt-self.hover_altitude)
                    delta_speed = target_speed + self.mag_speed()

                    throttle = self.throttle_control(delta_speed, pitch, 10)
                    if throttle > 0:
                        self.accelerating = True
                    self.vessel.control.throttle = throttle
                    #print(f'{delta_speed:.2f}')
            else:
                self.vessel.control.throttle = 0

    def throttle_control(self, accel, pitch, factor=1):
        aeng = self.thrust / self.mass()
        throttle = (self.surface_gravity + accel*factor) / (aeng * sin(radians(pitch)))
        return throttle

    def rocket_radius(self):
        size = 0.5
        for fuel in self.vessel.resources.with_resource('LiquidFuel'):
            size = max(0.5, fuel.part.bounding_box(self.vessel.reference_frame)[1][0] - fuel.part.bounding_box(self.vessel.reference_frame)[0][0])
        return abs(size)/2

    def time_fall(self, a, v, h):
        d = sqrt((v * v) - 4 * a * h)
        result_1 = (-v + d) / (2 * a)
        result_2 = (-v - d) / (2 * a)
        return max(result_1, result_2)

    def aim_vessel(self, v_speed):
        if self.accelerating:
            target_pos = np.array(self.target.position(self.surface_ref))
            target_dir = self.normalize(target_pos)
            prograde_dir = self.prograde_dir() # Talvez de para pegar pelo krpc
            error_dir = target_dir - prograde_dir
            target_dir = [2, 0, 0] + error_dir/2
        else:
            if v_speed < 0:
                vel = self.space_center.transform_direction(self.velocity(), self.body_ref, self.surface_ref)
                target_dir = ((10 if self.final_burn else 1) * -vel[0], -vel[1], -vel[2])
            else:
                target_dir = (1, 0, 0)

        self.vessel.auto_pilot.target_direction = self.space_center.transform_direction(target_dir, self.surface_ref, self.body_ref)

    def altitude(self):
        return max(0, self.surface_altitude() + self.vessel.bounding_box(self.surface_ref)[0][0])
    
    def normalize(self, vector):
        return vector / np.linalg.norm(vector)

    def prograde_dir(self):
        vel = self.space_center.transform_direction(self.vessel.velocity(self.body_ref), self.body_ref, self.surface_ref)
        return self.normalize(vel)

if __name__ == '__main__':
    LandingGuidance()