import krpc
from time import sleep
from math import radians, sin, sqrt
import numpy as np


# -ToDo-
# > Desligar o motor gradualmente quando pousar

class LandingGuidance:
    def __init__(self, vessel=None, target=None):
        self.conn = krpc.connect('LandingGuidance')
        self.space_center = self.conn.space_center
        self.vessel = self.space_center.active_vessel if vessel == None else vessel
        self.body = self.vessel.orbit.body
        self.body_ref = self.body.reference_frame
        self.surface_ref = self.vessel.surface_reference_frame
        self.flight = self.vessel.flight(self.body_ref)
        self.ag = self.body.surface_gravity
        self.drawing = self.conn.drawing

        # Check Target
        if target is None:
            self.target = self.space_center.target_vessel
            if self.target is None:
                print('Selecione um alvo!')
                exit()
        else:
            self.target = target

        # Streams
        self.mass = self.conn.add_stream(getattr, self.vessel, "mass")
        self.velocity = self.conn.add_stream(getattr, self.flight, "velocity") # Transform to surface_vel in a function
        self.surface_altitude = self.conn.add_stream(getattr, self.flight, "surface_altitude")
        self.pitch = self.conn.add_stream(getattr, self.vessel.flight(self.surface_ref), "pitch")

        # Initializing -> Apply default values
        self.vessel.control.throttle = 0
        self.vessel.control.brakes = True
        self.vessel.control.rcs = True
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.target_roll = -90
        self.vessel.auto_pilot.stopping_time = (0.5, 0.5, 0.5)
        self.vessel.auto_pilot.reference_frame = self.body_ref

        # Wait <- Aim up
        self.vessel.auto_pilot.target_direction = self.space_center.transform_direction((1, 0, 0), self.surface_ref, self.body_ref)
        
        #while self.vertical_speed() > 0:# or self.altitude() > 8000: # Wait vertical speed decrease
        #    pass

        # Variables
        self.final_burn = False
        self.final_approach = False
        self.accelerating = False
        self.point = np.array(self.target.position(self.body_ref))
        #self.point = np.array(self.body.surface_position(-0.09679294489551704, -74.61739078306573, self.body_ref)) # KSC Landing Site

        # Params
        self.eng_threshold = 0.9
        self.hover_altitude = 15
        self.final_speed = -2
        self.target_speed = 10
        
        self.target_radius = 8 # ToDo: Get with target info (BoundingBox)
        self.reorient_delay = 2
        self.gears_delay = 4 /2

        #Drawing
        self.draw = False
        if self.draw:
            self.draw_target_dir = self.drawing.add_direction((0, 0, 0), self.surface_ref)
            self.draw_prograde_dir = self.drawing.add_direction((0, 0, 0), self.surface_ref)
            self.draw_aim_dir = self.drawing.add_direction((0, 0, 0), self.surface_ref)
            self.draw_target_dir.color = (0, 0, 255) # Blue
            self.draw_prograde_dir.color = (255, 0, 0) # Red
            self.draw_aim_dir.color = (0, 255, 0) # Green
            self.draw_line_lenght = 10

        # Thrust de acordo com ângulo de montagem do motor
        self.thrust = 0
        for engine in self.vessel.parts.engines:
            if engine.active:
                self.thrust += engine.available_thrust * engine.part.direction(self.vessel.reference_frame)[1]

        while True:
            sleep(0.01)
            
            # Get streams
            vel = self.get_velocity()
            pitch = self.pitch()
            mass = self.mass()
            a_eng = self.thrust * self.eng_threshold / mass

            vert_speed = vel[0]
            hor_speed = np.linalg.norm(vel[1:])
            mag_speed = np.linalg.norm(vel)

            point_pos = self.get_point_pos()
            alt = max(0, -point_pos[0] + self.vessel.bounding_box(self.surface_ref)[0][0])
            time_fall = self.time_fall(-.5 * self.ag, vert_speed, alt)
            landing_prediction = self.landing_prediction(time_fall, alt)

            # Aim to point
            point_dist_hor = np.linalg.norm(point_pos[1:])
            target_dir = self.normalize(point_pos)
            prograde_dir = self.normalize(landing_prediction)
            error_dir = target_dir - prograde_dir
            

            # Aim
            if self.accelerating:
                self.vessel.auto_pilot.stopping_time = (1, 1, 1)
                aim_dir = [3, 0, 0] + error_dir*[0, 1, 1]

                if self.final_burn:
                    target_accel = ((self.hover_altitude - alt)/4 - vert_speed) # Altitude Control
                    if point_dist_hor <= 5 and alt <= self.hover_altitude+5:
                        self.final_approach = True

                    if self.final_approach:
                        target_accel = self.final_speed - vert_speed
                        if point_dist_hor >= self.target_radius: # Sair do raio do alvo
                            self.final_approach = False
                    
                    self.vessel.control.throttle = self.throttle_control(target_accel, a_eng, pitch, 5)

                if not self.final_approach and np.linalg.norm(prograde_dir[1:] - target_dir[1:]) < 0.5: # Goto target
                    target_speed = min(point_dist_hor/8, self.target_speed)
                    speed_error = max(min(hor_speed - target_speed, 5), -5) / 4
                    aim_dir -= prograde_dir*[0, 1, 1] * speed_error 


            else: # Atmosfera <- Limitar pitch
                self.vessel.auto_pilot.stopping_time = (0.7, 0.7, 0.7)
                aim_dir = [1, 0, 0] - (error_dir * [0, 10, 10])


            aim_dir = self.normalize(aim_dir)
            self.vessel.auto_pilot.target_direction = self.space_center.transform_direction(aim_dir, self.surface_ref, self.body_ref)


            # Throttle Control
            if alt < 8000 and not self.final_burn:
                if vert_speed > 0:
                    self.vessel.control.throttle = 0
                else:
                    if alt <= self.hover_altitude:
                        self.final_burn = True
                        self.vessel.gear = True
                        continue
                    elif time_fall <= self.gears_delay:
                        self.vessel.control.gear = True

                    a_net = max(0.1, a_eng - self.ag)

                    target_speed = -sqrt(self.final_speed*self.final_speed + 2*a_net*(alt-self.hover_altitude))
                    delta_speed = mag_speed + target_speed

                    delta_h = alt - (vert_speed*vert_speed + 2*self.ag*alt - 2*self.final_speed*self.final_speed) / (2*a_eng)
                    t_to_burn = (vert_speed + sqrt(vert_speed*vert_speed + 2*self.ag*delta_h)) / self.ag

                    throttle = self.throttle_control(delta_speed, a_eng, pitch, 10)

                    self.vessel.control.throttle = throttle

                    if self.accelerating: # Tempo para ligar motor
                        print(f'DeltaSpeed: {delta_speed:.2f}')
                    else:
                        if t_to_burn < self.reorient_delay: self.accelerating = True
                        print(f'Ignição em: {t_to_burn:.2f}s')
                            

            # Check Land
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

            if self.draw:
                self.draw_prograde_dir.end = prograde_dir * self.draw_line_lenght
                self.draw_target_dir.end = target_dir * self.draw_line_lenght
                self.draw_aim_dir.end = aim_dir * self.draw_line_lenght
    


    def throttle_control(self, accel, aeng, pitch, threshold=1):
        if pitch > 0:
            return (self.ag + accel*threshold) / (aeng* sin(radians(pitch)))
        return 0
    
    def get_point_pos(self):
        point_pos = np.array(self.space_center.transform_position(self.point, self.body_ref, self.surface_ref))
        return point_pos

    def get_velocity(self):
        return np.array(self.space_center.transform_direction(self.velocity(), self.body_ref, self.surface_ref))

    def landing_prediction(self, time_fall, alt):
        vel = self.space_center.transform_direction(self.velocity(), self.body_ref, self.surface_ref)
        return np.array([-alt, self.mu(0, vel[1], time_fall), self.mu(0, vel[2], time_fall)])

    # UTILS
    def time_fall(self, a, v, h):
        return (v + sqrt(v*v - 2*a*h)) / self.ag
    
    def mu(self, s0, v0, time):
        return s0 + v0 * time

    def altitude(self):
        return max(0, self.surface_altitude() + self.vessel.bounding_box(self.surface_ref)[0][0])
    
    def normalize(self, vector):
        return vector / np.linalg.norm(vector)


if __name__ == '__main__':
    LandingGuidance()