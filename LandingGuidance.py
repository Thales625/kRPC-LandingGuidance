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
        self.surface_gravity = self.body.surface_gravity
        self.drawing = self.conn.drawing

        # Check Target
        self.target = self.space_center.target_vessel
        if False and self.target is None:
            print('Selecione um alvo!')
            exit()

        # Streams
        self.mass = self.conn.add_stream(getattr, self.vessel, "mass")
        self.velocity = self.conn.add_stream(getattr, self.flight, "velocity")
        self.vertical_speed = self.conn.add_stream(getattr, self.flight, "vertical_speed")
        self.horizontal_speed = self.conn.add_stream(getattr, self.flight, "horizontal_speed")
        self.mag_speed = self.conn.add_stream(getattr, self.flight, "speed")
        self.surface_altitude = self.conn.add_stream(getattr, self.flight, "surface_altitude")
        self.pitch = self.conn.add_stream(getattr, self.vessel.flight(self.surface_ref), "pitch")

        # Initializing
        self.vessel.control.throttle = 0
        self.vessel.control.brakes = True
        self.vessel.control.rcs = True
        self.vessel.auto_pilot.engage()
        self.vessel.auto_pilot.target_roll = -90
        self.vessel.auto_pilot.stopping_time = (0.5, 0.5, 0.5)
        self.vessel.auto_pilot.reference_frame = self.body_ref

        # Wait
        self.vessel.auto_pilot.target_direction = self.space_center.transform_direction((1, 0, 0), self.surface_ref, self.body_ref)

        while self.vertical_speed() > 0:# or self.altitude() > 8000: # WAIT
            pass

        # Params
        self.eng_threshold = 0.9
        self.final_speed = -2
        self.target_speed = 10
        self.hover_altitude = 15
        self.target_radius = 8
        self.final_burn = False
        self.final_approach = False
        self.accelerating = False
        self.reorient_delay = 2
        self.gears_delay = 4 /2
        self.point = np.array(self.target.position(self.body_ref))
        #self.point = np.array(self.body.surface_position(-0.09679294489551704, -74.61739078306573, self.body_ref)) # KSC Landing Site

        #Drawing
        self.draw = False
        if self.draw:
            self.draw_target_dir = self.drawing.add_direction((0, 0, 0), self.surface_ref)
            self.draw_prograde_dir = self.drawing.add_direction((0, 0, 0), self.surface_ref)
            self.draw_aim_dir = self.drawing.add_direction((0, 0, 0), self.surface_ref)
            self.draw_target_dir.color = (0, 0, 255) # Azul
            self.draw_prograde_dir.color = (255, 0, 0) # Vermelho
            self.draw_aim_dir.color = (0, 255, 0) # Verde
            self.draw_line_lenght = 10

        # Thrust de acordo com ângulo de montagem do motor
        self.thrust = 0
        for engine in self.vessel.parts.engines:
            if engine.active:
                self.thrust += engine.available_thrust * engine.part.direction(self.vessel.reference_frame)[1]

        # Simulação
        self.simulation = Simulation(self.rocket_radius(), self.mass(), self.thrust*self.eng_threshold, self.altitude(), self.final_speed, self.body)

        while True:
            sleep(0.01)
            
            vert_speed = self.vertical_speed()
            pitch = self.pitch()
            point_pos = self.get_point_pos()
            alt = -point_pos[0] - abs(self.vessel.bounding_box(self.surface_ref)[0][0])
            time_fall = self.time_fall(-.5 * self.surface_gravity, vert_speed, alt)

            # Aim to point
            point_dist_hor = np.linalg.norm(point_pos[1:])
            point_vel_hor = self.horizontal_speed()
            target_dir = self.normalize(point_pos) # Direção do alvo
            prograde_dir = self.normalize(self.landing_prediction(time_fall, alt))
            error_dir = target_dir - prograde_dir
            

            if self.accelerating:
                self.vessel.auto_pilot.stopping_time = (1, 1, 1)
                aim_dir = [3, 0, 0] + error_dir*[0, 1, 1]

                if self.final_burn:
                    target_accel = ((self.hover_altitude - alt)/4 - vert_speed) # Controla altitude
                    if point_dist_hor <= 5 and alt <= self.hover_altitude+5:
                        self.final_approach = True

                    if self.final_approach:
                        target_accel = self.final_speed - vert_speed
                        #aim_dir = [3, 0, 0] + error_dir*[0, 1, 1]
                        if point_dist_hor >= self.target_radius: # Sair do raio do alvo
                            self.final_approach = False
                    
                    self.vessel.control.throttle = self.throttle_control(target_accel, pitch, 2)

                if not self.final_approach and np.linalg.norm(prograde_dir[1:] - target_dir[1:]) < 0.5: # Goto target
                        target_speed = min(point_dist_hor/8, self.target_speed)
                        speed_error = max(min(point_vel_hor - target_speed, 5), -5) / 4
                        aim_dir -= prograde_dir*[0, 1, 1] * speed_error 

                    
            else: # Atmosfera <- Limitar pitch
                self.vessel.auto_pilot.stopping_time = (0.7, 0.7, 0.7)
                aim_dir = [1, 0, 0] - (error_dir * [0, 10, 10])


            aim_dir = self.normalize(aim_dir)
            self.vessel.auto_pilot.target_direction = self.space_center.transform_direction(aim_dir, self.surface_ref, self.body_ref)
   


            if alt < 8000:
                if not self.final_burn:
                    if vert_speed > 0:
                        self.vessel.control.throttle = 0
                    else:
                        if alt <= self.hover_altitude+5:
                            self.final_burn = True
                            self.vessel.gear = True
                        elif time_fall <= self.gears_delay:
                            self.vessel.control.gear = True

                        target_speed = self.simulation.get_speed(alt-self.hover_altitude)
                        delta_speed = target_speed + self.mag_speed()

                        self.vessel.control.throttle = self.throttle_control(delta_speed, pitch, 10)

                        if not self.accelerating:
                            delay_size = abs(vert_speed*self.reorient_delay - (self.surface_gravity*(self.reorient_delay**2))/2)
                            if self.simulation.get_speed(alt-self.hover_altitude-delay_size) + self.mag_speed() > 0:
                                self.accelerating = True
            
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
    


    def throttle_control(self, accel, pitch, threshold=1):
        if pitch > 0:
            aeng = self.thrust / self.mass()
            return (self.surface_gravity + accel*threshold) / (aeng * sin(radians(pitch)))
        return 0
    
    def get_point_pos(self):
        point_pos = np.array(self.space_center.transform_position(self.point, self.body_ref, self.surface_ref))
        #point_pos[0] -= self.vessel.bounding_box(self.surface_ref)[0][0]
        return point_pos

    def get_velocity(self):
        return np.array(self.space_center.transform_direction(self.velocity(), self.body_ref, self.surface_ref))

    def landing_prediction(self, time_fall, alt):
        vel = self.space_center.transform_direction(self.velocity(), self.body_ref, self.surface_ref)
        return np.array([-alt, self.mu(0, vel[1], time_fall), self.mu(0, vel[2], time_fall)])

    # UTILS
    def rocket_radius(self):
        size = 0.5
        for fuel in self.vessel.resources.with_resource('LiquidFuel'):
            size = max(0.5, fuel.part.bounding_box(self.vessel.reference_frame)[1][0] - fuel.part.bounding_box(self.vessel.reference_frame)[0][0])
        return abs(size)/2

    def time_fall(self, a, v, h):
        try:
            d = sqrt((v * v) - 4 * a * h)
            result_1 = (-v + d) / (2 * a)
            result_2 = (-v - d) / (2 * a)
            return max(result_1, result_2)
        except:
            return 0
    
    def mu(self, s0, v0, time):
        return s0 + v0 * time

    def altitude(self):
        return max(0, self.surface_altitude() + self.vessel.bounding_box(self.surface_ref)[0][0])
    
    def normalize(self, vector):
        return vector / np.linalg.norm(vector)


if __name__ == '__main__':
    LandingGuidance()