import krpc
from time import sleep
from math import sqrt

from PyVecs import Vector3, Vector2

from PhaseController import PhaseController

class LandingGuidance:
    def __init__(self):
        self.conn = krpc.connect("LandingGuidance")
        self.space_center = self.conn.space_center
        self.vessel = self.space_center.active_vessel
        self.control = self.vessel.control
        self.auto_pilot = self.vessel.auto_pilot
        self.body = self.vessel.orbit.body

        self.body_ref = self.body.reference_frame
        self.surface_ref = self.vessel.surface_reference_frame
        self.hybrid_ref = self.space_center.ReferenceFrame.create_hybrid(position=self.body_ref, rotation=self.surface_ref)

        self.flight_body = self.vessel.flight(self.body_ref)
        self.flight_surface = self.vessel.flight(self.surface_ref)
        self.flight_hybrid = self.vessel.flight(self.hybrid_ref)

        # Check Target
        self.target = self.space_center.target_vessel
        if self.target is None: raise Exception("Select a target")

        # Streams
        self.stream_mass = self.conn.add_stream(getattr, self.vessel, "mass")
        self.stream_av_thrust = self.conn.add_stream(getattr, self.vessel, "available_thrust")
        self.stream_vel = self.conn.add_stream(getattr, self.flight_hybrid, "velocity")
        self.stream_position_body = self.conn.add_stream(self.vessel.position, self.body_ref)
        self.stream_surface_altitude = self.conn.add_stream(getattr, self.flight_body, "surface_altitude")
        self.stream_bbox = self.conn.add_stream(self.vessel.bounding_box, self.surface_ref)
        self.stream_situation = self.conn.add_stream(getattr, self.vessel, "situation")

        # ===================

        # Phase Controller
        self.phase_controller = PhaseController([(self.coasting, None), (self.landing_burn, None), (self.going_target, self.going_target_transition), (self.descent, self.descent_transition), (self.finish, None)])

        # Params
        self.max_twr = 4
        self.eng_threshold = .9
        self.final_speed = -2
        self.final_altitude = 20
        
        self.target_radius = 5
        self.reorient_delay = 1

        # Consts
        self.a_g = self.body.surface_gravity
        self.landed_situation = self.vessel.situation.landed
        self.splashed_situation = self.vessel.situation.splashed
        self.vf_2 = self.final_speed*abs(self.final_speed)

        # Vars
        #self.point = Vector3(self.target.position(self.body_ref)) # Target

        # VAB TOP
        #lat = -0.09679294489551704
        #lon = -74.61739078306573

        # RUNWAY
        lat = -0.04933661420340462
        lon = -74.61837250015034

        self.point = Vector3(self.body.surface_position(lat, lon, self.body_ref))

        # Initializing
        self.control.throttle = 0
        self.control.brakes = True
        self.control.rcs = True
        self.auto_pilot.engage()
        self.auto_pilot.reference_frame = self.body_ref
        self.auto_pilot.stopping_time = (0.5, 0.5, 0.5)
        self.auto_pilot.deceleration_time = (5, 5, 5)
        self.auto_pilot.target_roll = -90

        # Waiting
        while self.stream_vel()[0] > 0:
            self.auto_pilot.target_direction = self.space_center.transform_direction((1, 0, 0), self.surface_ref, self.body_ref)
            sleep(0.5)

        # Variables
        self._vel = Vector3()
        self._mass = 0
        self._aim_dir = Vector3()
        self._throttle = 0
        self._mag_speed = 0
        self._a_eng = 0
        self._a_eng_l = 0
        self._point_pos = Vector3()
        self._alt = 0
        self._land_predict_dir = Vector3()

        while True:
            self._vel = Vector3(self.stream_vel())
            self._mass = self.stream_mass()
            av_thrust = self.stream_av_thrust()

            self._aim_dir = Vector3(1, 0, 0)
            self._throttle = 0

            self._mag_speed = self._vel.magnitude()
            
            self._a_eng = av_thrust / self._mass
            self._a_eng_l = self._a_eng * self.eng_threshold * min(self.max_twr * self.a_g / self._a_eng, 1)
    
            self._point_pos = Vector3(self.space_center.transform_position(self.point, self.body_ref, self.surface_ref))
            self._alt = self.stream_bbox()[0][0] - self._point_pos.x
            if self._alt < 0: self._alt = 0

            t_free_fall = (self._vel.x + sqrt(2*self.a_g*self._alt + self._vel.x**2)) / self.a_g
            landing_prediction = Vector3(-self._alt, self._vel.y*t_free_fall, self._vel.z*t_free_fall)

            self._land_predict_dir = landing_prediction.normalize()

            self.phase_controller.loop()

            self._aim_dir = self._aim_dir.normalize()

            self.auto_pilot.target_direction = self.space_center.transform_direction(self._aim_dir, self.surface_ref, self.body_ref)
            self.control.throttle = self._throttle

            sleep(0.05)


    def coasting(self):
        print("COASTING")
        # AIM CONTROL
        error = self._land_predict_dir - (self._point_pos + Vector3(0, .1*self._point_pos.y, .1*self._point_pos.z)).normalize()
        self._aim_dir = Vector3(.1, error.y, error.z)

        v_2 = self._vel.x*abs(self._vel.x)
        burn_altitude = abs((self._mag_speed**2 + self.vf_2 + 2*self.a_g*self._alt) / (2 * self._a_eng_l))
        t_to_burn = (self._vel.x + sqrt(max(0, 2*self.a_g*(self._alt - burn_altitude) - v_2))) / self.a_g
        if t_to_burn < self.reorient_delay:
            self.phase_controller.next_phase()

    def landing_burn(self):
        print("LANDING-BURN")
        # THROTTLE CONTROL
        a_net = self._a_eng_l - self.a_g
        target_speed = sqrt(2*a_net*abs(self._alt-self.final_altitude))
        delta_speed = self._mag_speed - target_speed
        self._throttle = 0 if self._vel.x > self.final_speed else ((delta_speed*5 + self.a_g) / self._a_eng)

        # AIM CONTROL
        land_target_dir = (self._point_pos - Vector3(0, self._vel.y*5, self._vel.z*5)).normalize()
        error = self._land_predict_dir - land_target_dir
        self._aim_dir = Vector3(2, -error.y, -error.z)

        if self._alt <= self.final_altitude:
            self.phase_controller.next_phase()

    def going_target_transition(self):
        print("GOING-TARGET TRANSITION")
        #self.point = Vector3(self.target.position(self.body_ref))
        self.control.brakes = False

    def going_target(self):
        print("GOING-TARGET")
        # THROTTLE CONTROL
        delta_h = self.final_altitude - self._alt
        
        if delta_h < 0:
            a_net = self._a_eng - self.a_g
            v_target = -a_net * sqrt(-delta_h/a_net)
        else:
            v_target = self.a_g * sqrt(delta_h/self.a_g)

        delta_v = v_target - self._vel.x
        self._throttle = (delta_v + self.a_g) / self._a_eng

        # AIM CONTROL
        land_target_dir = (self._point_pos - Vector3(0, self._vel.y*5, self._vel.z*5)).normalize()
        error = self._land_predict_dir - land_target_dir
        self._aim_dir = Vector3(4, -error.y, -error.z)

        if Vector2(self._vel.y, self._vel.z).magnitude() <= 5 and Vector2(self._point_pos.y, self._point_pos.z).magnitude() <= self.target_radius:
            self.phase_controller.next_phase()

    def descent_transition(self):
        print("DESCENT TRANSITION")
        self.control.gear = True

    def descent(self):
        print("DESCENT")
        # THROTTLE CONTROL
        delta_speed = self.final_speed - self._vel.x
        self._throttle = (delta_speed*2 + self.a_g) / self._a_eng

        # AIM CONTROL
        land_target_dir = (self._point_pos - Vector3(0, self._vel.y*5, self._vel.z*5)).normalize()
        error = self._land_predict_dir - land_target_dir
        self._aim_dir = Vector3(6, -error.y, -error.z)

        situation = self.stream_situation()
        if situation == self.landed_situation or situation == self.splashed_situation:
            self.phase_controller.next_phase()

    def finish(self):
        print("FINISH")
        self.progressive_engine_cut()
        self.sas_aim_up()

        self.control.rcs = True
        self.control.brakes = False
        print(f"{self.vessel.name} has landed!")
        self.conn.close()

        exit()


    def progressive_engine_cut(self):
        a_eng = self.stream_av_thrust() / self.stream_mass()

        throttle = min((self.a_g / a_eng) *.8, 1)
        total_time = 1
        dt = 0.1
        
        delta_throttle = (throttle * dt) / total_time

        while throttle > 0:
            throttle -= delta_throttle
            self.control.throttle = throttle
            sleep(dt)

        self.control.throttle = 0

    def sas_aim_up(self):
        self.auto_pilot.disengage()
        self.control.rcs = True
        self.control.sas = True
        sleep(0.1)
        try:
            self.control.sas_mode = self.control.sas_mode.radial
        except:
            self.control.sas = False
            self.auto_pilot.engage()
            self.auto_pilot.target_direction = self.space_center.transform_direction((1, 0, 0), self.surface_ref, self.body_ref)
            input("Press enter to finish program.")
            self.auto_pilot.disengage()

if __name__ == "__main__":
    LandingGuidance()
