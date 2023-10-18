import krpc
from time import sleep
from math import sqrt

from Vector import Vector3, Vector2

from PhaseController import PhaseController

class LandingGuidance:
    def __init__(self, land_func=None):
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
        
        if land_func is None:
            self.land_func = lambda self: (
                self.progressive_engine_cut(),
                self.sas_aim_up(),
                self.finish())
        else:
            self.land_func = land_func

        # Phase Controller
        self.phase_controller = PhaseController([self.coasting, self.landing_burn, self.going_target, self.descent])

        # Params
        self.max_twr = 4
        self.eng_threshold = .9
        self.final_speed = -2
        self.final_altitude = 15
        self.target_speed = 10
        
        self.target_radius = 8
        self.reorient_delay = 2

        # Consts
        self.a_g = self.body.surface_gravity
        self.landed_situation = self.vessel.situation.landed
        self.splashed_situation = self.vessel.situation.splashed
        self.vf_2 = self.final_speed*abs(self.final_speed)

        # Vars
        self.phase = 0 # 0-coasting | 1-landing burn | 2-going target | 3-descent
        self.point = Vector3(self.target.position(self.body_ref))
        #self.point = Vector3(self.body.surface_position(-0.09679294489551704, -74.61739078306573, self.body_ref)) # KSC Landing Site

        # Initializing
        self.control.throttle = 0
        self.control.brakes = True
        self.control.rcs = True
        self.auto_pilot.engage()
        self.auto_pilot.reference_frame = self.body_ref
        self.auto_pilot.stopping_time = (0.8, 0.8, 0.8) #(0.5, 0.5, 0.5)
        self.auto_pilot.deceleration_time = (5, 5, 5)
        self.auto_pilot.target_roll = -90

        # Waiting
        while self.stream_vel()[0] > 0:
            self.auto_pilot.target_direction = self.space_center.transform_direction((1, 0, 0), self.surface_ref, self.body_ref)
            sleep(0.5)
        
        while self.stream_surface_altitude() > 15000:
            self.auto_pilot.target_direction = Vector3(self.flight_body.velocity) * -1
            sleep(0.5)

        # LINES
        line_aim_dir = self.conn.drawing.add_line((0, 0, 0), (0, 0, 0), self.surface_ref)
        line_aim_dir.color = (0, 0, 1)
        line_target_dir = self.conn.drawing.add_line((0, 0, 0), (0, 0, 0), self.surface_ref)
        line_target_dir.color = (1, 0, 0)

        while True:
            # Get Streams
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

          
            t_free_fall = (self._vel.x + sqrt(2*self.a_g*self._alt + self._vel.x**2)) / self.a_g
            landing_prediction = Vector3(-self._alt, self._vel.y*t_free_fall, self._vel.z*t_free_fall)
            self._dist = landing_prediction.magnitude()

            land_predict_dir = landing_prediction / self._dist
            land_target_dir = (self._point_pos - Vector3(0, self._vel.y*5, self._vel.z*5)).normalize()
            self._error = land_predict_dir - land_target_dir

            self.phase_controller.loop()

            self._aim_dir = self._aim_dir.normalize()

            self.auto_pilot.target_direction = self.space_center.transform_direction(self._aim_dir, self.surface_ref, self.body_ref)
            self.control.throttle = self._throttle

            line_aim_dir.end = self._aim_dir * 10
            line_target_dir.end = land_target_dir * 10

            sleep(0.05)


    def coasting(self):
        print("COASTING")
        v_2 = self._vel.x*abs(self._vel.x)
        burn_altitude = abs((self._mag_speed**2 + self.vf_2 + 2*self.a_g*self._alt) / (2 * self._a_eng_l))
        t_to_burn = (self._vel.x + sqrt(max(0, 2*self.a_g*(self._alt - burn_altitude) - v_2))) / self.a_g

        self._aim_dir = Vector3(2, self._error.y, self._error.z)

        if t_to_burn < self.reorient_delay:
            self.phase_controller.next_phase()

    def landing_burn(self):
        print("LANDING BURN")
        # THROTTLE CONTROL
        a_net = max(self._a_eng_l - self.a_g, .1)
        target_speed = sqrt(self.final_speed**2 + 2*a_net*abs(self._dist-self.final_altitude))
        delta_speed = self._mag_speed - target_speed
        self._throttle = (delta_speed*5 + self.a_g) / self._a_eng

        # AIM CONTROL
        """
        vel_dir = self._vel / self._mag_speed
        error = self._error
        error.x = abs(error.x)
        error = error.normalize()
        self._aim_dir = -vel_dir*2 - error
        """

        self._aim_dir = Vector3(5, -self._error.y, -self._error.z)

        if self._alt <= self.final_altitude:
            self.phase_controller.next_phase()

    def going_target(self):
        print("GOING TARGET")
        delta_h = self.final_altitude - self._alt
        v_target = sqrt(2*self.a_g*abs(delta_h)) * (delta_h / abs(delta_h))
        delta_v = v_target - self._vel.x
        self._throttle = (delta_v*2 + self.a_g) / self._a_eng

        self._aim_dir = Vector3(5, -self._error.y, -self._error.z)

        if Vector2(self._vel.y, self._vel.z).magnitude() <= 1 and Vector2(self._point_pos.y, self._point_pos.z).magnitude() <= 1:
            self.phase_controller.next_phase()

    def descent(self):
        print("DESCENT")
        self._aim_dir = Vector3(4, -self._error.y, -self._error.z)
        self.control.gear = True

        self.land_func()



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

    def finish(self):
        self.control.rcs = True
        self.control.brakes = False
        print(f"{self.vessel.name} has landed!")
        self.conn.close()

if __name__ == "__main__":
    LandingGuidance()