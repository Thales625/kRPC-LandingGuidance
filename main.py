import krpc
from time import sleep
from math import sqrt, cos, radians

from PyVecs import Vector3, angle_between_vectors

from utils import smooth_func
from trajectory          import Trajectory
from phase_controller    import PhaseControl
from landing_calculator  import LandingCalculator
from throttle_controller import ThrottleControl
from attitude_calculator import *

DEBUG_LINES = False

class LandingGuidance:
    def __init__(self):
        self.conn         = krpc.connect("ThaleX - Landing Guidance")
        self.space_center = self.conn.space_center
        self.vessel       = self.space_center.active_vessel
        self.control      = self.vessel.control
        self.auto_pilot   = self.vessel.auto_pilot
        self.body         = self.vessel.orbit.body
        self.drawing      = self.conn.drawing

        self.vessel_ref  = self.vessel.reference_frame
        self.body_ref    = self.body.reference_frame
        self.surface_ref = self.vessel.surface_reference_frame
        self.hybrid_ref  = self.space_center.ReferenceFrame.create_hybrid(position=self.body_ref, rotation=self.surface_ref)

        # Streams
        self.stream_mass             = self.conn.add_stream(getattr, self.vessel,       "mass")
        self.stream_av_thrust        = self.conn.add_stream(getattr, self.vessel,       "available_thrust")
        self.stream_situation        = self.conn.add_stream(getattr, self.vessel,       "situation")
        self.stream_ut               = self.conn.add_stream(getattr, self.space_center, "ut")
        self.stream_vel              = self.conn.add_stream(self.vessel.velocity,     self.hybrid_ref)
        self.stream_position_body    = self.conn.add_stream(self.vessel.position,     self.body_ref)
        self.stream_bbox             = self.conn.add_stream(self.vessel.bounding_box, self.surface_ref)
        self.stream_surface_altitude = self.conn.add_stream(getattr, self.vessel.flight(self.body_ref), "surface_altitude")

        # ===================
        
        # Target position

        # Barge
        #lat = -0.04933661420340462 
        #lon = -74.61837250015034  

        # VAB TOP
        #lat = -0.09679294489551704
        #lon = -74.61739078306573

        # BUILD TOP
        lat = -0.0925517713342951
        lon = -74.6631027997912

        # RUNWAY
        #lat = -0.04933661420340462
        #lon = -74.61837250015034

        self.target_pos_body = Vector3(self.body.surface_position(lat, lon, self.body_ref))
        del lat, lon

        '''
        self.target_pos_body = Vector3(self.space_center.target_vessel.position(self.body_ref))

        if self.space_center.target_vessel is None:
            print("Select a target!")
            self.conn.close()
            exit()
        '''

        # Params
        self.max_twr = 5 if self.body.has_atmosphere else 10
        self.thrust_threshold = 1 if self.body.has_atmosphere else 0.95

        self.final_speed    = -2
        self.final_altitude = 10
        self.max_hor_speed = 8
        self.downrange_target = 1

        # Drag params
        self.drag_area = 1.2**2 * 3.14
        self.cd = 1.2

        # Consts
        self.r_b = self.body.equatorial_radius
        self.a_g = self.body.surface_gravity
        self.landed_situation = self.vessel.situation.landed
        self.splashed_situation = self.vessel.situation.splashed
        self.k = self.drag_area * self.cd

        self.const_g_2   = self.a_g * 2
        self.const_g_inv = 1 / self.a_g

        self.vf_2 = self.final_speed*abs(self.final_speed)

        # Trajectory
        self.trajectory = Trajectory(True)

        # Phase Controller
        self.phase_controller = PhaseControl(
            [
                (self.waiting,         self.waiting_transition),
                (self.boost_back_burn, self.boost_back_burn_transition),
                (self.coasting,        self.coasting_transition),
                (self.landing_burn,    self.landing_burn_transition),
                (self.hovering,        None),
                (self.descent,         None),
                (self.touch_down,      None)
            ])

        # Calculator
        self.land_calculator = LandingCalculator(self.final_speed, self.body)

        # Throttle controller
        self.throttle_controller = ThrottleControl(self.stream_ut)
        self.throttle_controller.pid.adjust_pid(5, 15, 0)
        
        # Thrust correction
        self.thrust_correction = abs(min(1, sum([eng.available_thrust * eng.part.direction(self.vessel_ref)[1] for eng in self.vessel.parts.engines if (eng.active and eng.has_fuel)]) / self.stream_av_thrust()))

        # Initializing
        self.control.throttle = 0
        self.control.rcs = True

        self.auto_pilot.reference_frame   = self.body_ref
        self.auto_pilot.stopping_time     = (0.5, 0.5, 0.5)
        self.auto_pilot.deceleration_time = (5, 5, 5)
        self.auto_pilot.engage()

        self.trajectory.start(self.cd * self.drag_area)
        self.trajectory.step_size = 1
        self.trajectory.timeout = 0

        # Variables
        self.is_throttling = False
        self._downrange_error = 0
        self._alt = 0
        self._vel = Vector3()
        self._target_pos_surf = Vector3()
        self._land_pos_body   = Vector3()
        self._land_pos_surf   = Vector3()
        self._out_range_landing = False
        self.alt_func = lambda: self.stream_bbox()[0][0] - self._target_pos_surf.x

        # Inputs
        self._throttle = 0
        self._target_dir = (1, 0, 0)

        # DEBUG
        if DEBUG_LINES:
            self.line = self.drawing.add_line((0, 0, 0), (0, 0, 0), self.body_ref)
            self.line.color = (1, 0, 0)
            self.line.thickness = 5

            self.line_1 = self.drawing.add_line((0, 0, 0), (0, 0, 0), self.surface_ref)
            self.line_2 = self.drawing.add_line((0, 0, 0), (0, 0, 0), self.surface_ref)
            self.line_3 = self.drawing.add_line((0, 0, 0), (0, 0, 0), self.surface_ref)
            self.line_1.color = (0, 1, 0)
            self.line_2.color = (0, 0, 1)
            self.line_3.color = (1, 0, 1)

        # Main loop
        while True:
            self._vel = Vector3(self.stream_vel())

            self._target_pos_surf = Vector3(self.space_center.transform_position(self.target_pos_body, self.body_ref, self.surface_ref))

            self._alt = self.alt_func()

            self._land_pos_body = self.trajectory.land_pos
            self._land_pos_surf = Vector3(self.space_center.transform_position(self._land_pos_body, self.body_ref, self.surface_ref))

            self._downrange_error = self.r_b * angle_between_vectors(self._land_pos_body, self.target_pos_body)

            self.phase_controller.loop()

            self.control.throttle = self._throttle
            self.auto_pilot.target_direction = self.space_center.transform_direction(self._target_dir, self.surface_ref, self.body_ref)

            # Line
            if DEBUG_LINES:
                self.line.start = self._land_pos_body
                self.line.end   = self._land_pos_body * 2

                self.line_1.end = self._land_pos_surf
                self.line_2.end = self._target_pos_surf
                self.line_3.end = Vector3(self._target_dir).normalize() * 30

            sleep(0.05)
        
    # PHASES
    def waiting(self):
        if self.trajectory.CALCULATED or self._alt < self.body.atmosphere_depth * 0.7:
            self.phase_controller.next_phase()
            return

        sleep(0.5)

    def waiting_transition(self):
        self._boostback_last_downrange = self._downrange_error
        self._boostback_throttling = False

    def boost_back_burn(self):
        # THROTTLE CONTROL
        aim_error = sqrt(self.auto_pilot.pitch_error**2 + self.auto_pilot.heading_error**2)
        self._throttle = cos(radians(2*aim_error)) * self._downrange_error * 0.0001

        if self._throttle > 0: self._boostback_throttling = True

        # AIM CONTROL
        self._target_dir = aim_boost_back(self._land_pos_surf, self._target_pos_surf, self._vel)

        print(f'Downrange error: {self._downrange_error:.2f}')

        # END CHECK
        if self._boostback_throttling and aim_error < 20 and self._downrange_error - self._boostback_last_downrange > 5:
            print("DEU RUIM")
            self.phase_controller.next_phase()
            return

        if self._downrange_error < 50 or self._alt < self.body.atmosphere_depth * 0.7:
            self.phase_controller.next_phase()
            return

        self._boostback_last_downrange = self._downrange_error

    def boost_back_burn_transition(self):
        print("TRANSITION BOOST-BACK-BURN")
        self._throttle = 0
        self.control.brakes = True
        self.trajectory.step_size = 0.5
        self.trajectory.timeout = 0.1
        del self._boostback_last_downrange, self._boostback_throttling

    def coasting(self):
        # END CHECK
        if self._alt < 15000:
            self.phase_controller.next_phase()
            return

        # AIM CONTROL
        self._target_dir = aim_coasting(self._land_pos_surf, self._target_pos_surf, self._vel)

        print(f"COASTING => Δx: {self._downrange_error:.2f}")

    def coasting_transition(self):
        print("TRANSITION COASTING:", self._downrange_error)
        thrust_multiplier = self.thrust_correction * self.thrust_threshold * max(0.8, 0.95 ** (len([eng for eng in self.vessel.parts.engines if eng.active]) - 1)) * min(self.max_twr * self.a_g / (self.stream_av_thrust() * self.thrust_correction/self.vessel.mass), 1)

        if self._downrange_error > 1000:
            self._out_range_landing = True
            self.alt_func = lambda: max(0, self.stream_surface_altitude() + self.stream_bbox()[0][0])
            self._alt = self.alt_func()

        burn_height = 0.5 * (Vector3(self.stream_vel()).magnitude()**2 + self.const_g_2*self._alt) / (self.stream_av_thrust() * self.thrust_correction / self.vessel.mass)

        self.land_calculator.calculate(min(self._alt, burn_height + 500), self.vessel.mass, self.k, lambda p: thrust_multiplier * self.vessel.available_thrust_at(p))

    def landing_burn(self):
        # END CHECK
        if self._alt <= self.final_altitude + 2 or self._vel.x > 0:
            self.phase_controller.next_phase()
            return

        # Get streams
        mass = self.stream_mass()
        av_thrust = self.stream_av_thrust()
        mag_speed = self._vel.magnitude()

        # THROTTLE CONTROL
        h = self._alt - self.final_altitude

        delta_speed = self.land_calculator.get_v_target(h) + mag_speed
        
        self._throttle = self.throttle_controller.pid_control(delta_speed, av_thrust * self.thrust_correction / mass)

        print(f"LANDING BURN => | H: {h:.2f} | Δx: {self._downrange_error:.2f} | Δv: {delta_speed:.2f} | {self.is_throttling}")

        # AIM CONTROL
        if self._out_range_landing:
            self._target_dir = -self._vel
        else:
            if self.is_throttling:
                self._target_dir = aim_throttling(self._land_pos_surf, self._target_pos_surf, self._vel)
            else:

                #if self.vessel.thrust / (mass*self.a_g) > 1: self.is_throttling = True
                thrust_lift = (av_thrust * self._throttle) / (0.5 * self.k * mag_speed**2)
                if thrust_lift > 5:
                    if self._downrange_error > 100: self._out_range_landing = True
                    self.is_throttling = True

                self._target_dir = aim_coasting(self._land_pos_surf, self._target_pos_surf, self._vel)

    def landing_burn_transition(self):
        print("TRANSITION LANDING BURN")
        self.control.gear = True

    def hovering(self):
        # END CHECK
        if self._out_range_landing or self._downrange_error > 50:
            self._out_range_landing = True
            self.phase_controller.next_phase()
            return
        if self._downrange_error < 5 and sqrt(self._vel.y**2 + self._vel.z**2) < 4: # hor speed < 4
            self.phase_controller.next_phase()
            return

        # THROTTLE CONTROL
        a_eng = self.stream_av_thrust() * self.thrust_correction / self.stream_mass()
        delta_h = self.final_altitude - self._alt
        
        if delta_h < 0:
            a_net = a_eng - self.a_g
            v_target = -a_net * sqrt(-delta_h/a_net)
        else:
            v_target = self.a_g * sqrt(delta_h/self.a_g)

        delta_speed = v_target - self._vel.x

        self._throttle = self.throttle_controller.linear_control(delta_speed, a_eng, self.a_g, 1)

        # AIM CONTROL
        self._target_dir = aim_hovering(self._land_pos_surf, self._target_pos_surf, self._vel)

        print(f"GOING TARGET => | H: {self._alt:.2f} | Δx: {self._downrange_error:.2f} | Δv: {delta_speed:.2f}")


    def descent(self):
        # END CHECK
        situation = self.stream_situation()
        if situation == self.landed_situation or situation == self.splashed_situation:
            self.phase_controller.next_phase()
            return

        # THROTTLE CONTROL
        a_eng = self.stream_av_thrust() * self.thrust_correction / self.stream_mass()
        delta_speed = self.final_speed - self._vel.x

        self._throttle = self.throttle_controller.linear_control(delta_speed, a_eng, self.a_g, 2)

        # AIM CONTROL
        if self._out_range_landing:
            self._target_dir = aim_descent_out(self._vel)
        else:
            self._target_dir = aim_descent_in(self._land_pos_surf, self._target_pos_surf, self._vel)

        print(f"DESCENT => | H: {self._alt:.2f} | Δx: {self._downrange_error:.2f} | Δv: {delta_speed:.2f} | {'out' if self._out_range_landing else 'in'} target")


    def touch_down(self):
        self.progressive_engine_cut()

        self.control.brakes = False
        print(f"TOUCH-DOWN!\n{self.vessel.name} has landed!")

        # phase controller.end()
        self.trajectory.end()
        self.conn.close()
        exit()

    def progressive_engine_cut(self):
        # Check if rcs thrust is greather than
        self.control.rcs = True
        if self.vessel.available_rcs_force[0][1] / (self.a_g * self.stream_mass()) > 0.9:
            self.control.rcs = False

        # Engine cut-off
        smooth_func(self.control.throttle, 0, lambda v: setattr(self.control, "throttle", v), 0.5)

if __name__ == "__main__":
    LandingGuidance()