from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

MIN_SPEED = 0.1

PID_KP = 0.3
PID_KI = 0.1
PID_KD = 0.
PID_MIN_THROTTLE = 0.  
PID_MAX_THROTTLE = 0.2 

LOWPASS_TAU = 0.5
LOWPASS_SAMPLE_TIME = 0.2


class Controller(object):
   
    def __init__(
        self,
        vehicle_mass,
        fuel_capacity,
        brake_deadband,
        decel_limit,
        accel_limit,
        wheel_radius,
        wheel_base,
        steer_ratio,
        max_lat_accel,
        max_steer_angle
    ):
        
        self.yaw_contoller = YawController(
            wheel_base,
            steer_ratio,
            MIN_SPEED,
            max_lat_accel,
            max_steer_angle
        )

        self.throttle_controller = PID(PID_KP, PID_KI, PID_KD, PID_MIN_THROTTLE, PID_MAX_THROTTLE)

        self.vel_lpf = LowPassFilter(LOWPASS_TAU, LOWPASS_SAMPLE_TIME)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(
        self,
        current_vel,
        dbw_enabled,
        linear_vel,
        angular_vel
    ):

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_contoller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = current_vel - linear_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)

        brake = 0 # brake is in units of torque -- N*m

        if linear_vel == 0. and current_vel < 0.1:
            
            throttle = 0.
            brake = 400.

        elif throttle < 0.1 and vel_error < 0:

            throttle = 0.
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius 

        return throttle, brake, steering