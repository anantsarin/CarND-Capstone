import rospy

from ros.src.twist_controller.lowpass import LowPassFilter
from ros.src.twist_controller.pid import PID
from ros.src.twist_controller.yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']

        self.last_time = rospy.get_time()

        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']

        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 0.1, self.max_lat_accel,
                                            self.max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0
        mx = 0.2
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5  # 1/2pi*tau
        ts = 0.02  # sample time
        self.vel_lpf = LowPassFilter(tau.ts)

        # self.sampling_rate   = kwargs['sampling_rate']
        # self.max_braking_percentage= kwargs['max_braking_percentage']
        # self.max_throttle_percentage = kwargs['max_throttle_percentage']

    # def control(self, *args, **kwargs):
    def control(self, current_linear_vel, dbw_enabled, linear_vel, angular_vel):  # , current_lin_vel
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.steering_correction_pid.reset()
            return 0., 0., 0.
        current_linear_vel = self.vel_lpf.filt(current_linear_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_linear_vel)

        vel_error = linear_vel - current_linear_vel
        self.last_vel = current_linear_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake_ = 0

        if linear_vel == 0. and current_linear_vel < 0.1:
            throttle = 0
            brake_ = 400
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake_ = abs(decel) * self.vehicle_mass * self.wheel_radius  # torque n*m

        return throttle, brake_, steering
