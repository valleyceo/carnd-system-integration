from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# Controller Class
class Controller(object):
    def __init__(self, *args, **kwargs):
        self.yaw_controller = YawController(kwargs['wheel_base'], 
    										kwargs['steer_ratio'], 
    										kwargs['min_speed'], 
    										kwargs['max_lat_accel'], 
    										kwargs['max_steer_angle'])

        # init controller
        self.accel_limit = kwargs['accel_limit']
        self.decel_limit = kwargs['decel_limit']

        # throttle_PID
        #throttle_PID = PID(1, 1, 1, self.accel_limit, self.decel_limit)
        
        self.lpf = LowPassFilter(0.1, math.pi/6)

        # steering PID
        #steer_PID = PID(0.15, 3, 0.0, 1, -1)

    def control(self, linear_velocity, angular_velocity, current_velocity):

        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if (current_velocity > 15): #15m/s -> 33.5 mph
            throttle = 0.
        else:
            throttle = 1.

        brake = 0.

        # get next steer value
    	steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)
    	steer_out = self.lpf.filt(steer)
        return throttle, brake, steer_out