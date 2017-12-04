from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

####################
# Controller Class #
####################
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
        
        # low pass filter
        sampling_time = 0.2
        self.lpf = LowPassFilter(self.accel_limit, sampling_time)

        # steering PID
        #steer_PID = PID(0.15, 3, 0.0, 1, -1)

    # compute next control command
    def control(self, command_v, command_w, current_v, current_w):

        # implement throttle
        if (current_v > 5): #15m/s -> 33.5 mph
            throttle = 0.
        else:
            throttle = .3

        # implement brake
        # if command angle is above or below .2, apply brake
        if (abs(command_w) > .2) or (command_v < .1):
            brake = 0.2
            throttle = 0
            steer_out = 0.
        else:
            brake = 0.
            # get steer value
            steer = self.yaw_controller.get_steering(command_v, command_w, current_v)
            steer_out = self.lpf.filt(steer)

        return throttle, brake, steer_out