from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, dbw_enabled, accel_limit, decel_limit, current_velocity):
        # init controller
        self.dbw_enabled = dbw_enabled
		self.accel_limit = accel_limit
		self.decel_limit = decel_limit
		self.current_velocity = current_velocity

		# throttle_PID
		throttle_PID = PID(1, 1, 1, accel_limit, decel_limit)

		# steering PID
		steer_PID = PID(0.15, 3, 0.0, 1, -1)

    def control(self, twist_cmd, current_velocity, dbw_enabled):

        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer


		if (current_velocity > 30) {
			throttle = 0.;
		} else {
			throttle = 0.3;
		}

		brake = 0.
		steer = 0.


		cte = 0
		sample_time = 0

		# get next steer value
		#steer = self.steer_PID(cte, sample_time)

        return throttle, brake, steer