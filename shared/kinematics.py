"""Shared steering calibration for odometry, planning, and reverse checking."""

def steering_angle(command,cfg):
    return command*cfg['steering_gain']


def travel_speed(command,cfg):
    # The vendor reduces inner-wheel power linearly with steering angle.
    # Scale the center-speed reduction to the measured short turning steps.
    return cfg['speed_cm_s']*(1-(1-cfg.get('turn_speed_ratio',1.))*abs(command)/30.)
