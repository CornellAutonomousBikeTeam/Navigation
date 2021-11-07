import math

VERY_SMALL = 1e-20  # used to avoid division by zero

class PointFollower:
    def __init__(self, yaw_adjust_speed=50):
        self.YAW_ADJUST_SPEED = yaw_adjust_speed

    def get_nav_command(self, robot_pos, robot_heading, follow_point):
        """Returns the amount to turn the handlebars"""
        reverse_point = get_reverse_point(robot_pos, follow_point, robot_heading)
        curvature = get_curvature(follow_point, robot_pos, reverse_point)
        sign = -math.sin(robot_heading - math.atan2(follow_point[1] - robot_pos[1], follow_point[0] - robot_pos[0]))
        # just shake up the yaw a bit if follow_point is right behind/ahead of self, only really necessary in simulations
        return math.copysign(self.YAW_ADJUST_SPEED * curvature, sign) if sign ** 2 > 1e-10 else 0.01

def get_reverse_point(robot_pos, follow_point, robot_heading):
    dx = follow_point[0] - robot_pos[0]
    dy = follow_point[1] - robot_pos[1]
    magnitude = math.sqrt(dx ** 2 + dy ** 2)
    follow_heading = math.atan2(dy, dx)
    # the reverse heading formula is simplified from the more-intuitive formula:
    # follow_heading + 2(pi/2 - (follow_heading-robot_heading))
    reverse_heading = math.pi + 2 * robot_heading - follow_heading
    reverse_x = robot_pos[0] + magnitude * math.cos(reverse_heading)
    reverse_y = robot_pos[1] + magnitude * math.sin(reverse_heading)
    return reverse_x, reverse_y


def get_curvature(p1, p2, p3):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x1_minus_x2 = x1 - x2
    if x1_minus_x2 == 0:
        x1_minus_x2 = VERY_SMALL
    k1 = 0.5 * (x1 ** 2 + y1 ** 2 - x2 ** 2 - y2 ** 2) / x1_minus_x2
    k2 = (y1 - y2) / x1_minus_x2
    b_denom = (x3 * k2 - y3 + y2 - x2 * k2)
    if b_denom == 0:
        b_denom = VERY_SMALL
    b = 0.5 * (x2 ** 2 - 2 * x2 * k1 + y2 ** 2 - x3 ** 2 + 2 * x3 * k1 - y3 ** 2) / b_denom
    a = k1 - k2 * b
    square_reciprocal = ((x1 - a) ** 2 + (y1 - b) ** 2)
    if square_reciprocal == 0:
        square_reciprocal = VERY_SMALL
    return 1/math.sqrt(square_reciprocal)
