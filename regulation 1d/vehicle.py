from typing import List, Tuple
from geometry import *
import math

# x, y, theta, kappa, v, a
State = List[float]
X_ID = 0
Y_ID = 1
THETA_ID = 2
KAPPA_ID = 3
V_ID = 4
A_ID = 5


class Vehicle:
    """
    Pure struct specifying vehicle property
    """

    def __init__(self):
        self.type = "no_type"
        self.width = 2.0
        self.length = 5.0
        self.rac_to_front = 4.0
        self.wheel_base = 3.0

        self.max_abs_lat_acc = 3.0  # 1.5 comf
        self.max_abs_lat_jerk = 5.0  # 2.5 comf
        self.max_lng_jerk = 5.0  # 3.0 comf
        self.min_lng_jerk = -8.0  # -4.0 comf
        self.max_lng_acc = 2.0  # 1.0 comf
        self.min_lng_acc = -6.0  # -3.0 comf
        self.min_lng_speed = 0.0  #
        self.max_lng_speed = 22.22  # 80 kph

        self.driver_left_offset = 0.5  # rac
        self.driver_front_offset = 2.0  # rac

    def __eq__(self, other: "Vehicle"):
        if not isinstance(other, Vehicle):
            return False

        return all([
            self.type == other.type,
            self.width == other.width,
            self.length == other.length,
            self.rac_to_front == other.rac_to_front,
            self.wheel_base == other.wheel_base,

            self.max_abs_lat_acc == other.max_abs_lat_acc,
            self.max_abs_lat_jerk == other.max_abs_lat_jerk,
            self.max_lng_jerk == other.max_lng_jerk,
            self.min_lng_jerk == other.min_lng_jerk,
            self.max_lng_acc == other.max_lng_acc,
            self.min_lng_acc == other.min_lng_acc,
            self.min_lng_speed == other.min_lng_speed,
            self.max_lng_speed == other.max_lng_speed,
            self.driver_left_offset == other.driver_left_offset,
            self.driver_front_offset == other.driver_front_offset,
        ])

    def get_rac(self, s: State):
        return Vec2(s[X_ID], s[Y_ID])

    def get_tangent(self, s: State):
        return Vec2.form_unit(s[THETA_ID])

    def box(self, s: State):
        rac = self.get_rac(s)
        tangent = self.get_tangent(s)
        front = rac + tangent * self.rac_to_front
        back = front - tangent * self.length
        return Box2.from_vec_width(back, front, self.width)
