from vehicle import *


class PathBase:
    """
    single dimension dynamic
    """
    def get_state(self, s, v, a) -> State:
        """
        Given longitudinal info, get full state.
        :param s:
        :param v:
        :param a:
        :return:
        """
        pass

    def max_speed(self, vehicle_: Vehicle) -> float:
        """
        The uniform speed limit overestimation of the path.
        Speed higher than the limit can be out of consideration.
        :param vehicle_:
        :return:
        """
        pass


class StraightPath(PathBase):
    def __init__(self, origin: Vec2, direction: Vec2):
        super(StraightPath, self).__init__()
        assert direction.length() > kEps
        self.unit = direction.unit()
        self.origin = origin
        self.angle = self.unit.angle()

    def get_state(self, s, v, a) -> State:
        pos = self.origin + self.unit * s
        return [pos.x, pos.y, self.angle, 0.0, v, a]

    def max_speed(self, vehicle_: Vehicle) -> float:
        return vehicle_.max_lng_speed
