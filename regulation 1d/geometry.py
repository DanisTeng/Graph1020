from typing import Union, List, Tuple, Optional
import math

kEps = 1e-9
kInf = float('inf')


class Angle:
    _two_pi = math.pi * 2

    @classmethod
    def normalize(cls, x: float):
        """
        :param x:
        :return: x in [-pi, pi)
        """
        return (x + math.pi) % cls._two_pi - math.pi


class AngleInterval:
    def __init__(self, one_end: float, the_other_end: float):
        """
        the span of the interval <= math.pi
        Order of one_end and the_other_end doesn't matter
        :param one_end: closed
        :param another_end: closed
        """
        one_2_other = Angle.normalize(the_other_end - one_end)
        self.center = Angle.normalize(one_end + 0.5 * one_2_other)
        self.half_span = abs(0.5 * one_2_other)

    def is_inside(self, angle: float):
        return abs(Angle.normalize(angle - self.center)) <= self.half_span

    def has_intersect(self, other: "AngleInterval"):
        """
        False if no intersect.
        :param other:
        :return:
        """
        center_dist = abs(Angle.normalize(other.center - self.center))
        return center_dist <= self.half_span + other.half_span


class Interval:
    def __init__(self, begin, end, begin_open=False, end_open=False):
        self.begin = begin
        self.end = end

        self.begin_open = begin_open or self.begin is -kInf
        self.end_open = end_open or self.end is kInf

    def mid(self):
        assert not self.is_void()
        return 0.5*(self.begin + self.end)

    def is_void(self):
        if self.begin < self.end:
            return False
        elif not self.begin_open and not self.end_open and self.begin == self.end:
            return False
        return True

    def __bool__(self):
        return not self.is_void()

    def is_inside(self, value):
        if self.is_void():
            return False

        left_inside = value > self.begin if self.begin_open else value >= self.begin
        right_inside = value < self.end if self.end_open else value <= self.end
        return left_inside and right_inside

    def value_to_the_left(self, value):
        """
        Whether the value is to the left of the interval
        :param value:
        :return:
        """
        assert not self.is_void()
        return value <= self.begin if self.begin_open else value < self.begin

    def value_to_the_right(self, value):
        """
        Whether the value is to the right of the interval
        :param value:
        :return:
        """
        assert not self.is_void()
        return value >= self.end if self.end_open else value > self.end

    @staticmethod
    def void():
        return Interval(1.0, -1.0)

    def intersect(self, other: "Interval"):
        if self.is_void() or other.is_void():
            return self.void()

        # Is self begin stronger criterion?
        use_self_begin = self.begin > other.begin or (self.begin == other.begin and self.begin_open)
        # Is self end stronger criterion?
        use_self_end = self.end < other.end or (self.end == other.end and self.end_open)

        return Interval(self.begin if use_self_begin else other.begin,
                        self.end if use_self_end else other.end,
                        self.begin_open if use_self_begin else other.begin_open,
                        self.end_open if use_self_end else other.end_open)


class PartitionBase:

    def get_interval(self, index: int) -> Interval:
        pass

    def overlapping_index_range(self, interval: Interval) -> Optional[Tuple[int, int]]:
        pass

    def get_index_from_value(self, value: float) -> Optional[int]:
        pass


class BoundingValueSingletonPartition(PartitionBase):
    def __init__(self, min_value, max_value, step):
        self.min_value = min_value
        self.max_value = max_value
        self.step = step

        # min value singleton.
        self.intervals = [Interval(self.min_value, self.min_value)]

        # uniform step size in the middle
        num = math.floor((max_value - min_value) / step) + 1
        splits = [step * i + min_value for i in range(num)]
        # Ensure last split is max value.
        if max_value > splits[-1]:
            splits.append(max_value)
        self.intervals += [Interval(splits[i], splits[i + 1], begin_open=True, end_open=(i == len(splits) - 2)) for i in
                           range(len(splits) - 1)]

        # max value singleton
        self.intervals.append(Interval(self.max_value, self.max_value))

        assert all([not interval.is_void() for interval in self.intervals])

    def __len__(self):
        return len(self.intervals)

    def get_interval(self, index: int) -> Interval:
        assert 0 <= index < len(self.intervals)
        return self.intervals[index]

    def overlapping_index_range(self, interval: Interval) -> Optional[Tuple[int, int]]:
        max_id = self.get_index_from_value(interval.end)
        if max_id is None:
            max_id = 0 if interval.end <= self.min_value else len(self.intervals) - 1

        min_id = self.get_index_from_value(interval.begin)
        if min_id is None:
            min_id = 0 if interval.begin <= self.min_value else len(self.intervals) - 1

        # reduce from max_id side
        while max_id >= 0 and not self.intervals[max_id].intersect(interval):
            max_id -= 1

        # reduce from min_id side
        while min_id < len(self.intervals) and not self.intervals[min_id].intersect(interval):
            min_id += 1

        if max_id < min_id:
            # no overlap
            return None
        return min_id, max_id + 1


    def is_value_inside_partition(self, value:float) -> bool:
        return self.min_value <= value <= self.max_value


    def get_index_from_value(self, value: float) -> Optional[int]:
        """
        :param value:
        :return: The id of interval containing the value. None only when value is out of range.
        """
        if not self.is_value_inside_partition(value):
            return None

        id_end = len(self.intervals)
        id_hint = math.floor((value - self.min_value) / self.step) + 1
        id_hint = min(id_end - 1, max(0, id_hint))
        id_res = id_hint

        while id_res + 1 < id_end and self.intervals[id_res].value_to_the_right(value):
            id_res += 1
        while id_res > 0 and self.intervals[id_res].value_to_the_left(value):
            id_res -= 1

        assert self.intervals[id_res].is_inside(value), "Has a bug when this assertion violated."
        return id_res

BVSPartition = BoundingValueSingletonPartition


class UniformPartition(PartitionBase):
    def __init__(self, offset: float, step: float):
        assert step > 0
        self.offset = offset
        self.step = step

    def get_interval(self, index: int) -> Interval:
        left = self.offset + self.step * index
        return Interval(left, left + self.step, end_open = True)

    def overlapping_index_range(self, interval: Interval) -> Optional[Tuple[int, int]]:
        if not interval:
            return None

        max_id = self.get_index_from_value(interval.end)
        min_id = self.get_index_from_value(interval.begin)
        assert max_id >= min_id

        # reduce from max_id side
        while max_id >= min_id and not self.get_interval(max_id).intersect(interval):
            max_id -= 1

        # reduce from min_id side
        while min_id <= max_id and not self.get_interval(min_id).intersect(interval):
            min_id += 1

        return min_id, max_id + 1


    def get_index_from_value(self, value: float) -> Optional[int]:
        return math.floor((value - self.offset) / self.step)


# vectors

class Vec2:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __add__(self, other: Union["Vec2", float]):
        other_type = type(other)
        if other_type is Vec2:
            return Vec2(self.x + other.x, self.y + other.y)
        elif other_type in (float, int):
            return Vec2(self.x + other, self.y + other)

    def __radd__(self, other: Union["Vec2", float]):
        return self.__add__(other)

    def __sub__(self, other: Union["Vec2", float]):
        other_type = type(other)
        if other_type is Vec2:
            return Vec2(self.x - other.x, self.y - other.y)
        elif other_type in (float, int):
            return Vec2(self.x - other, self.y - other)

    def __rsub__(self, other: Union["Vec2", float]):
        other_type = type(other)
        if other_type is Vec2:
            return Vec2(other.x - self.x, other.y - self.y)
        elif other_type in (float, int):
            return Vec2(other - self.x, other - self.y)

    def __neg__(self):
        return Vec2(-self.x, -self.y)

    def __mul__(self, other: float):
        return Vec2(self.x * other, self.y * other)

    def __rmul__(self, other: float):
        return Vec2(self.x * other, self.y * other)

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def cross(self, other):
        return self.x * other.y - self.y * other.x

    def length_sqr(self):
        return self.x ** 2 + self.y ** 2

    def length(self):
        return math.sqrt(self.length_sqr())

    def unit(self):
        l = self.length()
        assert l > 1e-9
        return self.__mul__(1. / l)

    def angle(self):
        l = self.length()
        assert l > 1e-9
        return math.atan2(self.y, self.x)

    def perp(self):
        return Vec2(-self.y, self.x)

    def __copy__(self):
        return Vec2(self.x, self.y)

    def copy(self):
        return self.__copy__()

    @staticmethod
    def form_unit(angle: float):
        return Vec2(math.cos(angle), math.sin(angle))


def _lerp(start, end, ratio):
    return start + (end - start) * ratio


class Seg2:
    def __init__(self, start: Vec2, end: Vec2, unit: Vec2):
        self.start = start.copy()
        self.end = end.copy()
        self.unit = None if unit is None else unit.copy()

    @staticmethod
    def from_start_end(start: Vec2, end: Vec2):
        s2e = end - start
        if s2e.length() > 1e-9:
            unit = s2e.unit()
        else:
            unit = None
        return Seg2(start, end, unit)

    def __copy__(self):
        return Seg2(self.start, self.end, self.unit)

    def copy(self):
        return self.__copy__()

    def clamped(self, y_min: float = -float('inf'), y_max: float = float('inf')):
        """
        Get the overlap of the segment and {(x,y)| y_min <= y <= y_max}
        :param y_min:
        :param y_max:
        :return: None if he segment does not overlap the given area.
        or the clamped segment
        """
        assert y_max >= y_min

        start_higher = self.start.y >= self.end.y
        top = self.start if start_higher else self.end
        bot = self.end if start_higher else self.start

        if bot.y > y_max or top.y < y_min:
            # out of scope
            return None
        y_span = top.y - bot.y
        if y_span < 1e-9:
            return self.copy()

        if top.y > y_max:
            new_top = Vec2(_lerp(top.x, bot.x, (top.y - y_max) / y_span), y_max)
        else:
            new_top = top

        if bot.y < y_min:
            new_bot = Vec2(_lerp(top.x, bot.x, (top.y - y_min) / y_span), y_min)
        else:
            new_bot = bot

        new_start = new_top if start_higher else new_bot
        new_end = new_bot if start_higher else new_top

        return Seg2(new_start, new_end, self.unit)

    def blocking_angle_interval(self, view_point: Vec2):
        """
        The result won't be correct if the view point is on the segment.
        please make sure the view point is away from the segment.
        :param view_point:
        :return:
        """
        start = self.start - view_point
        end = self.end - view_point

        return AngleInterval(math.atan2(start.y, start.x), math.atan2(end.y, end.x))

    def product_onto_unit(self, pt: Vec2):
        assert self.unit is not None, "Segment must have unit"
        return self.unit.cross(pt - self.start)

    def project_onto_unit(self, pt: Vec2):
        assert self.unit is not None, "Segment must have unit"
        return self.unit.dot(pt - self.start)

    def clamped_ratio_range(self, seg_left: "Seg2", closed=True):
        """
        The ratio of a point on self segment's line is defined by project_onto_unit.
        for all points on self segments' line, their ratio range varies from -inf to inf.
        This function get the clamped ratio range by closed_seg_left.ProductOntoUnit(pt) >= 0.
        :param seg_left:
        :return:
        """
        start_offset = seg_left.product_onto_unit(self.start)
        end_offset = seg_left.product_onto_unit(self.end)

        if abs(start_offset - end_offset) < kEps:
            if start_offset > 0 or (start_offset == 0 and closed):
                return Interval(-kInf, kInf)
            else:
                return Interval.void()

        ratio_of_intersect = (0.0 - start_offset) / (end_offset - start_offset)
        if start_offset > end_offset:
            return Interval(-kInf, ratio_of_intersect, True, closed)
        else:
            return Interval(ratio_of_intersect, kInf, closed, True)


class Box2:
    def __init__(self,
                 origin: Vec2,
                 length: float,
                 width: float,
                 forward: Vec2,
                 left: Vec2):
        # origin is the bottom center
        self.origin = origin.copy()
        self.length = length
        self.width = width
        self.forward = forward.copy()
        self.left = left.copy()

    @staticmethod
    def from_vec_width(start: Vec2, end: Vec2, width: float) -> "Box2":
        # origin is the bottom center
        origin = start
        s2e = (end - start)
        length = s2e.length()
        width = width
        forward = (end - start).unit()
        left = forward.perp()

        return Box2(origin, length, width, forward, left)

    def four_corners(self)-> Tuple[Vec2,Vec2,Vec2,Vec2]:
        """
        front left, bot left, bot right, front right
        :return:
        """
        left_w = self.left * (self.width * 0.5)
        forward_h = self.length * self.forward
        return self.origin + forward_h + left_w, \
               self.origin + left_w, \
               self.origin - left_w, \
               self.origin + forward_h - left_w

    def four_segments(self):
        """
        front left->bot left,
        bot left->bot right,
        bot right-> front right,
        front right -> front left
        :return:
        """
        fl, bl, br, fr = self.four_corners()
        return Seg2(fl, bl, -self.forward), \
               Seg2(bl, br, -self.left), \
               Seg2(br, fr, self.forward), \
               Seg2(fr, fl, self.left)

    def to_ego_coordinate(self, p: Vec2):
        offset = p - self.origin
        return Vec2(x=offset.dot(self.forward),
                    y=offset.dot(self.left))

    def has_overlap_with_box(self, other: "Box2"):
        """
        :param other:
        :return:
        """
        pts = other.four_corners()
        pts = [self.to_ego_coordinate(pts[i]) for i in range(4)]
        other_segs = [Seg2.from_start_end(pts[i], pts[(i + 1) % 4]) for i in range(4)]

        hw = self.width * 0.5
        other_segs_clamped = [seg.clamped(y_max=hw, y_min=-hw) for seg in other_segs]

        x_min = float('inf')
        x_max = -float('inf')
        for seg in other_segs_clamped:
            if seg is None:
                continue
            x_min = min(seg.start.x, x_min)
            x_min = min(seg.end.x, x_min)
            x_max = max(seg.start.x, x_max)
            x_max = max(seg.end.x, x_max)

        return not (x_min > self.length or x_max < 0)

    def has_overlap_with_convex_region(self, convex_constraints: List[Seg2]):
        segs = self.four_segments()

        for seg in segs:
            ratio = Interval(0, 1)

            for constraint in convex_constraints:
                ratio = ratio.intersect(seg.clamped_ratio_range(constraint))
                if ratio.is_void():
                    break

            if not ratio.is_void():
                return True

        return False
