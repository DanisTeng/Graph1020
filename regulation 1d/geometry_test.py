from geometry import *
from utils import  random_float
import random

debug_plot = False
if debug_plot:
    from matplotlib import pyplot as plt

def test_vec2():
    a = Vec2(1, 2)
    b = Vec2(3, 4)
    c = 5

    # sub
    r = a - b
    assert r.x == -2
    assert r.y == -2

    r = a - c
    assert r.x == -4
    assert r.y == -3

    # rsub
    r = b - a
    assert r.x == 2
    assert r.y == 2

    r = c - a
    assert r.x == 4
    assert r.y == 3

    # add
    r = a + b
    assert r.x == 4
    assert r.y == 6

    r = a + c
    assert r.x == 6
    assert r.y == 7

    # radd
    r = b + a
    assert r.x == 4
    assert r.y == 6

    r = c + a
    assert r.x == 6
    assert r.y == 7

    # mul
    r = a * c
    assert r.x == 5
    assert r.y == 10

    # rmul
    r = c * a
    assert r.x == 5
    assert r.y == 10

    # neg
    r = -a
    assert r.x == -1
    assert r.y == -2

    # dot
    r = a.dot(b)
    assert r == 11

    # cross
    r = a.cross(b)
    assert r == -2

    # length
    assert b.length() == 5

    # unit
    r = a.unit()
    r.x = 1./(5**0.5)
    r.y = 2./(5**0.5)

    # perp
    r = a.perp()
    assert r.x == -2
    assert r.y == 1

def test_seg2():
    a = Seg2.from_start_end(Vec2(0,0), Vec2(6,8))

    # Clamp by y_min y_max
    r = a.clamped(y_min = 1.0)
    assert r.start.y == 1.
    assert r.start.x == 0.75
    assert r.end.x == 6
    assert r.end.y == 8

    # Clamp by y_min y_max
    r = a.clamped(y_max = 4.0)
    assert r.start.y == 0
    assert r.start.x == 0
    assert r.end.x == 3
    assert r.end.y == 4

    # Clamp by y_min y_max
    r = a.clamped(y_min = 1.0, y_max = 4.0)
    assert r.start.y == 1.
    assert r.start.x == 0.75
    assert r.end.x == 3
    assert r.end.y == 4

    # Clamp by y_min y_max
    r = a.clamped(y_min=9.0, y_max=10.0)
    assert r is None

    # ratio clamp by half plane
    a = Seg2.from_start_end(Vec2(0,0), Vec2(1,1))
    b = Seg2.from_start_end(Vec2(0,1), Vec2(1,0))

    res = a.clamped_ratio_range(b)
    assert res.begin == 0.5
    assert res.end is kInf

def test_interval():
    # intersect
    i0 = Interval(0, 1)
    i1 = Interval(0.5, 2)
    res = i0.intersect(i1)
    assert res.begin == 0.5
    assert res.end == 1
    assert not res.begin_open
    assert not res.end_open

    # intersect with open edge
    i0 = Interval(0, 1, True)
    i1 = Interval(0, 2)
    res = i0.intersect(i1)
    assert res.begin == 0
    assert res.end == 1
    assert res.begin_open
    assert not res.end_open

    # intersect with open edge
    i0 = Interval(0, 1)
    i1 = Interval(1, 2, True)
    res = i0.intersect(i1)
    assert res.is_void()

    # intersect with open edge
    i0 = Interval(0, 1)
    i1 = Interval(1, 2)
    res = i0.intersect(i1)
    assert not res.is_void()


def test_box_intersection():


    def random_vec():
        return Vec2(random.random() * 10.0, random.random()*10.0)

    def random_box():
        return Box2.from_vec_width(random_vec(),
                            random_vec(),
                            random.random()* 4.0)

    def has_collision_ground_truth(box1:Box2, box2:Box2):
        return box2.has_overlap_with_convex_region(box1.four_segments())

    def test_random_case():
        a = random_box()
        b = random_box()
        gt = has_collision_ground_truth(a, b)
        assert a.has_overlap_with_box(b) == gt

        if debug_plot:
            plt.figure(0)
            ac = a.four_corners()
            bc = b. four_corners()
            acx = [ac[i%4].x for i in range(5)]
            acy = [ac[i%4].y for i in range(5)]
            bcx = [bc[i % 4].x for i in range(5)]
            bcy = [bc[i % 4].y for i in range(5)]
            plt.plot(acx, acy, color = "red"if gt else "blue")
            plt.plot(bcx,bcy, color = "blue")
            plt.show()


    for i in range(10):
        test_random_case()


def test_bvs_partition():

    p0 = BVSPartition(0.0, 10.0, 0.3)
    assert len(p0) == 36

    # test get interval and get index from value.
    for i in range(100):
        f = random_float(0.0, 10.0)
        assert 0.0<=f <= 10.0

        id = p0.get_index_from_value(f)
        assert p0.get_interval(id).is_inside(f)

    # test get overlapping range

    for i in range(1000):

        start = random_float(-10.0,20.0)
        end = start + random_float(0.0, 5.0)
        target = Interval(start, end)

        res = p0.overlapping_index_range(Interval(start, end))
        if start > 10.0 or end < 0.0:
            assert res is None
        else:
            for j in range(len(p0)):
                overlaps= (res[0]<= j<res[1])
                overlaps_ground_truth = not p0.get_interval(j).intersect(target).is_void()
                assert overlaps_ground_truth == overlaps


def test_uniform_partition():
    p0 = UniformPartition(0.0, 0.3)

    # test get interval and get index from value.
    for i in range(100):
        f = random_float(0.0, 10.0)

        id = p0.get_index_from_value(f)
        assert p0.get_interval(id).is_inside(f)

    # test get overlapping range

    assert (0, 1) == p0. overlapping_index_range(Interval(0.0, 0.3,end_open=True))
    assert (0, 2) == p0.overlapping_index_range(Interval(0.0, 0.3, end_open=False))

    for i in range(1000):

        start = random_float(-10.0, 20.0)
        end = start + random_float(0.0, 5.0)
        target = Interval(start, end)

        res = p0.overlapping_index_range(Interval(start, end))

        for j in range(*res):
            assert p0.get_interval(j).intersect(target)

        assert not p0.get_interval(res[1]).intersect(target)
        assert not p0.get_interval(res[0]-1).intersect(target)



if __name__ == "__main__":
    test_vec2()
    test_seg2()
    test_interval()
    test_box_intersection()
    test_uniform_partition()
    test_bvs_partition()

