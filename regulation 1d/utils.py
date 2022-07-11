import random

kEps = 1e-9
kInf = float('inf')
kIntInf = kInf


def clamp(v, min_v, max_v):
    return min(max_v, max(min_v, v))


def random_float(min_v, max_v):
    return random.random() * (max_v - min_v) + min_v


def lerp(start, end, ratio):
    return start + (end - start) * ratio


def head_equal(head1, head2):
    assert isinstance(head1, list)
    assert isinstance(head2, list)

    if len(head1) != len(head2):
        return False

    for i in range(len(head1)):
        if not isinstance(head2[i], type(head1[i])):
            return False
        if not head1 == head2:
            return False
    return True


def lerp_color(start, end, ratio):
    res = "#"
    for i in range(3):
        s_v = int("0x" + start[2 * i + 1:2 * i + 3], 0)
        e_v = int("0x" + end[2 * i + 1:2 * i + 3], 0)

        v = int(lerp(s_v, e_v, ratio))
        v = clamp(v, 0, 255)
        res += hex(v)[2:].zfill(2)
    return res
