from utils import clamp, head_equal,lerp_color


def test_clamp():
    assert clamp(0.0, 1.0, 2.0) == 1.0
    assert clamp(0.0, -2.0, -1.0) == -1.0


def test_head_equal():
    class SomeHead:
        def __init__(self,v):
            self.v = v

        def __eq__(self, other:"SomeHead"):
            return self.v == other.v

    h1 = [1,"dsd", None, SomeHead(10)]
    h2 = [1,"dsd", None, SomeHead(10)]

    assert head_equal(h1, h2)


def test_lerp_color():
    assert lerp_color("#00ff00", "#ff0000", 0.4) == "#669900"

if __name__ == "__main__":
    test_clamp()
    test_head_equal()
    test_lerp_color()