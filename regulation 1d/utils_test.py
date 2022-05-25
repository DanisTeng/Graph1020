from utils import clamp


def test_clamp():
    assert clamp(0.0, 1.0, 2.0) == 1.0
    assert clamp(0.0, -2.0, -1.0) == -1.0


if __name__ == "__main__":
    test_clamp()
