from path import *
from random import seed


def test_straight_path():
    p = StraightPath(Vec2(0.0, 0.0), direction=Vec2.form_unit(0.0))

    state = p.get_state(3.0, 1.0, 1.0)

    assert state[X_ID] == 3.0
    assert state[Y_ID] == 0.0
    assert state[THETA_ID] == 0.0
    assert state[KAPPA_ID] == 0.0
    assert state[V_ID] == 1.0
    assert state[A_ID] == 1.0


if __name__ == "__main__":
    seed(0)
    test_straight_path()
