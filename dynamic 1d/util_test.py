from util import Dynamic1d
from random import random,seed

_debug_plot = True
if _debug_plot:
    from matplotlib import pyplot as plt


    class TrajectoryRecord:
        def __init__(self):
            self.S = []
            self.V = []
            self.A = []
            self.T = []

        def mark(self, s, v, a, t):
            self.S.append(s)
            self.V.append(v)
            self.A.append(a)
            self.T.append(t)

        def plot(self, figure_id=0):
            if not self.S:
                return

            plt.figure(figure_id)
            plt.plot(self.T, self.S)
            plt.plot(self.T, self.V)
            plt.plot(self.T, self.A)
            plt.legend(['s', 'v', 'a'])

            plt.xlabel('time (s)')

def _random_float(min_v, max_v):
    return random()*(max_v - min_v) + min_v

def plot_demo_curve():
    d0 = Dynamic1d()

    s0 = 0.0
    v0 = 0.0
    a0 = 1.0
    jc = 0.5

    t_samples = [i * 0.2 + 0.2 for i in range(50)]
    t_step_hint = 0.07

    traj = TrajectoryRecord()
    for t in t_samples:
        s, v, a = d0.state_transition_numerical(s0, v0, a0, jc, t, t_step_hint)
        traj.mark(s, v, a, t)

    traj.plot()
    plt.show()
    # ====

    traj = TrajectoryRecord()
    for t in t_samples:
        s, v, a = d0.state_transition(s0, v0, a0, jc, t)
        traj.mark(s, v, a, t)

    traj.plot()
    plt.show()


def test_dynamic_1d():
    d0 = Dynamic1d()

    for i in range(100):
        s = _random_float(-100., 100.,)
        v = _random_float(0.0, 22.)
        a = _random_float(-8.0, 3.0)
        j = _random_float(-10.0, 10.0)

        delta_t = _random_float(0.1, 8.0)
        analytical_res = d0.state_transition(s, v, a, j, delta_t)

        numerical_res = d0.state_transition_numerical(s, v, a, j,delta_t, 0.001)

        #print("testing %f,%f,%f,j = %f delta_t: %f"%(s,v,a,j, delta_t))
        #for i in range(3):
        #   print(analytical_res[i] - numerical_res[i])

        station_error = abs(analytical_res[0] - numerical_res[0])
        speed_error = abs(analytical_res[1] - numerical_res[1])
        acc_error = abs(analytical_res[2] - numerical_res[2])

        assert station_error < 0.1
        assert  speed_error < 0.01
        assert acc_error < 0.01


def benchmark_dynamic_1d_state_transition():
    import time
    d0 = Dynamic1d()
    n = 100

    seed(0)
    t0= time.perf_counter()
    for i in range(n):
        s = _random_float(-100., 100., )
        v = _random_float(0.0, 22.)
        a = _random_float(-8.0, 3.0)
        j = _random_float(-10.0, 10.0)

        delta_t = _random_float(0.1, 8.0)
        analytical_res = d0.state_transition(s, v, a, j, delta_t)
    print("Analytical function takes: %f"%(time.perf_counter() - t0))

    seed(0)
    t0 = time.perf_counter()
    for i in range(n):
        s = _random_float(-100., 100., )
        v = _random_float(0.0, 22.)
        a = _random_float(-8.0, 3.0)
        j = _random_float(-10.0, 10.0)

        delta_t = _random_float(0.1, 8.0)
        numerical_res = d0.state_transition_numerical(s, v, a, j, delta_t, 0.001)
    print("numerical function takes: %f" % (time.perf_counter() - t0))


if __name__ == "__main__":
    seed(0)
    test_dynamic_1d()
    benchmark_dynamic_1d_state_transition()
