from discrete_dynamic_1d import *
from random import seed

_debug_plot = True
if _debug_plot:
    from matplotlib import pyplot as plt

    class DiscreteTrajectoryRecord:
        def __init__(self):
            self.SMIN = []
            self.SMAX = []
            self.VMIN = []
            self.VMAX = []
            self.AMIN = []
            self.AMAX = []
            self.T = []

        def mark(self, s_min, s_max, v_min, v_max, a_min, a_max, t):
            self.SMIN.append(s_min)
            self.SMAX.append(s_max)
            self.VMIN.append(v_min)
            self.VMAX.append(v_max)
            self.AMIN.append(a_min)
            self.AMAX.append(a_max)
            self.T.append(t)

        def plot(self, figure_id=0):
            if not self.SMIN:
                return

            plt.figure(figure_id)
            plt.plot(self.T, self.SMIN, color="black")
            plt.plot(self.T, self.SMAX, color="black")
            plt.plot(self.T, self.VMIN, color="#2222CC")
            plt.plot(self.T, self.VMAX, color="#2222CC")
            plt.plot(self.T, self.AMIN, color="red")
            plt.plot(self.T, self.AMAX, color="red")

            plt.legend(['s_min', 's_max', 'v_min', 'v_max', 'a_min', 'a_max'])

            plt.xlabel('time (s)')


def plot_demo_curve(v0, a0, jc):
    param = DiscreteDynamic1dParam()
    param.dense_sample_dv = 0.1
    param.dense_sample_da = 0.1
    param.dt = 0.5
    d0 = DiscreteDynamic1d(Dynamic1d(), param)

    s0 = 0.0

    state0 = d0.s_to_sid(s0), d0.v_to_vid(v0), d0.a_to_aid(a0)
    control0 = d0 .jerk_round_to_control_id(jc)

    traj = DiscreteTrajectoryRecord()

    all_states = {state0}
    for i in range(20):
        s_min = kInf
        s_max = -kInf
        v_min = kInf
        v_max = -kInf
        a_min = kInf
        a_max = -kInf
        for state in all_states:
            sid, vid, aid = state
            s_max = max(s_max, d0.s_interval(sid).end)
            s_min = min(s_min, d0.s_interval(sid).begin)
            v_max = max(v_max, d0.v_interval(vid).end)
            v_min = min(v_min, d0.v_interval(vid).begin)
            a_max = max(a_max, d0.a_interval(aid).end)
            a_min = min(a_min, d0.a_interval(aid).begin)

        traj.mark(s_min,s_max, v_min,v_max,a_min,a_max, i*param.dt)

        next_all_states = set()
        for state in all_states:
            next_states = d0.state_transition(state, control0)
            next_all_states.update(next_states)
        all_states = next_all_states

    traj.plot()
    plt.show()
    # ====


def test_reachability():
    param = DiscreteDynamic1dParam()
    param.dense_sample_dv = 0.1
    param.dense_sample_da = 0.1
    param.dt = 0.5
    d0 = Dynamic1d()
    dd0 = DiscreteDynamic1d(d0, param)

    for i in range(100):
        s = random_float(-100., 100., )
        v = random_float(0.0, 22.)
        a = random_float(-8.0, 3.0)
        j = random_float(-10.0, 10.0)

        s, v, a = d0.clamp_state(s, v, a)
        from_state_id = dd0.s_to_sid(s), dd0.v_to_vid(v), dd0.a_to_aid(a)
        control_id = dd0.jerk_round_to_control_id(j)
        to_states = dd0.state_transition(from_state_id, control_id)

        for to_state in to_states:
            assert control_id in dd0.transition_controls(from_state_id, to_state)

            brs = dd0.back_transition(to_state, control_id)
            assert from_state_id in brs

    for i in range(100):
        s = random_float(-100., 100., )
        v = random_float(0.0, 22.)
        a = random_float(-8.0, 3.0)
        j = random_float(-10.0, 10.0)

        s, v, a = d0.clamp_state(s, v, a)
        to_state_id = dd0.s_to_sid(s), dd0.v_to_vid(v), dd0.a_to_aid(a)
        control_id = dd0.jerk_round_to_control_id(j)
        from_states = dd0.back_transition(to_state_id, control_id)

        for from_state in from_states:
            assert control_id in dd0.transition_controls(from_state, to_state_id)

            frs = dd0.state_transition(from_state, control_id)
            assert to_state_id in frs


if __name__ =="__main__":
    seed(0)

    if _debug_plot:
        plot_demo_curve(0.0, 0.0, 1.0)
        plot_demo_curve(6.0, 0.0, -1.0)

    test_reachability()
