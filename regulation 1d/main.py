from two_path_graph_debug_viewer import TwoPathGraphViewer
from two_path_graph import *
from path import StraightPath
from geometry import *
import time

def test_two_path_graph_viewer():
    vehicle = Vehicle()
    a1_path = StraightPath(Vec2(0.0, 0.0), Vec2.form_unit(1.57))
    a2_path = StraightPath(Vec2(0.0, 0.0), Vec2.form_unit(0.0))
    single_path_graph_param = SinglePathParam()

    a1 = SingleAgentMeta(a1_path, vehicle, single_path_graph_param)
    a2 = SingleAgentMeta(a2_path, vehicle, single_path_graph_param)

    class A1AccPositive(PredicateBase):
        def __init__(self, a1_meta: SingleAgentMeta):
            self.a1 = a1_meta

        def name(self) -> str:
            return "a1 acc positive"

        def evaluate(self, state_id: TwoPathGraph.StateId) -> bool:
            s1, _ = state_id

            return self.a1.discrete_dynamic.a_interval(s1[2]).value_to_the_left(0.0)

    class A2Stationary(PredicateBase):
        def __init__(self, a2_meta: SingleAgentMeta):
            self.a2 = a2_meta

        def name(self) -> str:
            return "a2 stationary"

        def evaluate(self, state_id: TwoPathGraph.StateId) -> bool:
            _, s2 = state_id

            return self.a2.discrete_dynamic.v_interval(s2[1]).value_to_the_right(0.1)

    a1.discrete_dynamic.backward_reachable_states((0, 1, 36))

    a1_a2_all_zero = PredicateByTable("a1 a2 all zero")
    a1_a2_all_zero.set_true(((0, 0, 0), (0, 0, 0)))

    tpg = TwoPathGraph(a1, a2, [A1AccPositive(a1), A2Stationary(a2), a1_a2_all_zero])

    TwoPathGraphViewer(tpg, ((0, 2, 0), (0, 2, 0))).main_loop()


def test_initial_collision_set():
    vehicle = Vehicle()
    vehicle.max_lng_speed = 12.0
    a1_path = StraightPath(Vec2(0.0, 0.0), Vec2.form_unit(1.57))

    #a2_angle = 1.42
    a2_angle = 0.0
    a2_path = StraightPath(Vec2(0.0, 0.0), Vec2.form_unit(a2_angle))
    single_path_graph_param = SinglePathParam()
    single_path_graph_param.dd1d_param.da = 2.0
    single_path_graph_param.dd1d_param.dv = 1.0
    single_path_graph_param.dd1d_param.dj = 4.0
    single_path_graph_param.dd1d_param.ds = 0.25

    a1 = SingleAgentMeta(a1_path, vehicle, single_path_graph_param)
    a2 = SingleAgentMeta(a2_path, vehicle, single_path_graph_param)
    t0 = time.perf_counter()
    tpg = TwoPathGraphEx(a1, a2)
    print(time.perf_counter() - t0, " seconds elapsed.")

    TwoPathGraphViewer(tpg, ((0, 2, 0), (0, 2, 0))).main_loop()


if __name__ == "__main__":
    # test_two_path_graph_viewer()
    test_initial_collision_set()