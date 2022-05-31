from typing import List, Tuple, Set, Dict, Any, Optional, Callable
from path import PathBase
from vehicle import Vehicle
from discrete_dynamic_1d import DiscreteDynamic1dParam, DiscreteDynamic1d
from dynamic1d import Dynamic1d


class PredicateBase:
    def name(self) -> str:
        pass

    def evaluate(self, state_id) -> bool:
        pass


class PredicateByTable(PredicateBase):
    def __init__(self, name_: str):
        self.name_ = name_
        self.true_set = set()

    def name(self) -> str:
        return self.name_

    def evaluate(self, state_id) -> bool:
        return state_id in self.true_set

    def set_true(self, state_id):
        self.true_set.add(state_id)

    def set_false(self, state_id):
        self.true_set.discard(state_id)


class SinglePathParam:
    def __init__(self):
        self.dd1d_param = DiscreteDynamic1dParam()


class SingleAgentMeta:
    """
    Meta for single agent
    """

    def __init__(self,
                 path: PathBase,
                 vehicle: Vehicle,
                 param: SinglePathParam):
        self.path = path
        self.vehicle = vehicle
        self.param = param

        # Use ground truth for graph structure.
        # precedence, comfortableness are not in
        self.dynamic = Dynamic1d()
        self.dynamic.max_acc = self.vehicle.max_lng_acc
        self.dynamic.min_acc = self.vehicle.min_lng_acc
        self.dynamic.max_speed = self.path.max_speed(self.vehicle)
        self.dynamic.min_speed = 0.0

        self.dynamic.max_jerk = self.vehicle.max_lng_jerk
        self.dynamic.min_jerk = self.vehicle.min_lng_jerk

        self.discrete_dynamic = DiscreteDynamic1d(self.dynamic, self.param.dd1d_param)


class TwoPathGraph:
    """
    The graph of two agents.
    """

    StateId = Tuple[DiscreteDynamic1d.StateId, DiscreteDynamic1d.StateId]
    ControlId = Tuple[int, int]

    def __init__(self, a1: SingleAgentMeta, a2: SingleAgentMeta, predicates: List[PredicateBase]):
        self.a1 = a1
        self.a2 = a2

        self.predicates = predicates

        # Two agents share same transition time tick.
        self.dt = self.a1.discrete_dynamic.dt
        assert self.dt == self.a2.discrete_dynamic.dt


    def state_valid(self, state: StateId):
        a1_sva_id, a2_sva_id = state
        return self.a1.discrete_dynamic.state_valid(a1_sva_id) and self.a2.discrete_dynamic.state_valid(a2_sva_id)

    def state_name_string(self, state: StateId) -> str:
        if not self.state_valid(state):
            return "(Invalid state)"
        else:
            a1_sva_id, a2_sva_id = state
            return "(" + self.a1.discrete_dynamic.state_name_string(
                a1_sva_id) + "," + self.a2.discrete_dynamic.state_name_string(a2_sva_id) + ")"

    def try_parse_state_str(self, state_string: str) -> Optional[StateId]:
        try:
            # TODO(huaiyuan): make it possible for two path state to support different dd1d.
            state_ints = state_string.replace("(", "").replace(")", "").split(",")
            return (int(state_ints[0]), int(state_ints[1]), int(state_ints[2])), (
                int(state_ints[3]), int(state_ints[4]), int(state_ints[5]))
        except:
            return None

    def state_debug_info(self, state: StateId) -> str:
        a1_sva_id, a2_sva_id = state

        res = "===TwoPathGraphState===\n"
        if not self.state_valid(state):
            res += "(Invalid state)\n"
            return res
        res += self.state_name_string(state) + "\n"

        res += "Agent 1:\n"
        res += self.a1.discrete_dynamic.state_debug_info(a1_sva_id)
        res += "\n"
        res += "Agent 2:\n"
        res += self.a1.discrete_dynamic.state_debug_info(a2_sva_id)

        return res


    def interactive_debug_gui_main_loop(self):
        pass
