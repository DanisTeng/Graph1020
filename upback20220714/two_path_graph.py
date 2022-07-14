#!/usr/bin/env python3
"""
Copyright @2022 QCraft AI Inc. All rights reserved.
Authors: huaiyuan@qcraft.ai (Huaiyuan Teng)

Graph describing two paths with discrete dynamic1d
"""
import os
import pickle
import time
from queue import Queue
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

from experimental.users.huaiyuan.regulated_safety.box_set import IntInterval
from experimental.users.huaiyuan.regulated_safety.discrete_dynamic1d import (
    DiscreteDynamic1d,
    DiscreteDynamic1dParam,
)
from experimental.users.huaiyuan.regulated_safety.dynamic1d import Dynamic1d
from experimental.users.huaiyuan.regulated_safety.geometry import *
from experimental.users.huaiyuan.regulated_safety.path import StraightPath
from experimental.users.huaiyuan.regulated_safety.vehicle import Vehicle


class RegulationBase:
    def name(self) -> str:
        pass

    def is_a1(self) -> bool:
        pass

    def can_do(
        self, state_id: "TwoPathGraph.StateId", control_id: DiscreteDynamic1d.ControlId
    ) -> bool:
        pass


class RegulationByTableReal(RegulationBase):
    BrakeReg = 0
    AccReg = 1
    GeneralReg = 2

    def __init__(self, min_jid, max_jid, is_a1=False, _name=""):
        """
        For any state, there is no "no choice" condition.
        Each state must have at least one control id to be can do.
        :param raw_table:
        :param unsafe_set:
        :param is_a1:
        :param _name:
        :param destroy_raw_table:
        """
        self.min_jid = min_jid
        self.max_jid = max_jid
        self.full_actions = set(range(self.min_jid, self.max_jid + 1))
        self._is_a1 = is_a1
        self._name = _name
        self.regulation_at_state: Dict[
            "TwoPathGraph.StateId", Tuple[int, Union[int, Set[int]]]
        ] = {}

    def name(self) -> str:
        return self._name

    def set_name(self, name_: str):
        self._name = name_

    def is_a1(self) -> bool:
        return self._is_a1

    def forbidden_jids(self, state_id: "TwoPathGraph.StateId"):
        if state_id not in self.regulation_at_state:
            return set()
        else:
            enum, content = self.regulation_at_state[state_id]
            if enum == self.BrakeReg:
                return set(range(content + 1, self.max_jid + 1))
            elif enum == self.AccReg:
                return set(range(self.min_jid, content))
            else:
                return content

    def _get_reg_enum_content(self, forbidden_jids: Set[int]):
        if len(forbidden_jids) == 0:
            return None
        # determine color
        jid_ok_num = 0
        prev_ok = None
        rise = False
        drop = False
        # 3 types: fully ok, fully banned, ban acc, ban brake
        # mixed ban.
        last_rise_jid = None
        last_drop_jid = None
        for jid in range(self.min_jid, self.max_jid + 1):
            jid_ok = jid not in forbidden_jids
            if jid_ok:
                jid_ok_num += 1
            if prev_ok is not None:
                if prev_ok and not jid_ok:
                    last_drop_jid = jid
                    drop = True
                if not prev_ok and jid_ok:
                    last_rise_jid = jid
                    rise = True
            prev_ok = jid_ok

        if rise and not drop:
            # Acc
            assert last_rise_jid is not None
            return self.AccReg, last_rise_jid
        elif drop and not rise:
            # Brake
            assert last_drop_jid is not None
            return self.BrakeReg, last_drop_jid - 1
        elif not drop and not rise:
            return None
        else:
            # drop and rise
            return self.GeneralReg, forbidden_jids

    def complement(self, jids: Set[int]):
        return self.full_actions - jids

    def set_reg(self, state_id: "TwoPathGraph.StateId", forbidden_jids: Set[int]):
        """
        Will judge whether the forbidden set can be simplified to acc or brake reg.
        :param forbidden_jids:
        :return:
        """
        assert len(forbidden_jids) < len(self.full_actions)

        for jid in forbidden_jids:
            assert self.min_jid <= jid <= self.max_jid
        enum_content = self._get_reg_enum_content(forbidden_jids)
        if enum_content is not None:
            self.regulation_at_state[state_id] = enum_content

    def can_do(
        self, state_id: "TwoPathGraph.StateId", control_id: DiscreteDynamic1d.ControlId
    ) -> bool:
        if state_id not in self.regulation_at_state:
            return True
        else:
            enum, content = self.regulation_at_state[state_id]
            if enum == self.BrakeReg:
                return control_id <= content
            elif enum == self.AccReg:
                return control_id >= content
            else:
                return control_id not in content

    # TODO(huaiyuan): implement
    def union_in_place(self, other_reg: RegulationBase):
        """
        Definition : Only both forbidden actions are forbidden.
        :param other_reg:
        :return:
        """
        # Mark those to delete as EmptyReg, None
        keys_to_remove = []
        for state_id in self.regulation_at_state.keys():
            forbidden_jids = self.forbidden_jids(state_id)

            new_forbidden = set()
            for jid in forbidden_jids:
                if not other_reg.can_do(state_id, jid):
                    new_forbidden.add(jid)
            new_enum_content = self._get_reg_enum_content(new_forbidden)
            if new_enum_content is None:
                keys_to_remove.append(state_id)
            else:
                self.regulation_at_state[state_id] = new_enum_content

        for state_id in keys_to_remove:
            self.regulation_at_state.pop(state_id)


class RegulationByRegUnion(RegulationBase):
    """
    An action is forbidden only when both reg1 and reg2 forbids it.
    """

    def __init__(self, reg_1: RegulationBase, reg_2: RegulationBase, _name: str = ""):
        self._name = _name
        self.reg_1 = reg_1
        self.reg_2 = reg_2

        assert self.reg_1.is_a1() == self.reg_2.is_a1()

    def is_a1(self) -> bool:
        return self.reg_1.is_a1()

    def name(self) -> str:
        return self._name

    def can_do(
        self, state_id: "TwoPathGraph.StateId", control_id: DiscreteDynamic1d.ControlId
    ) -> bool:
        return self.reg_1.can_do(state_id, control_id) or self.reg_2.can_do(
            state_id, control_id
        )


class RegulationByBrakeRight(RegulationBase):
    """
    When acc > max acc, vehicle shall perform decelerate.
    When performing that deceleration to max_acc, vehicle shall
    at least use jerk < max_jerk

    """

    def __init__(
        self, dd1d: DiscreteDynamic1d, is_a1: bool, max_acc, max_jerk, name: str = ""
    ):
        assert max_jerk < 0.0
        # avoid reaching state containing max acc
        max_aid = dd1d.a_to_aid(max_acc) - 1

        max_jid = dd1d.jerk_round_to_control_id(max_jerk)
        while dd1d.control_id_to_jerk(max_jid) > max_jerk:
            max_jid -= 1

        self.max_jid = clamp(max_jid, dd1d.min_jid, dd1d.max_jid)
        self.max_aid = clamp(max_aid, dd1d.all_aids()[0], dd1d.all_aids()[-1])
        self._name = name
        self.dd1d = dd1d
        self._is_a1 = is_a1

    def is_a1(self) -> bool:
        return self._is_a1

    def name(self):
        return self._name

    def can_do(
        self, state_id: "TwoPathGraph.StateId", control_id: DiscreteDynamic1d.ControlId
    ) -> bool:
        if self._is_a1:
            single_state, _ = state_id
        else:
            _, single_state = state_id

        sid, vid, aid = single_state
        if aid > self.max_aid:
            # must decelerate
            return control_id <= self.max_jid
        else:
            # cannot accelerate to aid > max_aid
            frs = self.dd1d.state_transition(single_state, control_id)
            for s_next in frs:
                if s_next[2] > self.max_aid:
                    return False
            return True


class PredicateBase:
    def name(self) -> str:
        pass

    def evaluate(self, state_id) -> bool:
        pass


class PredicateByTable(PredicateBase):
    def __init__(self, name_: str = ""):
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

    def rename(self, name_: str):
        self.name_ = name_


class SingleAgentParam:
    def __init__(self):
        self.dd1d_param = DiscreteDynamic1dParam()


class SingleAgentMeta:
    """
    Meta for single agent
    """

    def __init__(self, path: StraightPath, vehicle: Vehicle, param: SingleAgentParam):
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

    def head(self):
        return self.discrete_dynamic.head() + [self.path, self.vehicle]


TwoPathGraphlog = True


class TwoPathGraph:
    """
    The graph of two agents.
    """

    CacheExtension = ".tpgx"
    StateId = Tuple[DiscreteDynamic1d.StateId, DiscreteDynamic1d.StateId]
    ControlId = Tuple[int, int]

    def __init__(
        self,
        a1: SingleAgentMeta,
        a2: SingleAgentMeta,
        predicates: List[PredicateBase] = None,
        regulations: List[RegulationBase] = None,
    ):
        self.a1 = a1
        self.a2 = a2

        self.predicates = [] if predicates is None else predicates
        self.index_of_predicate = {
            self.predicates[i].name(): i for i in range(len(self.predicates))
        }
        assert len(self.index_of_predicate) == len(
            self.predicates
        ), "Repeated pred name!"

        #
        self.regulations = [] if regulations is None else regulations
        self.index_of_regulation = {
            self.regulations[i].name(): i for i in range(len(self.regulations))
        }
        assert len(self.index_of_regulation) == len(
            self.index_of_regulation
        ), "Repeated reg name!"

        self.regulation_binding_unsafe_set: Dict[str, str] = {}

        # Two agents share same transition time tick.
        self.dt = self.a1.discrete_dynamic.dt
        assert self.dt == self.a2.discrete_dynamic.dt

    def has_predicate(self, pred_name: str):
        return pred_name in self.index_of_predicate

    def has_regulation(self, reg_name: str):
        return reg_name in self.index_of_regulation

    def add_predicate(self, predicate: PredicateBase):
        if predicate.name() in self.index_of_predicate:
            # predicate already exist
            return False
        self.predicates.append(predicate)
        self.index_of_predicate[predicate.name()] = len(self.predicates) - 1
        return True

    def add_regulation(self, regulation: RegulationBase):
        if regulation.name() in self.index_of_regulation:
            # regulation already exist
            return False

        self.regulations.append(regulation)
        self.index_of_regulation[regulation.name()] = len(self.regulations) - 1
        return True

    def sorted_regulation_names(self):
        names = list(self.index_of_regulation.keys())
        names.sort()
        return names

    def class_name(self):
        return "TwoPathGraph"

    def bind_regulation_with_unsafe_set(self, reg_name: str, unsafe_pred_name: str):
        if not self.has_regulation(reg_name):
            return
        if not self.has_predicate(unsafe_pred_name):
            return
        self.regulation_binding_unsafe_set[reg_name] = unsafe_pred_name

    def get_regulation_binding_unsafe_set(
        self, reg_name: str
    ) -> Optional[PredicateBase]:
        if not self.has_regulation(reg_name):
            return None
        if reg_name not in self.regulation_binding_unsafe_set:
            return None
        return self.predicate(self.regulation_binding_unsafe_set[reg_name])

    def head(self):
        # The compatibility check is not really enough
        # Yet it is necessary for two graph to be same
        # such that their sorted pred & reg names are the same.
        # However, it is still possible that same table be used in two
        # reg graphs, with the advance of study.
        # only base prefix is now used for compatibility check.
        """
        pred_names = [pred.name() for pred in self.predicates]
        pred_names.sort()
        reg_names = [reg.name() for reg in self.regulations]
        reg_names.sort()
        :return:
        """
        return self.a1.head() + self.a2.head()  # + [pred_names, reg_names]

    def log(self, message):
        if TwoPathGraphlog:
            print("TwoPathGraph: " + message)

    def may_dump_to_cache(
        self,
        cache_dir: str,
        cache_prefix="base_",
        name_white_list: Optional[Set[str]] = None,
    ):
        """
        The function will storage the result of predicate and regulation into files.
        If same name compatible file exists in the cache, the dump action will be skipped.
        :param cache_dir: a directory used to store caches.
        :param cache_prefix: used to distinguish tables.
        :return:
        """
        if name_white_list is None:
            name_white_list = set()

        if not os.path.exists(cache_dir):
            os.mkdir(cache_dir)
        self_head = self.head()

        predicate_file_and_name = self.list_compatible_filenames(
            cache_dir, "pred", cache_prefix
        )
        existing_pred_names = {p_n[1] for p_n in predicate_file_and_name}

        for pred in self.predicates:
            # Check whether same table exists.
            if pred.name() in existing_pred_names:
                self.log("Skipped pred %s, table already exists." % pred.name())
                continue

            if pred.name() not in name_white_list:
                self.log("Skipped pred %s." % pred.name())
                continue

            filename = (
                cache_prefix
                + str(time.time())
                + ".pred."
                + pred.name()
                + TwoPathGraph.CacheExtension
            )
            self.log("Dumping %s to %s" % (pred.name(), filename))

            with open(os.path.join(cache_dir, filename), "wb") as fp:
                pickle.dump(self_head, fp)
                pickle.dump(pred, fp)
                self.log("Dumped file: %s" % filename)

        regulation_file_and_name = self.list_compatible_filenames(
            cache_dir, "reg", cache_prefix
        )
        existing_reg_names = {p_n[1] for p_n in regulation_file_and_name}

        for reg in self.regulations:
            # Check whether same table exists.
            if reg.name() in existing_reg_names:
                self.log("Skipped reg %s, table already exists." % reg.name())
                continue

            if reg.name() not in name_white_list:
                self.log("Skipped pred %s." % reg.name())
                continue

            filename = (
                cache_prefix
                + str(time.time())
                + ".reg."
                + reg.name()
                + TwoPathGraph.CacheExtension
            )
            self.log("Dumping %s to %s" % (reg.name(), filename))

            with open(os.path.join(cache_dir, filename), "wb") as fp:
                pickle.dump(self_head, fp)
                pickle.dump(reg, fp)
                self.log("Dumped file: %s" % filename)

    def list_compatible_filenames(
        self, cache_dir: str, type_="pred", cache_prefix="base_"
    ) -> List[Tuple[str, str]]:
        """

        :param cache_dir:
        :param type_:
        :param cache_prefix:
        :return: list of filename, table_name
        """
        if not os.path.exists(cache_dir):
            return []

        mid = ".%s." % type_

        res = []
        all_file_names = os.listdir(cache_dir)
        self_head = self.head()
        for filename in all_file_names:
            # parse filename
            if filename[: len(cache_prefix)] != cache_prefix:
                # must have same prefix
                continue

            m_id = filename.find(mid)
            suffix_id = filename.find(TwoPathGraph.CacheExtension)
            abs_fn = os.path.join(cache_dir, filename)

            if suffix_id > m_id > 0:
                # open predicate file
                with open(abs_fn, "rb") as fp:
                    head = pickle.load(fp)
                    table_name = filename[m_id + len(mid) : suffix_id]
                    if head == self_head and table_name != "":
                        # compatible table file found.
                        res.append((filename, table_name))
        return res

    def try_load_from_cache(self, cache_dir: str, cache_prefix="base_"):
        """
        Pre-requisite: empty predicate table/reg table exist.
        Will try to
        :param cache_dir:
        :return:
        """
        predicate_file_and_name = self.list_compatible_filenames(
            cache_dir, "pred", cache_prefix
        )
        for filename, pred_name in predicate_file_and_name:
            if self.has_predicate(pred_name):
                self.log("Skipped %s since already has pred %s" % (filename, pred_name))
                continue
            self.log("loading pred %s from %s" % (pred_name, filename))

            with open(os.path.join(cache_dir, filename), "rb") as fp:
                # skip head
                _ = pickle.load(fp)

                predicate = pickle.load(fp)
                self.add_predicate(predicate)
                self.log("loaded pred %s" % predicate.name())

        reg_file_and_name = self.list_compatible_filenames(
            cache_dir, "reg", cache_prefix
        )
        for filename, reg_name in reg_file_and_name:
            if self.has_regulation(reg_name):
                self.log("Skipped %s since already has reg %s" % (filename, reg_name))
                continue
            self.log("loading reg %s from %s" % (reg_name, filename))

            with open(os.path.join(cache_dir, filename), "rb") as fp:
                # skip head
                _ = pickle.load(fp)

                regulation = pickle.load(fp)
                self.add_regulation(regulation)
                self.log("loaded reg %s" % regulation.name())

    def predicate(self, name: str) -> PredicateBase:
        assert name in self.index_of_predicate
        return self.predicates[self.index_of_predicate[name]]

    def regulation(self, name: str) -> RegulationBase:
        assert self.has_regulation(name)
        return self.regulations[self.index_of_regulation[name]]

    def state_valid(self, state: StateId):
        a1_sva_id, a2_sva_id = state
        return self.a1.discrete_dynamic.state_valid(
            a1_sva_id
        ) and self.a2.discrete_dynamic.state_valid(a2_sva_id)

    def state_name_string(self, state: StateId) -> str:
        if not self.state_valid(state):
            return "(Invalid state)"
        else:
            a1_sva_id, a2_sva_id = state
            return (
                "("
                + self.a1.discrete_dynamic.state_name_string(a1_sva_id)
                + ","
                + self.a2.discrete_dynamic.state_name_string(a2_sva_id)
                + ")"
            )

    def try_parse_state_str(self, state_string: str) -> Optional[StateId]:
        try:
            # TODO(huaiyuan): make it possible for two path state to support different dd1d.
            state_ints = state_string.replace("(", "").replace(")", "").split(",")
            return (int(state_ints[0]), int(state_ints[1]), int(state_ints[2])), (
                int(state_ints[3]),
                int(state_ints[4]),
                int(state_ints[5]),
            )
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

        res += "predicates:\n"
        for pred in self.predicates:
            if pred.evaluate(state) is True:
                res += pred.name() + ": True \n"
            else:
                res += pred.name() + ": False \n"

        return res

    def interactive_debug_gui_main_loop(self):
        pass


class TwoPathGraphEx(TwoPathGraph):
    """
    Simply speaking, this class provides certain basic tables:
    1, collision
    and some commonly used functions.
    1, defend


    When your graph has the same need of a function, put it here.

    """

    CollisionName = "collision"
    A1FinalReg = "a1_final_reg"
    A1FinalUnsafe = "a1_final_reg_unsafe"
    CacheDir = os.path.join(kProjectRegulatedSafetyDir, "graph_table_cache")
    Prefix = "common_"

    @staticmethod
    def path_near_parallel(p1: StraightPath, p2: StraightPath):
        return abs(p1.unit.cross(p2.unit)) < 0.1

    def __init__(self, a1: SingleAgentMeta, a2: SingleAgentMeta):
        assert not TwoPathGraphEx.path_near_parallel(a1.path, a2.path)
        super(TwoPathGraphEx, self).__init__(a1, a2)

        # load everything
        self.try_load_from_cache(TwoPathGraphEx.CacheDir, TwoPathGraphEx.Prefix)

        # collision set initialization
        if not self.has_predicate(TwoPathGraphEx.CollisionName):
            self.build_initial_collision_set()

        # Dump everything
        self.may_dump_to_cache(
            TwoPathGraphEx.CacheDir,
            TwoPathGraphEx.Prefix,
            {TwoPathGraphEx.CollisionName},
        )

    def a1_defend_a2(
        self, a2_regulation: RegulationBase
    ) -> Tuple[RegulationByTableReal, PredicateByTable]:
        """
        :param a2_regulation:
        :return:
        """
        collision_pred: PredicateByTable = self.predicate(TwoPathGraphEx.CollisionName)
        unsafe_set = copy.copy(collision_pred.true_set)
        unhandled_set = Queue()
        for state_id in unsafe_set:
            unhandled_set.put(state_id)

        forbidden_actions: Dict[TwoPathGraph.StateId, Set[int]] = {}
        u1_jid_num = (
            self.a1.discrete_dynamic.max_jid + 1 - self.a1.discrete_dynamic.min_jid
        )

        log_indicator = 0
        tick_per_log = 10000
        while unhandled_set.qsize() > 0:
            log_indicator += 1
            if log_indicator > tick_per_log:
                log_indicator = 0
                debug_string = "unsafe_set: %8d, unhandled: %8d" % (
                    len(unsafe_set),
                    unhandled_set.qsize(),
                )
                self.log(debug_string)
            state = unhandled_set.get_nowait()

            s1, s2 = state

            s1_prev_list = self.a1.discrete_dynamic.backward_reachable_states(s1)
            s2_prev_list = self.a2.discrete_dynamic.backward_reachable_states(s2)
            for s1_p in s1_prev_list:
                for s2_p in s2_prev_list:
                    state_p = (s1_p, s2_p)
                    if state_p in unsafe_set:
                        continue
                    u1_set = self.a1.discrete_dynamic.transition_controls(s1_p, s1)
                    u2_set = self.a2.discrete_dynamic.transition_controls(s2_p, s2)

                    transition_regulated = False
                    for j2 in range(u2_set.min_jid, u2_set.max_jid + 1):
                        if a2_regulation.can_do(state_p, j2):
                            transition_regulated = True
                    if not transition_regulated:
                        continue

                    for j1 in range(u1_set.min_jid, u1_set.max_jid + 1):
                        if state_p in forbidden_actions:
                            set_of_interest = forbidden_actions[state_p]
                            set_of_interest.add(j1)
                            all_black_after_forbidden = (
                                len(set_of_interest) >= u1_jid_num
                            )
                        else:
                            forbidden_actions[state_p] = {j1}
                            all_black_after_forbidden = 1 >= u1_jid_num

                        if all_black_after_forbidden:
                            del forbidden_actions[
                                state_p
                            ]  # release memory as it is no longer used
                            unsafe_set.add(state_p)
                            unhandled_set.put(state_p)
                            break
        unsafe_pred = PredicateByTable()
        unsafe_pred.true_set = unsafe_set

        # Transform into smart reg table
        a1_regulation = RegulationByTableReal(
            self.a1.discrete_dynamic.min_jid,
            self.a1.discrete_dynamic.max_jid,
            is_a1=True,
        )
        for state, forbidden_jids in forbidden_actions.items():
            a1_regulation.set_reg(state, forbidden_jids)

        return a1_regulation, unsafe_pred

    def a2_defend_a1(
        self, a1_regulation: RegulationBase
    ) -> Tuple[RegulationByTableReal, PredicateByTable]:
        """
        :param a1_regulation:
        :return:
        """
        collision_pred: PredicateByTable = self.predicate(TwoPathGraphEx.CollisionName)
        unsafe_set = copy.copy(collision_pred.true_set)
        unhandled_set = Queue()
        for state_id in unsafe_set:
            unhandled_set.put(state_id)

        forbidden_actions: Dict[TwoPathGraph.StateId, Set[int]] = {}
        u2_jid_num = (
            self.a2.discrete_dynamic.max_jid + 1 - self.a2.discrete_dynamic.min_jid
        )

        log_indicator = 0
        tick_per_log = 10000
        while unhandled_set.qsize() > 0:
            log_indicator += 1
            if log_indicator > tick_per_log:
                log_indicator = 0
                debug_string = "unsafe_set: %8d, unhandled: %8d" % (
                    len(unsafe_set),
                    unhandled_set.qsize(),
                )
                self.log(debug_string)
            state = unhandled_set.get_nowait()

            s1, s2 = state

            s1_prev_list = self.a1.discrete_dynamic.backward_reachable_states(s1)
            s2_prev_list = self.a2.discrete_dynamic.backward_reachable_states(s2)
            for s1_p in s1_prev_list:
                for s2_p in s2_prev_list:
                    state_p = (s1_p, s2_p)
                    if state_p in unsafe_set:
                        continue
                    u1_set = self.a1.discrete_dynamic.transition_controls(s1_p, s1)
                    u2_set = self.a2.discrete_dynamic.transition_controls(s2_p, s2)

                    transition_regulated = False
                    for j1 in range(u1_set.min_jid, u1_set.max_jid + 1):
                        if a1_regulation.can_do(state_p, j1):
                            transition_regulated = True
                    if not transition_regulated:
                        continue

                    for j2 in range(u2_set.min_jid, u2_set.max_jid + 1):
                        if state_p in forbidden_actions:
                            set_of_interest = forbidden_actions[state_p]
                            set_of_interest.add(j2)
                            all_black_after_forbidden = (
                                len(set_of_interest) >= u2_jid_num
                            )
                        else:
                            forbidden_actions[state_p] = {j2}
                            all_black_after_forbidden = 1 >= u2_jid_num

                        if all_black_after_forbidden:
                            del forbidden_actions[
                                state_p
                            ]  # release memory as it is no longer used
                            unsafe_set.add(state_p)
                            unhandled_set.put(state_p)
                            break
        unsafe_pred = PredicateByTable()
        unsafe_pred.true_set = unsafe_set

        # Transform into smart reg table
        a2_regulation = RegulationByTableReal(
            self.a2.discrete_dynamic.min_jid,
            self.a2.discrete_dynamic.max_jid,
            is_a1=False,
        )
        for state, forbidden_jids in forbidden_actions.items():
            a2_regulation.set_reg(state, forbidden_jids)

        return a2_regulation, unsafe_pred

    def a2_sweep_region_by_a1(self, vid1, vid2, sid2) -> ConvexPolygon:
        """
        In the co-moving system of a1, the region that a2 sweeps in next dt.
        :param vid1:
        :param vid2:
        :param sid2:
        :return:
        """
        u1 = self.a1.path.unit
        u2 = self.a2.path.unit

        a2_s_range = self.a2.discrete_dynamic.s_interval(sid2)
        a2_s_step = a2_s_range.end - a2_s_range.begin
        a2_rac_to_back = self.a2.vehicle.length - self.a2.vehicle.rac_to_front
        rac_2 = self.a2.path.origin + u2 * a2_s_range.begin

        b2_with_s2_error = Box2(
            rac_2 - u2 * a2_rac_to_back,
            self.a2.vehicle.length + a2_s_step,
            self.a2.vehicle.width,
            u2,
            u2.perp(),
        )

        a1_speed_range = self.a1.discrete_dynamic.v_interval(vid1)
        a2_speed_range = self.a2.discrete_dynamic.v_interval(vid2)
        v00 = self.dt * (u2 * a2_speed_range.begin - u1 * a1_speed_range.begin)
        v10 = self.dt * (u2 * a2_speed_range.end - u1 * a1_speed_range.begin)
        v01 = self.dt * (u2 * a2_speed_range.begin - u1 * a1_speed_range.end)
        v11 = self.dt * (u2 * a2_speed_range.end - u1 * a1_speed_range.end)

        u2_cross_neg_u1 = u2.cross(-u1)
        if u2_cross_neg_u1 > 0:
            # u2, -u1 is right-hand axis
            return sum_vehicle_box_with_speed_cone(
                b2_with_s2_error, [v00, v10, v11, v01]
            )
        else:
            # u2, -u1 is left-hand axis
            return sum_vehicle_box_with_speed_cone(
                b2_with_s2_error, [v00, v01, v11, v10]
            )

    def colliding_s2_range(self, sid1, vid1, vid2) -> IntInterval:
        """
        Will judge collision using constant speed model.
        :param sid1:
        :param vid1:
        :param aid1:
        :param vid2:
        :param aid2:
        :return:
        """
        # In a1 co-moving axis, x-axis is parallel with a1 forward direction.

        u1 = self.a1.path.unit
        u2 = self.a2.path.unit
        o1 = self.a1.path.origin
        o2 = self.a2.path.origin

        # Use following a2 axis, a2a
        a2a_nx = u2
        a2a_ny = u2.perp()
        a2a_origin = o2

        def ToA2Axis(pt: Vec2):
            offset = pt - a2a_origin
            return Vec2(offset.dot(a2a_nx), offset.dot(a2a_ny))

        # Build a1's box with s1 uncertainty in a2a
        a1_rac_to_back = self.a1.vehicle.length - self.a1.vehicle.rac_to_front
        a1_s_range = self.a1.discrete_dynamic.s_interval(sid1)
        a1_s_step = a1_s_range.end - a1_s_range.begin
        rac_1_a2a = ToA2Axis(o1 + u1 * a1_s_range.begin)
        u1_a2a = Vec2(u1.dot(a2a_nx), u1.dot(a2a_ny))

        b1_with_s1_error_a2a = Box2(
            rac_1_a2a - u1_a2a * a1_rac_to_back,
            self.a1.vehicle.length + a1_s_step,
            self.a1.vehicle.width,
            u1_a2a,
            u1_a2a.perp(),
        )

        # Build a2's box use sid2 = 0.0

        b2_sweep_0 = self.a2_sweep_region_by_a1(vid1, vid2, 0)
        b2_sweep_0_a2a = ConvexPolygon([ToA2Axis(v) for v in b2_sweep_0.vertices])

        # Now the problem becomes: find all sid2 such that:
        # 1, b2_sweep_0_a2a + Vec(1.0, 0.0) * sid2 * a2_s_step
        # 2, b1_with_s1_error_a2a
        # 1 and 2 overlaps.
        # -> Find all sid2 s.t.
        # Vec(1.0, 0.0) * sid2 * a2_s_step \in (b1_with_s1_error_a2a - b2_sweep_0_a2a)

        m_diff = b2_sweep_0_a2a.negation().minkowski_sum_with_box(b1_with_s1_error_a2a)

        y_a2a = (Vec2(0.0, 0.0) - a2a_origin).dot(a2a_ny)
        x_a2a_range = m_diff.x_range_in_y_interval(Interval(y_a2a, y_a2a))

        if x_a2a_range.is_void():
            return IntInterval.void()

        sid2_min = math.ceil(x_a2a_range.begin / a1_s_step)
        sid2_max = math.floor(x_a2a_range.end / a1_s_step)

        if sid2_min <= sid2_max:
            return IntInterval(sid2_min, sid2_max + 1)
        else:
            return IntInterval.void()

    def build_initial_collision_set(self):
        """
        Not capable of handling parallel case.
        :return:
        """
        assert not self.has_predicate(TwoPathGraphEx.CollisionName)
        self.log("Building initial collision set")
        self.add_predicate(PredicateByTable(TwoPathGraphEx.CollisionName))

        line1 = Seg2(
            self.a1.path.origin,
            self.a1.path.origin + self.a1.path.unit,
            self.a1.path.unit,
        )
        line2 = Seg2(
            self.a2.path.origin,
            self.a2.path.origin + self.a2.path.unit,
            self.a2.path.unit,
        )

        intersect = line1.line_intersect(line2)
        assert intersect is not None
        sid1_at_intersect = self.a1.discrete_dynamic.s_to_sid(
            (intersect - self.a1.path.origin).dot(self.a1.path.unit)
        )

        all_vid1 = self.a1.discrete_dynamic.all_vids()
        for vid1 in all_vid1:
            debug_string = "vid1 progress: (%d/%d)" % (vid1, len(all_vid1))
            self.log(debug_string)
            for vid2 in self.a2.discrete_dynamic.all_vids():
                sid1 = sid1_at_intersect + 1

                while True:
                    sid2_range = self.colliding_s2_range(sid1, vid1, vid2)
                    if not sid2_range:
                        break
                    else:
                        for sid2 in range(sid2_range.begin, sid2_range.end):
                            for aid1 in self.a1.discrete_dynamic.all_aids():
                                for aid2 in self.a2.discrete_dynamic.all_aids():
                                    state_id = ((sid1, vid1, aid1), (sid2, vid2, aid2))
                                    self.predicate(
                                        TwoPathGraphEx.CollisionName
                                    ).set_true(state_id)
                        sid1 += 1

                sid1 = sid1_at_intersect
                while True:
                    sid2_range = self.colliding_s2_range(sid1, vid1, vid2)
                    if not sid2_range:
                        break
                    else:
                        for sid2 in range(sid2_range.begin, sid2_range.end):
                            for aid1 in self.a1.discrete_dynamic.all_aids():
                                for aid2 in self.a2.discrete_dynamic.all_aids():
                                    state_id = ((sid1, vid1, aid1), (sid2, vid2, aid2))
                                    self.predicate(
                                        TwoPathGraphEx.CollisionName
                                    ).set_true(state_id)
                        sid1 -= 1

        self.log("Initial collision set built")


def sum_vehicle_box_with_speed_cone(
    box: Box2, speed_vertices_ccw: List[Vec2]
) -> ConvexPolygon:
    """
    ccw vertices can be on the same line, can be
    :param box:
    :param speed_vertices_ccw:
    :return:
    """
    assert speed_vertices_ccw
    # remove repeated points
    box_polygon = box.to_convex_polygon()

    prev: Optional[Vec2] = None
    pts = []
    for pt in speed_vertices_ccw:
        if prev is not None and (pt - prev).is_zero():
            continue
        pts.append(pt)
        prev = pt

    if len(pts) == 1:
        v = pts[0]
        if v.is_zero():
            return box_polygon
        else:
            return box_polygon.minkowski_sum_with_vec(v)

    # whether all pts are in the same line
    pts_in_same_line = True
    e_0 = pts[1] - pts[0]

    for i in range(len(pts)):
        if abs((pts[i] - pts[0]).cross(e_0)) > kEps:
            pts_in_same_line = False
            break

    if pts_in_same_line:
        # find extreme points
        v0 = pts[0]
        v1 = pts[0]
        for pt in pts:
            offset = pt.dot(e_0)
            if offset < v0.dot(e_0):
                v0 = pt
            if offset > v1.dot(e_0):
                v1 = pt

        assert not (v1 - v0).is_zero()

        lat = (v1 - v0).cross(v0)
        if abs(lat) < kEps:
            # in same line with origin
            if v1.dot(v0) < 0:
                return box_polygon.minkowski_sum_with_vec(v1).minkowski_sum_with_vec(v0)
            else:
                if v1.length_sqr() > v0.length_sqr():
                    return box_polygon.minkowski_sum_with_vec(v1)
                else:
                    return box_polygon.minkowski_sum_with_vec(v0)
        else:
            if lat < 0:
                cone = ConvexPolygon([Vec2(0, 0), v0, v1])
            else:
                cone = ConvexPolygon([Vec2(0, 0), v1, v0])
            return cone.minkowski_sum_with_box(box)
    else:
        sb_bloom = ConvexPolygon(speed_vertices_ccw)
        sb_cone = sb_bloom.convex_hull_with_point(Vec2(0.0, 0.0))
        return sb_cone.minkowski_sum_with_box(box)
