#!/usr/bin/env python3
"""
Copyright @2022 QCraft AI Inc. All rights reserved.
Authors: huaiyuan@qcraft.ai (Huaiyuan Teng)

Discrete version of dynamic 1d.
"""

import copy
import os
import pickle
from time import time
from typing import Dict, List, Optional, Set, Tuple

from experimental.users.huaiyuan.regulated_safety.dynamic1d import Dynamic1d
from experimental.users.huaiyuan.regulated_safety.geometry import *
from experimental.users.huaiyuan.regulated_safety.utils import *


class DiscreteDynamic1dParam:
    def __init__(self):
        self.dense_sample_dv = 0.05
        self.dense_sample_da = 0.05
        self.ds = 0.1
        self.dv = 0.1
        self.da = 0.2
        self.dt = 0.5
        self.dj = 0.4

    def __eq__(self, other: "DiscreteDynamic1dParam"):
        return all(
            [
                self.dense_sample_dv == other.dense_sample_dv,
                self.dense_sample_da == other.dense_sample_da,
                self.ds == other.ds,
                self.dv == other.dv,
                self.da == other.da,
                self.dt == other.dt,
                self.dj == other.dj,
            ]
        )


DiscreteDynamic1dLog = True


class DiscreteDynamic1d:
    StateId = Tuple[int, int, int]
    ControlId = int

    # Transition control id, and the possible states that can be reached
    # from current state using such control.
    class JidSingleSet:
        def __init__(self, jid: int):
            self.min_jid = jid
            self.max_jid = jid

        def add(self, new_id):
            """
            Assume it is always a single interval
            :param new_id:
            :return:
            """
            self.min_jid = min(self.min_jid, new_id)
            self.max_jid = max(self.max_jid, new_id)

        def __contains__(self, jid: int):
            return self.min_jid <= jid <= self.max_jid

    ForwardReachableSet = Dict[StateId, JidSingleSet]

    # Transition control id, and the possible states that can lead to
    # current state using such control.
    BackwardReachableSet = Set[StateId]

    CacheSuffix = ".dd1dx"
    CacheDir = os.path.join(kProjectRegulatedSafetyDir, "discrete_dynamic1d_cache")

    def __init__(
        self,
        dynamic1d: Dynamic1d,
        param: DiscreteDynamic1dParam,
        load_from_cache=True,
        dump_to_cache=True,
    ):
        self.dynamic1d = dynamic1d
        self.param = param
        # jerks, discrete controls including pure 0.0 as choice.
        # doing so can reduces:
        # 1, the driving capability of the vehicle.
        # 2, the transition complexity of the graph.

        # Use discrete jerk control system
        self.dj = param.dj
        self.ds = param.ds
        self.dv = param.dv
        self.da = param.da
        self.dt = param.dt

        self.min_jid = math.ceil(self.dynamic1d.min_jerk / self.dj)
        self.max_jid = math.floor(self.dynamic1d.max_jerk / self.dj)

        assert (
            0.0 <= self.dynamic1d.min_speed < 1e-9
        ), "not supporting reversed driving, or speed lower bound."

        # sid = self.s_partition.get_index_from_value(s)
        self.s_partition = UniformPartition(0.0, self.ds)

        # vid = self.v_partition.get_index_from_value(v)
        self.v_partition = BVSPartition(0.0, self.dynamic1d.max_speed, self.dv)

        # aid =  self.a_partition.get_index_from_value(a)
        self.a_partition = BVSPartition(
            self.dynamic1d.min_acc, self.dynamic1d.max_acc, self.da
        )

        # build vid, aid, reachability graph. for all v, a, j.
        VidAid = Tuple[int, int]

        # The following cache only records forward reachable sets of [0, vid, aid]
        # Since:
        # [sid, vid, aid] \in [sid_now, vid_now, aid_now].FRS
        # <=> [sid-sid_now, vid, aid] \in [0, vid_now, aid_now].FRS
        #
        # What we need is to build up the full FRS for all [0, vid_now, aid_now].
        self.frs_of_0_vid_aid: Dict[VidAid, DiscreteDynamic1d.ForwardReachableSet] = {}

        # The following cache only records backward reachable sets of [0, vid, aid]
        # Since:
        # [sid, vid, aid] \in [sid_now, vid_now, aid_now].BRS
        # <=> [sid - sid_now, vid, aid] \in [0, vid_now, aid_now].BRS
        #
        # What we need is to build up the full BRS for all [0, vid_now, aid_now].
        # Since:
        # [sid, vid, aid] \in [0, vid_now, aid_now].BRS
        # <=> [0, vid, aid] \in [-sid, vid_now, aid_now].BRS.
        # Find all transitions by enumerating through starting state [0, vid, aid].
        self.brs_of_0_vid_aid: Dict[VidAid, DiscreteDynamic1d.BackwardReachableSet] = {}

        loaded_from_cache = False
        if load_from_cache:
            self.log("Trying load from cache.")
            loaded = False
            try:
                loaded = self._try_load_frs_brs_from_cache(DiscreteDynamic1d.CacheDir)
            except:
                pass

            if loaded:
                self.log("loaded from cache.")
            else:
                self.log("Load from cache not possible.")

            loaded_from_cache = loaded

        if not loaded_from_cache:
            self.log("Building frs brs.")
            self._build_frs_brs_from_scratch()

            if dump_to_cache:
                self.log("Trying dump to cache.")
                try:
                    self.try_dump_frs_brs_to_cache(DiscreteDynamic1d.CacheDir)
                    self.log("dumped to cache.")
                except:
                    self.log("dump to cache failed.")

    # TODO(huaiyuan)
    def head(self):
        return [self.dynamic1d, self.param]

    @staticmethod
    def log(string: str):
        if DiscreteDynamic1dLog:
            print("DiscreteDynamic1d:" + string, flush=True)

    def _build_frs_brs_from_scratch(self):
        VidAid = Tuple[int, int]
        self.frs_of_0_vid_aid: Dict[VidAid, DiscreteDynamic1d.ForwardReachableSet] = {}
        self.brs_of_0_vid_aid: Dict[VidAid, DiscreteDynamic1d.BackwardReachableSet] = {}
        self.log("building forward reachable set")

        for vid in range(len(self.v_partition)):
            self.log(
                "frs vid = %d, percent = %.2f"
                % (vid, vid / (len(self.v_partition) - 1))
            )
            for aid in range(len(self.a_partition)):
                from_state_id = 0, vid, aid
                self.frs_of_0_vid_aid[(vid, aid)] = {}
                frs = self.frs_of_0_vid_aid[(vid, aid)]

                for jid in range(self.min_jid, self.max_jid + 1):
                    reachable_states = self.calculate_forward_reachable_states(
                        from_state_id, jid
                    )
                    for state in reachable_states:
                        if state in frs:
                            frs[state].add(jid)
                        else:
                            frs[state] = self.JidSingleSet(jid)

        self.log("building backward reachable set")
        for vid in range(len(self.v_partition)):
            self.log(
                "brs vid = %d, percent = %.2f"
                % (vid, vid / (len(self.v_partition) - 1))
            )
            for aid in range(len(self.a_partition)):
                assert (vid, aid) in self.frs_of_0_vid_aid
                frs = self.frs_of_0_vid_aid[(vid, aid)]

                for to_state_id, jerk_set in frs.items():
                    # [0, vid, aid] reaches [to_sid, to_vid, to_aid] by control_id \in jerk_set
                    # <=> [-to_sid, vid, aid] reaches [0, to_vid, to_aid]  by control_id \in jerk_set
                    to_sid, to_vid, to_aid = to_state_id

                    if (to_vid, to_aid) not in self.brs_of_0_vid_aid:
                        self.brs_of_0_vid_aid[(to_vid, to_aid)] = set()

                    brs_of_to_state = self.brs_of_0_vid_aid[(to_vid, to_aid)]
                    brs_of_to_state.add((-to_sid, vid, aid))

    def _try_load_frs_brs_from_cache(self, cache_dir: str):
        for name in os.listdir(cache_dir):
            sn = len(DiscreteDynamic1d.CacheSuffix)
            if len(name) > sn and name[-sn:] == DiscreteDynamic1d.CacheSuffix:
                loaded = False
                with open(os.path.join(cache_dir, name), "rb") as fp:
                    try:
                        param = pickle.load(fp)
                        dynamic_1d = pickle.load(fp)

                        if self.dynamic1d == dynamic_1d and self.param == param:
                            self.frs_of_0_vid_aid = pickle.load(fp)
                            self.brs_of_0_vid_aid = pickle.load(fp)
                            loaded = True

                    except:
                        pass

                if loaded:
                    self.log("loaded from %s" % name)
                    return True
        return False

    def try_dump_frs_brs_to_cache(self, cache_dir: str):
        if not os.path.exists(cache_dir):
            os.mkdir(cache_dir)

        filename = str(time()) + DiscreteDynamic1d.CacheSuffix
        with open(os.path.join(cache_dir, filename), "wb") as fp:
            pickle.dump(self.param, fp)
            pickle.dump(self.dynamic1d, fp)
            pickle.dump(self.frs_of_0_vid_aid, fp)
            pickle.dump(self.brs_of_0_vid_aid, fp)
            self.log("dumped to %s" % filename)

    def calculate_forward_reachable_states(
        self, from_state_id: StateId, control_id: ControlId
    ) -> Set[StateId]:
        assert self.state_valid(from_state_id)
        jerk = self.control_id_to_jerk(control_id)
        sid, vid, aid = from_state_id
        # For the states in the box s_interval, v_interval, a_interval.
        # Do grid sampling and transition.

        s_interval = self.s_interval(sid)
        v_interval = self.v_interval(vid)
        a_interval = self.a_interval(aid)

        num_v_sample = (
            max(
                math.floor(
                    (v_interval.end - v_interval.begin) / self.param.dense_sample_dv
                ),
                1,
            )
            + 1
        )
        v_step = (v_interval.end - v_interval.begin) / (num_v_sample - 1)

        num_a_sample = (
            max(
                math.floor(
                    (a_interval.end - a_interval.begin) / self.param.dense_sample_da
                ),
                1,
            )
            + 1
        )
        a_step = (a_interval.end - a_interval.begin) / (num_a_sample - 1)

        reachable_states = set()
        # For any s:
        for i_v in range(num_v_sample):
            v = v_interval.begin + i_v * v_step
            if not v_interval.is_inside(v):
                continue

            for i_a in range(num_a_sample):
                a = a_interval.begin + i_a * a_step
                if not a_interval.is_inside(a):
                    continue

                next_s, next_v, next_a = self.dynamic1d.state_transition(
                    0.0, v, a, jerk, self.dt
                )
                next_vid = self.v_to_vid(next_v)
                next_aid = self.a_to_aid(next_a)
                assert next_vid is not None
                assert next_aid is not None

                # If from state s extends to a range of s, the transition leads to a span of
                # next states along s-axis.
                next_s_interval = copy.copy(s_interval)
                next_s_interval.begin += next_s
                next_s_interval.end += next_s

                for next_sid in range(
                    *self.s_partition.overlapping_index_range(next_s_interval)
                ):
                    reachable_states.add((next_sid, next_vid, next_aid))

        return reachable_states

    def s_to_sid(self, s: float) -> int:
        return self.s_partition.get_index_from_value(s)

    def v_to_vid(self, v: float) -> int:
        v = clamp(v, self.v_partition.min_value, self.v_partition.max_value)
        return self.v_partition.get_index_from_value(v)

    def a_to_aid(self, a: float) -> int:
        a = clamp(a, self.a_partition.min_value, self.a_partition.max_value)
        return self.a_partition.get_index_from_value(a)

    def jerk_round_to_control_id(self, jerk: float) -> int:
        jid = round(jerk / self.dj)
        return clamp(jid, self.min_jid, self.max_jid)

    def control_id_to_jerk(self, control_id: ControlId):
        return control_id * self.dj

    def jid_set_to_jerk_min_max(self, jid_set: JidSingleSet) -> Tuple[float, float]:
        return self.control_id_to_jerk(jid_set.min_jid), self.control_id_to_jerk(
            jid_set.max_jid
        )

    def all_control_ids(self):
        """
        :return: control ids from small to big
        """
        return list(range(self.min_jid, self.max_jid + 1))

    def all_vids(self):
        return list(range(len(self.v_partition)))

    def all_aids(self):
        return list(range(len(self.a_partition)))

    # === Make sure state valid before calling ===
    def s_interval(self, sid: int):
        """
        :param sid:
        :return:
        """
        return self.s_partition.get_interval(sid)

    def v_interval(self, vid: int):
        """
        :param vid: 0 <= vid < len(self.v_partition), checked by state valid
        :return:
        """
        return self.v_partition.get_interval(vid)

    def a_interval(self, aid: int):
        """
        :param aid: 0 <= aid < len(self.a_partition), checked by state valid
        :return:
        """
        return self.a_partition.get_interval(aid)

    def state_valid(self, state_id: StateId) -> bool:
        sid, vid, aid = state_id
        if type(sid) is not int:
            return False
        if type(vid) is not int:
            return False
        if type(aid) is not int:
            return False

        return 0 <= vid < len(self.v_partition) and 0 <= aid < len(self.a_partition)

    def control_valid(self, control_id: ControlId):
        return self.min_jid <= control_id <= self.max_jid

    def transition_controls(
        self, from_id: StateId, to_id: StateId
    ) -> Optional[JidSingleSet]:
        """
        :param from_id:
        :param to_id:
        :param control_id:
        :return:
        """
        sid, vid, aid = from_id
        to_sid, to_vid, to_aid = to_id
        # (sid, vid, aid) transits to (to_sid, to_vid, to_aid) by control id
        # <=> (0, vid, aid) transits to (to_sid-sid, to_vid, to_aid) by control id

        assert (vid, aid) in self.frs_of_0_vid_aid
        frs = self.frs_of_0_vid_aid[(vid, aid)]
        #
        to_state_0 = to_sid - sid, to_vid, to_aid

        if to_state_0 not in frs:
            return None

        return frs[to_state_0]

    def state_transition(
        self, state_id: StateId, control_id: ControlId
    ) -> Set[StateId]:
        """
        :param state_id:
        :param control_id:
        :return: a set of states that can be reached.
        """
        assert self.state_valid(state_id)
        sid, vid, aid = state_id
        assert (vid, aid) in self.frs_of_0_vid_aid

        frs = self.frs_of_0_vid_aid[(vid, aid)]

        result = set()

        for res_id, jerk_set in frs.items():
            if control_id in jerk_set:
                result.add((res_id[0] + sid, res_id[1], res_id[2]))

        return result

    def forward_reachable_states(self, state_id: StateId) -> Set[StateId]:
        assert self.state_valid(state_id)
        sid, vid, aid = state_id
        assert (vid, aid) in self.frs_of_0_vid_aid

        frs = self.frs_of_0_vid_aid[(vid, aid)]

        result = set()

        for res_id in frs.keys():
            result.add((res_id[0] + sid, res_id[1], res_id[2]))

        return result

    def back_transition(self, state_id: StateId, control_id: ControlId) -> Set[StateId]:
        """
        Find all back states such that .state_transition(back_state, control_id) contains state_id.
        :param state_id:
        :param control_id:
        :return:
        """
        assert self.state_valid(state_id)
        sid, vid, aid = state_id

        if (vid, aid) not in self.brs_of_0_vid_aid:
            return set()
        brs = self.brs_of_0_vid_aid[(vid, aid)]

        # (res_sid, res_vid, res_aid) transits to (0, vid, aid) by control id
        # <=> (res_sid+sid, res_vid, res_aid) transits to (sid, vid, aid) by control id
        # <=> (0, res_vid, res_aid) transits to (-res_sid, vid, aid) by control id

        result = set()

        for res_id in brs:
            res_vid_aid = res_id[1], res_id[2]
            assert res_vid_aid in self.frs_of_0_vid_aid

            frs = self.frs_of_0_vid_aid[res_vid_aid]
            assert (-res_id[0], vid, aid) in frs

            if control_id not in frs[(-res_id[0], vid, aid)]:
                continue

            result.add((res_id[0] + sid, res_id[1], res_id[2]))

        return result

    def backward_reachable_states(self, state_id: StateId) -> Set[StateId]:
        assert self.state_valid(state_id)
        sid, vid, aid = state_id

        if (vid, aid) not in self.brs_of_0_vid_aid:
            return set()
        brs = self.brs_of_0_vid_aid[(vid, aid)]

        # (res_sid, res_vid, res_aid) transits to (0, vid, aid) by control id
        # <=> (res_sid+sid, res_vid, res_aid) transits to (sid, vid, aid) by control id
        # <=> (0, res_vid, res_aid) transits to (-res_sid, vid, aid) by control id

        result = set()

        for res_id in brs:
            result.add((res_id[0] + sid, res_id[1], res_id[2]))

        return result

    def state_debug_info(self, state: StateId) -> str:
        if not self.state_valid(state):
            return "Invalid state."
        res = "StateId(%5d,%5d,%5d):\n" % state
        sid, vid, aid = state

        def interval_string(interval: Interval):
            return (
                ("(" if interval.begin_open else "[")
                + "%6.2f, %6.2f" % (interval.begin, interval.end)
                + (")" if interval.end_open else "]")
            )

        res += "  s ∈ " + interval_string(self.s_interval(sid)) + "\n"
        res += "  v ∈ " + interval_string(self.v_interval(vid)) + "\n"
        res += "  a ∈ " + interval_string(self.a_interval(aid)) + "\n"

        return res

    def state_name_string(self, state: StateId) -> str:
        return "(%d,%d,%d)" % state
