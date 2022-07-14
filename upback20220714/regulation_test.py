#!/usr/bin/env python3
"""
Copyright @2022 QCraft AI Inc. All rights reserved.
Authors: huaiyuan@qcraft.ai (Huaiyuan Teng)

Unit test of regulation
"""
import random

from experimental.users.huaiyuan.regulated_safety.two_path_graph import *
from experimental.users.huaiyuan.regulated_safety.utils import *


def test_regulation_by_table_1():
    r = RegulationByTableReal(0, 10)

    # Acc reg
    r.set_reg(((0, 0, 0), (0, 0, 0)), {0, 1, 2, 3})
    f = r.forbidden_jids(((0, 0, 0), (0, 0, 0)))
    assert (
        r.regulation_at_state[((0, 0, 0), (0, 0, 0))][0] == RegulationByTableReal.AccReg
    )
    for jid in range(11):
        if jid >= 4:
            assert r.can_do(((0, 0, 0), (0, 0, 0)), jid)
            assert jid not in f
        else:
            assert jid in f

    # Brake reg
    r.set_reg(((1, 0, 0), (0, 0, 0)), {6, 7, 8, 9, 10})
    f = r.forbidden_jids(((1, 0, 0), (0, 0, 0)))
    assert (
        r.regulation_at_state[((1, 0, 0), (0, 0, 0))][0]
        == RegulationByTableReal.BrakeReg
    )
    for jid in range(11):
        if jid <= 5:
            assert r.can_do(((1, 0, 0), (0, 0, 0)), jid)
            assert jid not in f
        else:
            assert jid in f

    # General reg
    forb = {1, 2, 5, 7}
    r.set_reg(((2, 0, 0), (0, 0, 0)), forb)
    assert (
        r.regulation_at_state[((2, 0, 0), (0, 0, 0))][0]
        == RegulationByTableReal.GeneralReg
    )
    f = r.forbidden_jids(((2, 0, 0), (0, 0, 0)))
    assert f == forb


def test_regulation_by_table_2():
    # Compare same with a general regulation problem
    jid_min = -3
    jid_max = 2

    def random_state_id():
        rint = lambda: random.randint(-100, 100)
        return (rint(), rint(), rint()), (rint(), rint(), rint())

    def random_jid():
        return clamp(random.randint(jid_min, jid_max), jid_min, jid_max)

    def random_forbidden_jids():
        count = random.randint(1, 5)
        res = set()
        for _ in range(count):
            res.add(random_jid())
        return res

    reg_by_table = RegulationByTableReal(jid_min, jid_max)
    brute_force_table: Dict[TwoPathGraph.StateId, Set[int]] = {}

    def check_same():
        for state_id, forbiden_jids in brute_force_table.items():
            assert forbiden_jids == reg_by_table.forbidden_jids(state_id)

        for state_id in reg_by_table.regulation_at_state.keys():
            assert state_id in brute_force_table
            assert brute_force_table[state_id] == reg_by_table.forbidden_jids(state_id)

    # Generate a bunch of random forbidden actions:
    for i in range(1000):
        s = random_state_id()
        f = random_forbidden_jids()
        brute_force_table[s] = f
        reg_by_table.set_reg(s, f)

    check_same()

    class RegNoAccelerate(RegulationBase):
        def can_do(
            self,
            state_id: "TwoPathGraph.StateId",
            control_id: DiscreteDynamic1d.ControlId,
        ) -> bool:
            return control_id <= 0

    reg_no_acc = RegNoAccelerate()

    for k, forb in brute_force_table.items():
        # union
        forb -= {-3, -2, -1, 0}

    reg_by_table.union_in_place(reg_no_acc)

    check_same()


def test_regulation_by_table_3():
    r = RegulationByTableReal(0, 5)


if __name__ == "__main__":
    random.seed(0)
    test_regulation_by_table_1()
    test_regulation_by_table_2()
