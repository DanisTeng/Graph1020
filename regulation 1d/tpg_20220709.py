from two_path_graph_debug_viewer import TwoPathGraphViewer
from two_path_graph import *
from path import StraightPath
from geometry import *
import time


class Tpg20220709(TwoPathGraphEx):
    """
    mutation: angle of a2
    """

    A2RegBasicRight = "a2_basic_right"
    A1RegBasicRight = "a1_basic_right"
    A2DefendA1B = "a2_df_a1_br"

    Tpg20220709Prefix = "tpg_20220709_"

    def __init__(self):
        # path
        single_path_graph_param = SinglePathParam()
        single_path_graph_param.dd1d_param.da = 2.0
        single_path_graph_param.dd1d_param.dv = 1.0
        single_path_graph_param.dd1d_param.dj = 4.0
        single_path_graph_param.dd1d_param.ds = 0.25

        # vehicle
        vehicle = Vehicle()
        vehicle.max_lng_speed = 12.0

        # Paths
        a1_path = StraightPath(Vec2(0.0, 0.0), Vec2.form_unit(1.57))

        # a2_angle = 1.42
        self.a2_angle = 0.0
        self.a2_angle = Angle.normalize(self.a2_angle)
        a2_path = StraightPath(Vec2(0.0, 0.0), Vec2.form_unit(self.a2_angle))

        a1 = SingleAgentMeta(a1_path, vehicle, single_path_graph_param)
        a2 = SingleAgentMeta(a2_path, vehicle, single_path_graph_param)

        # common predicates
        super(Tpg20220709, self).__init__(a1, a2)

        self.try_load_from_cache(self.CacheDir, self.prefix())

        if not self.has_regulation(self.A2RegBasicRight):
            a2_br = RegulationByBrakeRight(a2.discrete_dynamic, False, 0.0, -4.0, self.A2RegBasicRight)
            self.add_regulation(a2_br)

        if not self.has_regulation(self.A1RegBasicRight):
            a1_br = RegulationByBrakeRight(a1.discrete_dynamic, True, -3.0, -6.0, self.A1RegBasicRight)
            self.add_regulation(a1_br)

        if not self.has_regulation(self.A1FinalRegName):
            a1_fn = RegulationByBrakeRight(a1.discrete_dynamic, True, -3.0, -6.0, self.A1FinalRegName)
            self.add_regulation(a1_fn)

        # CONTI

        if not self.has_regulation(self.A2DefendA1B):
            self.log("calculating: " + self.A2DefendA1B)
            reg_a2_df_a1b, _ = self.a2_defend_a1(self.regulation(self.A1RegBasicRight))
            reg_a2_df_a1b.set_name(self.A2DefendA1B)
            self.add_regulation(reg_a2_df_a1b)

        self.may_dump_to_cache(self.CacheDir, self.prefix(), None)

    def prefix(self):
        return self.Tpg20220709Prefix + "a2_%.4f_" % self.a2_angle


if __name__ == "__main__":
    t0 = time.perf_counter()
    tpg = Tpg20220709()
    print(time.perf_counter() - t0, " seconds elapsed.")
    TwoPathGraphViewer(tpg, ((0, 2, 0), (0, 2, 0))).main_loop()
