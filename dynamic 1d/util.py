from typing import List, Tuple
from geometry import *
import math


def clamp(v, min_v, max_v):
    return min(max_v, max(min_v, v))


class Dynamic1d:
    """
    Dynamic model is a follows:
    s_dot = v
    v_dot = a (v = clamp(v, min_speed, max_speed))
    a_dot = j (a = clamp(a, min_acc, max_acc))

    j = clamp(j_command, min_jerk, max_jerk)
    """
    StateType = Tuple[float, float, float]

    def __init__(self):
        self.max_jerk = 5.0  # 3.0 comfortable
        self.min_jerk = -8.0  # -4.0 comfortable
        self.max_acc = 2.0  # 1.0 comfortable
        self.min_acc = -6.0  # -3.0 comfortable
        self.min_speed = 0.0  #
        self.max_speed = 22.22  # 80 kph

    def clamp_state(self, s, v, a) -> StateType:
        return s, clamp(v, self.min_speed, self.max_speed), clamp(a, self.min_acc, self.max_acc)




    class _AnalyticalTypes:
        v_max_a_max = 0
        v_max_a_free = 1
        v_min_a_min = 2
        v_min_a_free = 3
        v_free_a_max = 4
        v_free_a_min = 5
        v_free_a_free = 6

    _EventType = List

    def _initial_type(self, v,a, jerk) -> int:
        AnaType = Dynamic1d._AnalyticalTypes

        # In the immediate future, whether there is considerable time such that
        # velocity will stay at self.max_speed.
        velocity_upper_clamped = v >= self.max_speed and (a > 0.0 or (a == 0.0 and jerk >= 0.0))

        # In the immediate future, whether there is considerable time such that
        # velocity will stay at self.min_speed.
        velocity_lower_clamped = v <= self.min_speed and (a < 0.0 or (a == 0.0 and jerk <= 0.0))

        # In the immediate future, whether there is considerable time such that
        # acc will stay at self.max_acc.
        acc_upper_clamped = a >= self.max_acc and jerk >= 0.0

        # In the immediate future, whether there is considerable time such that
        # acc will stay at self.min_acc.
        acc_lower_clamped = a <= self.min_acc and jerk <= 0.0

        if velocity_upper_clamped:
            if acc_upper_clamped:
                return AnaType.v_max_a_max
            else:
                return AnaType.v_max_a_free
        if velocity_lower_clamped:
            if acc_lower_clamped:
                return AnaType.v_min_a_min
            else:
                return AnaType.v_min_a_free

        if acc_upper_clamped:
            return AnaType.v_free_a_max
        if acc_lower_clamped:
            return AnaType.v_free_a_min
        return AnaType.v_free_a_free


    def _analytical_dynamics(self, analytical_type: int, s, v, a, jerk, t)->StateType:
        AnaType = Dynamic1d._AnalyticalTypes
        if analytical_type == AnaType.v_max_a_max:
            return s + self.max_speed * t, self.max_speed, self.max_acc
        elif analytical_type == AnaType.v_max_a_free:
            return s + self.max_speed * t, self.max_speed, a + jerk * t
        elif analytical_type == AnaType.v_min_a_min:
            return s + self.min_speed * t, self.min_speed, self.min_acc
        elif analytical_type == AnaType.v_min_a_free:
            return s + self.min_speed * t, self.min_speed, a + jerk * t
        elif analytical_type == AnaType.v_free_a_max:
            return s + v * t + 0.5 * self.max_acc * (t ** 2), v + self.max_acc * t, self.max_acc
        elif analytical_type == AnaType.v_free_a_min:
            return s + v * t + 0.5 * self.min_acc * (t ** 2), v + self.min_acc * t, self.min_acc
        elif analytical_type == AnaType.v_free_a_free:
            return s + v * t + 0.5 * a * (t ** 2) + (1.0 / 6.0) * jerk * (t ** 3), v + a * t + 0.5 * jerk * (t ** 2), a + jerk * t
        else:
            assert False, "Invalid type"

    def _next_event(self, analytical_type: int, v, a, jerk) -> _EventType:
        AnaType = Dynamic1d._AnalyticalTypes
        Event = Dynamic1d._EventType
        next_event: Event = [kInf, analytical_type]
        def update_event(event_res: Event, new_event: Event):
            # Reduce numerical error.
            new_event[0] = max(0.0, new_event[0])
            if new_event[0] < event_res[0]:
                # new event happens earlier than current.
                event_res[0] = new_event[0]
                event_res[1] = new_event[1]

        if analytical_type == AnaType.v_max_a_max:
            pass
        elif analytical_type == AnaType.v_max_a_free:
            if jerk > 0.0:
                update_event(next_event, [(self.max_acc - a) / jerk, AnaType.v_max_a_max])
            if jerk < 0.0:
                update_event(next_event, [(0.0 - a) / jerk, AnaType.v_free_a_free])
        elif analytical_type == AnaType.v_min_a_min:
            pass
        elif analytical_type == AnaType.v_min_a_free:
            if jerk < 0.0:
                update_event(next_event, [(self.min_acc - a) / jerk, AnaType.v_min_a_min])
            if jerk > 0.0:
                update_event(next_event, [(0.0 - a) / jerk, AnaType.v_free_a_free])
        elif analytical_type == AnaType.v_free_a_max:
            update_event(next_event, [(self.max_speed - v) / self.max_acc, AnaType.v_max_a_max])
        elif analytical_type == AnaType.v_free_a_min:
            update_event(next_event, [(self.min_speed - v) / self.min_acc, AnaType.v_min_a_min])
        elif analytical_type == AnaType.v_free_a_free:
            if jerk == 0.0:
                if a > 0.0:
                    update_event(next_event, [(self.max_speed - v) / a, AnaType.v_max_a_free])
                if a < 0.0:
                    update_event(next_event, [(self.min_speed - v) / a, AnaType.v_min_a_free])
            elif jerk > 0.0:
                update_event(next_event, [(self.max_acc - a) / jerk, AnaType.v_free_a_max])

                # speed upper clamp: v + a * t + 0.5 * jerk * (t ** 2) = v_max
                delta = a ** 2 + 2 * jerk * (self.max_speed - v)
                update_event(next_event, [(math.sqrt(delta) - a) / jerk, AnaType.v_max_a_free])

                # speed lower clamp: v + a * t + 0.5 * jerk * (t ** 2) = v_min
                delta = a ** 2 + 2 * jerk * (self.min_speed - v)
                if a < -kEps and delta > kEps:
                    update_event(next_event, [(-math.sqrt(delta) - a) / jerk, AnaType.v_min_a_free])

            else:  # jerk < 0.0
                # Acc lower clamp:
                update_event(next_event, [(self.min_acc - a) / jerk, AnaType.v_free_a_min])

                # speed lower clamp: v + a * t + 0.5 * jerk * (t ** 2) = v_min
                delta = a ** 2 + 2 * jerk * (self.min_speed - v)
                update_event(next_event, [(-math.sqrt(delta) - a) / jerk, AnaType.v_min_a_free])

                # speed upper clamp: v + a * t + 0.5 * jerk * (t ** 2) = v_max
                delta = a ** 2 + 2 * jerk * (self.max_speed - v)
                if delta > kEps and a > kEps:
                    update_event(next_event, [(math.sqrt(delta) - a) / jerk, AnaType.v_max_a_free])
        else:
            assert False, "Invalid type"

        return next_event

    def state_transition(self, s, v, a, j_command, delta_t) -> StateType:
        """
        Transit longitudinal state using constant jerk control.
        The function returns analytical result.
        :param s:
        :param v:
        :param a:
        :param j_command: time of constant jerk command motion.
        :param delta_t: sampling hint for integration.
        :return:
        """
        jerk = clamp(j_command, self.min_jerk, self.max_jerk)
        s,v,a = self.clamp_state(s,v,a)
        ana_type = self._initial_type(v, a, jerk)

        while delta_t > 0.0:
            next_event_time, next_ana_type = self._next_event(ana_type, v, a, jerk)

            if next_event_time < delta_t:
                s, v, a = self._analytical_dynamics(ana_type, s, v, a, jerk, next_event_time)
                delta_t -= next_event_time
                ana_type = next_ana_type
            else:
                s, v, a = self._analytical_dynamics(ana_type, s, v, a, jerk, delta_t)
                break

        return s,v,a


    def state_transition_numerical(self, s, v, a, j_command, delta_t, dt_hint)->StateType:
        """
        Transit longitudinal state using constant jerk command.
        The function returns numerical result.
        :param s:
        :param v:
        :param a:
        :param j_command:
        :param delta_t: time of constant jerk command motion.
        :param dt_hint: sampling hint for integration.
        :return:
        """
        assert delta_t > 0
        assert dt_hint > 0
        s, v, a = self.clamp_state(s, v, a)

        n = max(1, math.floor(delta_t / dt_hint))
        dt = delta_t / n

        for i in range(n):
            jerk = clamp(j_command, self.min_jerk, self.max_jerk)

            next_s = s + v * dt
            next_v = clamp(v + a * dt, self.min_speed, self.max_speed)
            next_a = clamp(a + jerk * dt, self.min_acc, self.max_acc)

            s, v, a = next_s, next_v, next_a

        return s, v, a
