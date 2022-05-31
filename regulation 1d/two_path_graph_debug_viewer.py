import tkinter as tk
import random
import time, math

import path_based_graph
import vehicle
from path import PathBase
from path_based_graph import TwoPathGraph
from vehicle import Vehicle
from discrete_dynamic_1d import DiscreteDynamic1d
from typing import Optional, List, Set, Tuple, Literal, Callable
from utils import *
from collections import deque


class TwoPathGraphViewer:
    """
    GUI summary:
    Current utilities:
    1, Display the position of two vehicles given state

    2, Display the information of the given state.

    3, Display the unique id of the state.

    4, Show all possible forward/backward states:
        a, filtered by certain predicate: white list mode & black list mode.
        b, filtered by control: range of a1's jerk, range of a2's jerk

    structure of GUI:


    [canvas]           |[stateinfo]                |  [filter panel ]
    [Error logger]     |[id of the state] goto but |  goto selected button.

    update panel driven by action.

    update canvas driven by time.
    """

    _canvas_period_ms = 33
    _max_frame_count = 65535

    _display_vehicle_pos_ds = 0.1

    _display_pixel_per_meter_max = 50
    _display_pixel_per_meter_min = 5

    _display_path_length = 50.0
    _display_path_offset = -10.0
    _display_path_s_step = 0.5

    class GuiState:
        def __init__(self):
            self.graph_state: Optional[TwoPathGraph.StateId] = None

            self.graph: Optional[TwoPathGraph] = None

            # canvas world matrix:
            self.center_pos = 0.0, 0.0
            self.pixel_per_meter = 40

            # transition filters:
            self.a1_jerk_min: float = -float('inf')
            self.a1_jerk_max: float = float('inf')

            self.a2_jerk_min: float = -float('inf')
            self.a2_jerk_max: float = float('inf')

            self.filter_label_white_list: Set[int] = set()
            self.filter_label_black_list: Set[int] = set()
            self.transition_type: Literal["forward", "backward"] = "backward"

    class FilterPanel:
        _MaxStateListSize = 200

        class SinglePredFilter:
            FilterTrue = 0
            FilterFalse = 1
            FilterAny = 2

            def __init__(self, mw, row: int, name: str, pred_id: int,
                         gui_state: "TwoPathGraphViewer.GuiState",
                         state_change_call_back: Optional[Callable] = None):
                self.pred_id = pred_id
                self.state_change_call_back = state_change_call_back

                self.true_var = tk.IntVar()
                self.false_var = tk.IntVar()
                self.any_var = tk.IntVar()

                self.true_check = tk.Checkbutton(mw, onvalue=1, offvalue=0, variable=self.true_var, text="True",
                                                 command=self.on_select_true)
                self.false_check = tk.Checkbutton(mw, onvalue=1, offvalue=0, variable=self.false_var, text="False",
                                                  command=self.on_select_false)
                self.any_check = tk.Checkbutton(mw, onvalue=1, offvalue=0, variable=self.any_var, text="Any",
                                                command=self.on_select_any)

                label = tk.Label(mw, text=name[:15])
                label.grid(row=row, column=0)
                self.true_check.grid(row=row, column=1)
                self.false_check.grid(row=row, column=2)
                self.any_check.grid(row=row, column=3)

                self.gui_state = gui_state

                self.true_check.deselect()
                self.false_check.deselect()
                self.any_check.select()

            def update_from_gui_state(self):
                self.true_check.deselect()
                self.false_check.deselect()
                self.any_check.deselect()

                if self.pred_id in self.gui_state.filter_label_white_list:
                    self.true_check.select()
                elif self.pred_id in self.gui_state.filter_label_black_list:
                    self.false_check.select()
                else:
                    self.any_check.select()

            def on_select_true(self):
                """
                Change gui state, update from gui state.
                :return:
                """
                self.gui_state.filter_label_white_list.add(self.pred_id)
                self.gui_state.filter_label_black_list.discard(self.pred_id)

                self.update_from_gui_state()

                if self.state_change_call_back is not None:
                    self.state_change_call_back()

            def on_select_false(self):
                self.gui_state.filter_label_white_list.discard(self.pred_id)
                self.gui_state.filter_label_black_list.add(self.pred_id)

                self.update_from_gui_state()

                if self.state_change_call_back is not None:
                    self.state_change_call_back()

            def on_select_any(self):
                self.gui_state.filter_label_white_list.discard(self.pred_id)
                self.gui_state.filter_label_black_list.discard(self.pred_id)

                self.update_from_gui_state()

                if self.state_change_call_back is not None:
                    self.state_change_call_back()

        class JerkFilter:
            def __init__(self, mw, row: int, agent_id: int,
                         gui_state: "TwoPathGraphViewer.GuiState",
                         state_change_call_back: Optional[Callable] = None):
                self.gui_state = gui_state
                self.state_change_call_back = state_change_call_back

                assert agent_id in [1, 2]
                self.agent_id = agent_id

                label = tk.Label(mw, text="a%d jerk" % agent_id + " from")
                label.grid(row=row, column=0)

                self.min_entry = tk.Entry(mw)
                self.min_entry.bind("<Return>", self.on_user_change_input)
                self.min_entry.grid(row=row, column=1, sticky=tk.NSEW)

                label = tk.Label(mw, text="to")
                label.grid(row=row, column=2)

                self.max_entry = tk.Entry(mw)
                self.max_entry.bind("<Return>", self.on_user_change_input)
                self.max_entry.grid(row=row, column=3, sticky=tk.NSEW)

            def update_from_gui_state(self):
                if self.agent_id == 1:
                    min_v = self.gui_state.a1_jerk_min
                    max_v = self.gui_state.a1_jerk_max
                else:
                    min_v = self.gui_state.a2_jerk_min
                    max_v = self.gui_state.a2_jerk_max

                self.min_entry.delete(0, "end")
                self.max_entry.delete(0, "end")
                self.min_entry.insert(0, "%.3f" % min_v)
                self.max_entry.insert(0, "%.3f" % max_v)

            def on_user_change_input(self, e):
                try:
                    min_v = float(self.min_entry.get())
                    max_v = float(self.max_entry.get())

                    if min_v <= max_v:
                        if self.agent_id == 1:
                            self.gui_state.a1_jerk_min = min_v
                            self.gui_state.a1_jerk_max = max_v
                        elif self.agent_id == 2:
                            self.gui_state.a2_jerk_min = min_v
                            self.gui_state.a2_jerk_max = max_v
                except:
                    pass

                self.update_from_gui_state()
                if self.state_change_call_back is not None:
                    self.state_change_call_back()

        def __init__(self, father_window, gui_state: "TwoPathGraphViewer.GuiState"):
            assert gui_state.graph is not None
            self.father = father_window
            self.gui_state = gui_state

            self.frame = tk.Frame(self.father)

            self.title = tk.Label(self.frame, text="")
            self.title.grid(row=0, column=0, columnspan=4, sticky=tk.NSEW)

            self.state_list = tk.Listbox(self.frame, width=40)
            self.state_list.grid(row=1, column=0, columnspan=4, sticky=tk.NSEW)
            self.frame.rowconfigure(1, weight=1)
            self.frame.columnconfigure(0, weight=1)

            self.predicate_filters = []
            current_row = 2

            for i in range(len(gui_state.graph.predicates)):
                self.predicate_filters.append(
                    self.SinglePredFilter(self.frame, current_row, gui_state.graph.predicates[i].name(), i,
                                          self.gui_state, self.update_by_gui_state))
                current_row += 1

            self.jerk_filters = []
            for i in range(1, 3):
                self.jerk_filters.append(
                    self.JerkFilter(self.frame, current_row, i,
                                    self.gui_state, self.update_by_gui_state))
                current_row += 1

            self.transition_type_button = tk.Button(self.frame, text="", command=self.on_transition_type_change)
            self.transition_type_button.grid(row=current_row, column=0, columnspan=2, sticky=tk.NSEW)
            self.refresh_button = tk.Button(self.frame, text="apply", command=self.update_by_gui_state)
            self.refresh_button.grid(row=current_row, column=2, columnspan=2, sticky=tk.NSEW)

            self.update_by_gui_state()

        def on_transition_type_change(self):
            if self.gui_state.transition_type == "forward":
                self.gui_state.transition_type = "backward"
            else:
                self.gui_state.transition_type = "forward"

            self.update_by_gui_state()

        def update_by_gui_state(self):
            for filter_ in self.predicate_filters:
                filter_.update_from_gui_state()
            for filter_ in self.jerk_filters:
                filter_.update_from_gui_state()

            # The state list
            graph = self.gui_state.graph
            cur_state = self.gui_state.graph_state
            assert graph is not None
            assert cur_state is not None

            self.state_list.delete(0, "end")

            limit_breached = False
            set_too_large = False

            is_forward = self.gui_state.transition_type == "forward"
            a1 = graph.a1
            a2 = graph.a2
            s1_0, s2_0 = cur_state
            if is_forward:
                a1_reachable_states = a1.discrete_dynamic.forward_reachable_states(s1_0)
                a2_reachable_states = a2.discrete_dynamic.forward_reachable_states(s2_0)
            else:
                a1_reachable_states = a1.discrete_dynamic.backward_reachable_states(s1_0)
                a2_reachable_states = a2.discrete_dynamic.backward_reachable_states(s2_0)

            for s1_t in a1_reachable_states:
                if is_forward:
                    a1_jerk_set = graph.a1.discrete_dynamic.transition_controls(s1_0, s1_t)
                else:
                    a1_jerk_set = graph.a1.discrete_dynamic.transition_controls(s1_t, s1_0)

                a1_jerk_range = graph.a1.discrete_dynamic.jid_set_to_jerk_min_max(a1_jerk_set)
                a1_disjoint = a1_jerk_range[0] > self.gui_state.a1_jerk_max or a1_jerk_range[
                    1] < self.gui_state.a1_jerk_min
                if a1_disjoint:
                    continue

                for s2_t in a2_reachable_states:
                    if is_forward:
                        a2_jerk_set = graph.a2.discrete_dynamic.transition_controls(s2_0, s2_t)
                    else:
                        a2_jerk_set = graph.a2.discrete_dynamic.transition_controls(s2_t, s2_0)

                    a2_jerk_range = graph.a2.discrete_dynamic.jid_set_to_jerk_min_max(a2_jerk_set)
                    a2_disjoint = a2_jerk_range[0] > self.gui_state.a2_jerk_max or a2_jerk_range[
                        1] < self.gui_state.a2_jerk_min
                    if a2_disjoint:
                        continue

                    state_to_show = s1_t, s2_t
                    pred_all_pass = True
                    for true_pass_pred in self.gui_state.filter_label_white_list:
                        if not self.gui_state.graph.predicates[true_pass_pred].evaluate(state_to_show):
                            pred_all_pass = False
                            break
                    for false_pass_pred in self.gui_state.filter_label_black_list:
                        if self.gui_state.graph.predicates[false_pass_pred].evaluate(state_to_show):
                            pred_all_pass = False
                            break
                    if not pred_all_pass:
                        continue

                    control_name = " (%3.1f,%3.1f) , (%3.1f, %3.1f)" % (
                    a1_jerk_range[0], a1_jerk_range[1], a2_jerk_range[0], a2_jerk_range[1])
                    item = graph.state_name_string(state_to_show).ljust(37) + " | " + control_name
                    self.state_list.insert("end", item)
                    if self.state_list.size() >= self._MaxStateListSize:
                        limit_breached = True
                        break
                if limit_breached:
                    break

            if self.gui_state.transition_type == "forward":
                title_str = "forward transition"
            else:
                title_str = "backward transition"

            number_suffix = "+" if limit_breached else ""

            self.title.configure(text="=====%s(%d%s)=====" % (title_str, self.state_list.size(), number_suffix))

            # The transition type button
            if self.gui_state.transition_type == "forward":
                self.transition_type_button.configure(text="Show backward")
            else:
                self.transition_type_button.configure(text="Show forward")

        def get_selected_state(self) -> Optional[TwoPathGraph.StateId]:
            assert self.gui_state.graph is not None
            index = self.state_list.curselection()
            if len(index) != 1:
                return None

            item: str = str(self.state_list.get(index))
            state_string_end = item.find("|")
            assert state_string_end > 0

            return self.gui_state.graph.try_parse_state_str(item[:state_string_end])

        def destroy(self):
            self.frame.destroy()

    def __init__(self, graph: TwoPathGraph = None, graph_state: TwoPathGraph.StateId = None):
        # initialize state
        self.gui_state = TwoPathGraphViewer.GuiState()
        self.gui_state.graph = graph
        self.gui_state.graph_state = graph_state

        self.main_window = tk.Tk()
        # GUI summary:

        self.canvas = tk.Canvas(self.main_window, width=400, height=400, background="white", name="canvas")

        self.main_window.rowconfigure(0, weight=1)
        self.main_window.columnconfigure(0, weight=1)

        self.canvas.bind("<B1-Motion>", self.on_drag_canvas)
        self.canvas.bind("<Button-1>", self.on_canvas_left_click)
        self.canvas.bind("<ButtonRelease-1>", self.on_canvas_left_release)
        self.canvas.bind("<MouseWheel>", self.on_scroll_canvas)
        self.drag_canvas_start_pixel = None
        self.canvas.bind("<KeyPress-f>", self.on_canvas_focus)
        self.canvas.grid(row=0, column=0, sticky=tk.NSEW)

        self.state_info_text_box = tk.Text(self.main_window, width=40, name="state_info_text_box")
        self.state_info_text_box.insert(1.0, "(No graph)")
        self.state_info_text_box.configure(state='disabled')
        self.state_info_text_box.grid(row=0, column=1, columnspan=2, sticky=tk.NSEW)

        self.state_go_to_entry = tk.Entry(self.main_window, width=40, name="state_go_to_entry")
        self.state_go_to_entry.bind("<Return>", self.on_state_go)
        self.state_go_to_entry.insert(0, "((0,0,0),(0,0,0))")
        self.state_go_to_entry.grid(row=1, column=1, sticky=tk.EW)

        self.state_go_to_button = tk.Button(self.main_window, text=">", height=1, width=2, name="state_go_to_button")
        self.state_go_to_button.bind("<Button-1>", self.on_state_go)
        self.state_go_to_button.grid(row=1, column=2, sticky=tk.NE)

        self.log_bar = tk.Label(self.main_window, text="ready", height=1)
        self.log_bar.grid(row=1, column=0, sticky=tk.NW)
        self.log_bar.configure(text="ready")

        if self.gui_state.graph is not None:
            self.frs_brs_panel = TwoPathGraphViewer.FilterPanel(self.main_window, self.gui_state)
            self.frs_brs_panel.frame.grid(row=0, column=3, sticky=tk.NS)

            self.frs_brs_go_to_button = tk.Button(self.main_window, text="go to selected state",
                                                  name="frs_brs_go_to_button")
            self.frs_brs_go_to_button.bind("<Button-1>", self.on_frs_brs_state_go)
            self.frs_brs_go_to_button.grid(row=1, column=3, sticky=tk.NSEW)

        # go_to_state retraction:
        self.state_history = deque()
        self.main_window.bind("<Control-z>", self.on_ctrl_z)

        self.frame_count = 0
        self.started = False

    def main_loop(self):
        assert not self.started, "GUI cant be started twice"
        self.started = True
        self.main_window.after(ms=self._canvas_period_ms, func=self.on_canvas_timer)
        self.main_window.after(ms=self._canvas_period_ms, func=self.update_panel)
        self.main_window.mainloop()

    def on_ctrl_z(self, e):
        if self.gui_state.graph is None:
            return
        if len(self.state_history) == 0:
            self.log_to_gui("No history")
            return
        last = self.state_history.pop()
        if last is None or not self.gui_state.graph.state_valid(last):
            self.log_to_gui("No history")
            return

        self.gui_state.graph_state = last

        self.log_to_gui("Returned to:" + str(last))

        self.update_panel()

    def log_to_gui(self, string: str):
        self.log_bar.configure(text=string)

    def go_to_state(self, state_id: path_based_graph.TwoPathGraph.StateId):
        if self.gui_state.graph is None:
            self.log_to_gui("No graph")

        if not self.gui_state.graph.state_valid(state_id):
            self.log_to_gui("Invalid state:" + str(state_id))
        else:
            self.state_history.append(self.gui_state.graph_state)
            if len(self.state_history) > 100:
                self.state_history.popleft()

            self.gui_state.graph_state = state_id

        self.log_to_gui("Navigated to:" + self.gui_state.graph.state_name_string(state_id))

        self.update_panel()

    def on_frs_brs_state_go(self, e: tk.Event):
        assert self.gui_state.graph is not None

        new_state_id = self.frs_brs_panel.get_selected_state()
        if new_state_id is None:
            self.log_to_gui("No selected state")
        else:
            self.go_to_state(new_state_id)

    def on_state_go(self, e: tk.Event):
        if self.gui_state.graph is None:
            self.log_to_gui("No graph ")

        state_id_str = self.state_go_to_entry.get()
        # ((1,2,3),(4,5,6))

        new_state_id = self.gui_state.graph.try_parse_state_str(state_id_str)
        if new_state_id is None:
            self.log_to_gui("Unrecognized input: " + state_id_str)
        else:
            self.go_to_state(new_state_id)

    def on_canvas_left_click(self, e):
        self.canvas.focus_set()

        self.drag_canvas_start_pixel = None

    def on_canvas_left_release(self, e):
        self.drag_canvas_start_pixel = None

    def on_drag_canvas(self, e):
        if self.drag_canvas_start_pixel is None:
            self.drag_canvas_start_pixel = e.x, e.y
        else:
            dx = e.x - self.drag_canvas_start_pixel[0]
            dy = e.y - self.drag_canvas_start_pixel[1]
            self.gui_state.center_pos = self.gui_state.center_pos[0] - dx * 1.0 / self.gui_state.pixel_per_meter, \
                                        self.gui_state.center_pos[1] + dy * 1.0 / self.gui_state.pixel_per_meter
            self.drag_canvas_start_pixel = e.x, e.y

    def on_scroll_canvas(self, e):
        tick = round(abs(e.delta) / 120.)
        gain = 1.25 if e.delta > 0 else 0.8

        for _ in range(tick):
            self.gui_state.pixel_per_meter *= gain
        self.gui_state.pixel_per_meter = \
            clamp(self.gui_state.pixel_per_meter, self._display_pixel_per_meter_min, self._display_pixel_per_meter_max)

    def on_canvas_focus(self, e):
        if self.gui_state.graph is None or self.gui_state.graph_state is None:
            return
        a1_state, _ = self.gui_state.graph_state

        a1_s_min = self.gui_state.graph.a1.discrete_dynamic.s_interval(a1_state[2]).begin
        a1_v_state = self.gui_state.graph.a1.path.get_state(a1_s_min, 0.0, 0.0)
        a1_rac = self.gui_state.graph.a1.vehicle.get_rac(a1_v_state)

        self.gui_state.center_pos = a1_rac.x, a1_rac.y

    def on_canvas_timer(self):
        start_time = time.perf_counter()

        self.update_canvas()

        self.frame_count = (self.frame_count + 1) % self._max_frame_count
        time_until_next_run = (start_time - time.perf_counter()) % self._canvas_period_ms
        if time_until_next_run <= 0:
            # skip frame
            time_until_next_run += self._canvas_period_ms

        self.main_window.after(ms=int(time_until_next_run), func=self.on_canvas_timer)

    def pos2canvas(self, x, y) -> Tuple[int, int]:
        xc, yc = self.gui_state.center_pos
        canvas_x = (x - xc) * self.gui_state.pixel_per_meter + 0.5 * self.canvas.winfo_width()
        canvas_y = -(y - yc) * self.gui_state.pixel_per_meter + 0.5 * self.canvas.winfo_height()
        return round(canvas_x), round(canvas_y)

    def update_canvas(self):
        self.canvas.delete("all")

        # alive indicator
        def draw_alive_ind():
            ticks_of_one_cycle = 3000.0 / self._canvas_period_ms
            ticks_of_one_cycle = max(ticks_of_one_cycle, 1)
            ratio = (self.frame_count % ticks_of_one_cycle) / (1.0 * ticks_of_one_cycle)
            angle0 = 2 * ratio * math.pi
            angle1 = angle0 + math.pi / 1.5
            angle2 = angle1 + math.pi / 1.5

            radius = 6
            xc = 7
            yc = 7

            x0 = xc + radius * math.cos(angle0)
            y0 = yc + radius * math.sin(angle0)
            x1 = xc + radius * math.cos(angle1)
            y1 = yc + radius * math.sin(angle1)
            x2 = xc + radius * math.cos(angle2)
            y2 = yc + radius * math.sin(angle2)

            self.canvas.create_polygon(x0, y0, x1, y1, x2, y2, fill="blue")

        draw_alive_ind()

        # transition from physical world to canvas world.

        def draw_single_vehicle(path: PathBase,
                                vehicle_: Vehicle,
                                dynamic: DiscreteDynamic1d,
                                state_id: DiscreteDynamic1d.StateId,
                                color: str):
            s_range = dynamic.s_interval(state_id[0])
            assert s_range
            num_boxes = max(int((s_range.end - s_range.begin) / self._display_vehicle_pos_ds), 1) + 1

            ds = (s_range.end - s_range.begin) / (num_boxes - 1)

            for i in range(num_boxes):
                s = s_range.begin + ds * i
                box = vehicle_.box(path.get_state(s, 0.0, 0.0))
                corners = box.four_corners()
                corners_in_canvas = [self.pos2canvas(pt.x, pt.y) for pt in corners]
                self.canvas.create_polygon(*corners_in_canvas, fill=color)

            # self.canvas.create_text()

        def draw_vehicles():
            if self.gui_state.graph is None or self.gui_state.graph_state is None:
                return
            a1_sva_id, a2_sva_id = self.gui_state.graph_state

            a1 = self.gui_state.graph.a1
            a2 = self.gui_state.graph.a2
            draw_single_vehicle(a1.path,
                                a1.vehicle,
                                a1.discrete_dynamic,
                                a1_sva_id,
                                "#cc0000")

            draw_single_vehicle(a2.path,
                                a2.vehicle,
                                a2.discrete_dynamic,
                                a2_sva_id,
                                "#0000cc")

        def draw_single_path(path: PathBase,
                             dynamic: DiscreteDynamic1d,
                             state_id: DiscreteDynamic1d.StateId,
                             color: str):
            current_sid = state_id[0]
            current_s = dynamic.s_interval(current_sid).begin

            _display_path_length = 100.0
            _display_path_offset = -10.0
            _display_path_s_step = 0.5

            s_start = current_s + self._display_path_offset
            num_s = math.floor((self._display_path_length - self._display_path_offset) / self._display_path_s_step) + 1

            polyline_canvas_pts = []

            for i in range(num_s):
                s = i * self._display_path_s_step + s_start
                state = path.get_state(s, 0.0, 0.0)
                polyline_canvas_pts.append(self.pos2canvas(state[vehicle.X_ID], state[vehicle.Y_ID]))

            self.canvas.create_line(*polyline_canvas_pts, fill=color)

        def draw_paths():
            if self.gui_state.graph is None or self.gui_state.graph_state is None:
                return
            a1_sva_id, a2_sva_id = self.gui_state.graph_state

            a1 = self.gui_state.graph.a1
            a2 = self.gui_state.graph.a2
            draw_single_path(a1.path, a1.discrete_dynamic, a1_sva_id, "#cc9999")
            draw_single_path(a2.path, a2.discrete_dynamic, a2_sva_id, "#9999cc")

        draw_paths()
        draw_vehicles()

    def update_panel(self):
        """
        For all panels, update their content.
        :return:
        """
        # State debug info
        self.state_info_text_box.configure(state='normal')
        self.state_info_text_box.delete(1.0, 'end')

        if self.gui_state.graph is None:
            self.state_info_text_box.insert(1.0, "(No graph)")
            return

        if self.gui_state.graph_state is None:
            self.state_info_text_box.insert(1.0, "(No state)")
            return

        state_debug_info = self.gui_state.graph.state_debug_info(self.gui_state.graph_state)
        self.state_info_text_box.insert(1.0, state_debug_info)
        self.state_info_text_box.configure(state='disabled')

        self.state_go_to_entry.delete(0, "end")
        self.state_go_to_entry.insert(0, self.gui_state.graph.state_name_string(self.gui_state.graph_state))

        self.frs_brs_panel.update_by_gui_state()
