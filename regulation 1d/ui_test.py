import tkinter as tk
import random
import time


class MainUI():
    _one_frame_time_in_ms = 100

    def __init__(self):
        self.main_window = tk.Tk()

        self.canvas = tk.Canvas(self.main_window, width=200, height=200)

        self.canvas.grid(row=0, column=0, stick=tk.NSEW)

        self.canvas.create_line(0, 0, 100, 100)

        self.canvas.delete('all')

        self.main_window.bind("<KeyPress-e>", self.OnPressE)

        self.started = False

    def main_loop(self):
        assert not self.started, "UI cant be started twice"
        self.started = True
        self.main_window.after(ms=self._one_frame_time_in_ms, func=self.OnTimer)
        self.main_window.mainloop()


    def OnPressE(self, e: tk.Event):
        print('dsd')
        self.canvas.delete('all')
        self.canvas.create_line(0, 0, random.random() * 100, random.random() * 100)

    def OnTimer(self):
        _start_clock = time.perf_counter()

        print("timer")

        time_until_next_run = (_start_clock - time.perf_counter()) % self._one_frame_time_in_ms
        if time_until_next_run <= 0:
            time_until_next_run += self._one_frame_time_in_ms
        self.main_window.after(ms=int(time_until_next_run), func=self.OnTimer)


m = MainUI()
m.main_loop()