import sys
import time
import numpy as np
import pandas as pd

from PyQt6 import QtCore, QtWidgets
import pyqtgraph as pg


class AbsoluteTimeAxis(pg.AxisItem):
    """
    Axis that displays tick labels as absolute time by adding an offset.
    We still plot data in relative time (t_rel = t - t_now), so 'now' stays at x=0.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.t_offset = 0.0  # absolute time corresponding to x=0 (i.e., current t_now)

    def set_offset(self, t_now: float):
        self.t_offset = float(t_now)

    def tickStrings(self, values, scale, spacing):
        # values are tick positions in relative-time coordinates (seconds)
        out = []
        for v in values:
            ta = self.t_offset + v
            out.append(f"{ta:.3f}")  # show enough precision to see updates
        return out


class PlaybackSource:
    """
    CSV playback for telemetry with look-ahead support.
    Requires columns: time, surge, sway, heave
    """
    def __init__(self, csv_path: str):
        df = pd.read_csv(csv_path)

        req = ["time", "surge", "sway", "heave"]
        for c in req:
            if c not in df.columns:
                raise ValueError(f"CSV missing required column: {c}")

        self.t = df["time"].to_numpy(dtype=float)
        self.surge = df["surge"].to_numpy(dtype=float)
        self.sway  = df["sway"].to_numpy(dtype=float)
        self.heave = df["heave"].to_numpy(dtype=float)

        n = min(len(self.t), len(self.surge), len(self.sway), len(self.heave))
        self.t = self.t[:n]
        self.surge = self.surge[:n]
        self.sway  = self.sway[:n]
        self.heave = self.heave[:n]

        if np.any(np.diff(self.t) <= 0):
            raise ValueError("time must be strictly increasing")

        self.i = 0  # current playback index

    def reset(self):
        self.i = 0

    def peek_time0(self) -> float:
        return float(self.t[0])

    def has_next(self) -> bool:
        return self.i < len(self.t)

    def advance_to_time(self, t_play: float):
        """
        Move internal index forward so that t[i] is the first sample with t[i] > t_play.
        Return the last sample at or before t_play (t_now), or None if not advanced yet.
        """
        last = None
        while self.i < len(self.t) and float(self.t[self.i]) <= t_play:
            last = (float(self.t[self.i]),
                    float(self.surge[self.i]),
                    float(self.sway[self.i]),
                    float(self.heave[self.i]))
            self.i += 1
        return last

    def window_indices(self, t_center: float, past: float, future: float):
        """
        Return slice indices [i0:i1] for time window [t_center - past, t_center + future]
        Uses binary search for speed.
        """
        t0 = t_center - past
        t1 = t_center + future
        i0 = int(np.searchsorted(self.t, t0, side="left"))
        i1 = int(np.searchsorted(self.t, t1, side="right"))
        i0 = max(0, min(i0, len(self.t)))
        i1 = max(0, min(i1, len(self.t)))
        return i0, i1


class FutureViewer(QtWidgets.QMainWindow):
    def __init__(self, source: PlaybackSource, *,
                 past_sec=2.5, future_sec=2.5,
                 realtime_factor=1.0,
                 ylims=None):
        super().__init__()
        self.setWindowTitle("Telemetry Viewer (Now Center + Future)")

        self.source = source
        self.past_sec = float(past_sec)
        self.future_sec = float(future_sec)
        self.realtime_factor = float(realtime_factor)

        self.ylims = ylims

        self.t0_data = None
        self.t0_wall = None
        self.t_now = None

        self._build_ui()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(16)  # ~60 FPS UI tick

    def _build_ui(self):
        cw = QtWidgets.QWidget()
        self.setCentralWidget(cw)

        layout = QtWidgets.QGridLayout(cw)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setVerticalSpacing(10)

        pg.setConfigOptions(antialias=True)

        # --- IMPORTANT CHANGE: attach custom axes via PlotItem (reliable across pyqtgraph versions)
        self.ax_surge = AbsoluteTimeAxis(orientation="bottom")
        self.ax_sway  = AbsoluteTimeAxis(orientation="bottom")
        self.ax_heave = AbsoluteTimeAxis(orientation="bottom")

        pi_surge = pg.PlotItem(axisItems={"bottom": self.ax_surge}, title="Surge")
        pi_sway  = pg.PlotItem(axisItems={"bottom": self.ax_sway},  title="Sway")
        pi_heave = pg.PlotItem(axisItems={"bottom": self.ax_heave}, title="Heave")

        self.p_surge = pg.PlotWidget(plotItem=pi_surge)
        self.p_sway  = pg.PlotWidget(plotItem=pi_sway)
        self.p_heave = pg.PlotWidget(plotItem=pi_heave)

        for p in (self.p_surge, self.p_sway, self.p_heave):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel("bottom", "Time", units="s")  # absolute-time tick labels
            p.setXRange(-self.past_sec, self.future_sec, padding=0)

        self.p_surge.setLabel("left", "m/s²")
        self.p_sway.setLabel("left", "m/s²")
        self.p_heave.setLabel("left", "m/s²")

        layout.addWidget(self.p_surge, 0, 0)
        layout.addWidget(self.p_sway,  1, 0)
        layout.addWidget(self.p_heave, 2, 0)

        # Curves
        self.c_surge = self.p_surge.plot([], [])
        self.c_sway  = self.p_sway.plot([], [])
        self.c_heave = self.p_heave.plot([], [])

        # "Now" vertical line fixed at x=0 (center)
        self.now_surge = pg.InfiniteLine(pos=0.0, angle=90, movable=False)
        self.now_sway  = pg.InfiniteLine(pos=0.0, angle=90, movable=False)
        self.now_heave = pg.InfiniteLine(pos=0.0, angle=90, movable=False)
        self.p_surge.addItem(self.now_surge)
        self.p_sway.addItem(self.now_sway)
        self.p_heave.addItem(self.now_heave)

        # Time label at x=0 (shows absolute time)
        self.time_label_surge = pg.TextItem(anchor=(0.5, 1.0))
        self.time_label_sway  = pg.TextItem(anchor=(0.5, 1.0))
        self.time_label_heave = pg.TextItem(anchor=(0.5, 1.0))
        self.p_surge.addItem(self.time_label_surge)
        self.p_sway.addItem(self.time_label_sway)
        self.p_heave.addItem(self.time_label_heave)

        # Current markers at x=0
        self.m_surge = pg.ScatterPlotItem(size=8)
        self.m_sway  = pg.ScatterPlotItem(size=8)
        self.m_heave = pg.ScatterPlotItem(size=8)
        self.p_surge.addItem(self.m_surge)
        self.p_sway.addItem(self.m_sway)
        self.p_heave.addItem(self.m_heave)

        # Controls
        row = QtWidgets.QHBoxLayout()
        self.btn_reset = QtWidgets.QPushButton("Reset")
        self.btn_reset.clicked.connect(self._reset)
        row.addWidget(self.btn_reset)

        self.btn_pause = QtWidgets.QPushButton("Pause")
        self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(self._pause_toggled)
        row.addWidget(self.btn_pause)

        layout.addLayout(row, 3, 0)

        self._apply_static_ylims()

    def _apply_static_ylims(self):
        if self.ylims is None:
            def auto_limits(arr):
                mn, mx = float(np.min(arr)), float(np.max(arr))
                if mn == mx:
                    mn -= 1.0
                    mx += 1.0
                pad = 0.05 * (mx - mn)
                return (mn - pad, mx + pad)

            ys = auto_limits(self.source.surge)
            yw = auto_limits(self.source.sway)
            yh = auto_limits(self.source.heave)
        else:
            ys = self.ylims.get("surge", None)
            yw = self.ylims.get("sway", None)
            yh = self.ylims.get("heave", None)
            if ys is None or yw is None or yh is None:
                raise ValueError("ylims must provide 'surge', 'sway', 'heave' tuples")

        self.p_surge.setYRange(ys[0], ys[1], padding=0)
        self.p_sway.setYRange(yw[0], yw[1], padding=0)
        self.p_heave.setYRange(yh[0], yh[1], padding=0)

    def _reset(self):
        self.source.reset()
        self.t0_data = None
        self.t0_wall = None
        self.t_now = None
        self.btn_pause.setChecked(False)

    def _pause_toggled(self, paused: bool):
        self.btn_pause.setText("Resume" if paused else "Pause")
        if not paused and self.t_now is not None:
            self.t0_wall = time.perf_counter()
            self.t0_data = self.t_now  # resume from current time

    def _tick(self):
        if self.btn_pause.isChecked():
            return
        if not self.source.has_next():
            return

        if self.t0_data is None:
            self.t0_data = self.source.peek_time0()
            self.t0_wall = time.perf_counter()

        wall_elapsed = time.perf_counter() - self.t0_wall
        t_play = self.t0_data + wall_elapsed * self.realtime_factor

        last = self.source.advance_to_time(t_play)
        if last is None:
            return

        t_now, surge_now, sway_now, heave_now = last
        self.t_now = t_now

        # Update absolute-time axis offsets and force redraw (clear cached picture)
        for ax in (self.ax_surge, self.ax_sway, self.ax_heave):
            ax.set_offset(t_now)
            ax.picture = None
            ax.update()

        # Window around now (past+future)
        i0, i1 = self.source.window_indices(t_now, self.past_sec, self.future_sec)

        tw = self.source.t[i0:i1] - t_now  # relative time (now = 0)
        surge = self.source.surge[i0:i1]
        sway  = self.source.sway[i0:i1]
        heave = self.source.heave[i0:i1]

        # Update curves (data moves right->left because now stays at x=0)
        self.c_surge.setData(tw, surge)
        self.c_sway.setData(tw, sway)
        self.c_heave.setData(tw, heave)

        # Current sample markers at x=0
        self.m_surge.setData([{"pos": (0.0, surge_now)}])
        self.m_sway.setData([{"pos": (0.0, sway_now)}])
        self.m_heave.setData([{"pos": (0.0, heave_now)}])

        # Update time label at x=0 near top
        txt = f"t = {t_now:.3f} s"

        ys0, ys1 = self.p_surge.viewRange()[1]
        yw0, yw1 = self.p_sway.viewRange()[1]
        yh0, yh1 = self.p_heave.viewRange()[1]

        self.time_label_surge.setText(txt)
        self.time_label_sway.setText(txt)
        self.time_label_heave.setText(txt)

        self.time_label_surge.setPos(0.0, ys1)
        self.time_label_sway.setPos(0.0, yw1)
        self.time_label_heave.setPos(0.0, yh1)

        self.statusBar().showMessage(f"t = {t_now:.3f} s")


def main():
    csv_path = "MCA_Output.csv"
    source = PlaybackSource(csv_path)

    past_sec = 2.5
    future_sec = 2.5

    ylims = {
        "surge": (-25, 25),
        "sway":  (-25, 25),
        "heave": (-25, 25),
    }

    app = QtWidgets.QApplication(sys.argv)
    w = FutureViewer(source, past_sec=past_sec, future_sec=future_sec,
                     realtime_factor=1.0, ylims=ylims)
    w.resize(900, 800)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
