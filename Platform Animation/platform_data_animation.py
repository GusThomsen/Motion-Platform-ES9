import sys
import time
import numpy as np
import pandas as pd
from PyQt6 import QtCore, QtWidgets
import pyqtgraph as pg

try:
    import pyqtgraph.opengl as gl
    GL_OK = True
except Exception as e:
    gl = None
    GL_OK = False
    GL_IMPORT_ERR = e


class AbsoluteTimeAxis(pg.AxisItem):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.t_offset = 0.0

    def set_offset(self, t_now: float):
        self.t_offset = float(t_now)

    def tickStrings(self, values, scale, spacing):
        return [f"{(self.t_offset + v):.3f}" for v in values]


class PlaybackSource:
    """
    Requires columns:
      time, z_m, beta, gamma, surge, sway, heave
    """
    def __init__(self, csv_path: str, *, degrees=True):
        df = pd.read_csv(csv_path)

        req = ["time", "z_m", "beta", "gamma", "surge", "sway", "heave"]
        for c in req:
            if c not in df.columns:
                raise ValueError(f"CSV missing required column: {c}")

        self.t = df["time"].to_numpy(dtype=float)
        self.z_m = df["z_m"].to_numpy(dtype=float)
        self.beta = df["beta"].to_numpy(dtype=float)
        self.gamma = df["gamma"].to_numpy(dtype=float)
        self.surge = df["surge"].to_numpy(dtype=float)
        self.sway  = df["sway"].to_numpy(dtype=float)
        self.heave = df["heave"].to_numpy(dtype=float)

        n = min(map(len, [self.t, self.z_m, self.beta, self.gamma, self.surge, self.sway, self.heave]))
        self.t = self.t[:n]
        self.z_m = self.z_m[:n]
        self.beta = self.beta[:n]
        self.gamma = self.gamma[:n]
        self.surge = self.surge[:n]
        self.sway  = self.sway[:n]
        self.heave = self.heave[:n]

        if np.any(np.diff(self.t) <= 0):
            raise ValueError("time must be strictly increasing")

        if degrees:
            self.beta = np.deg2rad(self.beta)
            self.gamma = np.deg2rad(self.gamma)

        self.i = 0

    def reset(self):
        self.i = 0

    def peek_time0(self) -> float:
        return float(self.t[0])

    def has_next(self) -> bool:
        return self.i < len(self.t)

    def advance_to_time(self, t_play: float):
        last = None
        while self.i < len(self.t) and float(self.t[self.i]) <= t_play:
            last = (
                float(self.t[self.i]),
                float(self.z_m[self.i]),
                float(self.beta[self.i]),
                float(self.gamma[self.i]),
                float(self.surge[self.i]),
                float(self.sway[self.i]),
                float(self.heave[self.i]),
            )
            self.i += 1
        return last

    def window_indices(self, t_center: float, past: float, future: float):
        t0 = t_center - past
        t1 = t_center + future
        i0 = int(np.searchsorted(self.t, t0, side="left"))
        i1 = int(np.searchsorted(self.t, t1, side="right"))
        i0 = max(0, min(i0, len(self.t)))
        i1 = max(0, min(i1, len(self.t)))
        return i0, i1


def rot_x(a: float) -> np.ndarray:
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0],
                     [0, ca, -sa],
                     [0, sa,  ca]], dtype=float)

def rot_y(a: float) -> np.ndarray:
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[ ca, 0, sa],
                     [  0, 1,  0],
                     [-sa, 0, ca]], dtype=float)

def platform_kinematics(z_m, beta, gamma, d_pts, g_pts, z_offset=0.30):
    C = np.array([0.0, 0.0, z_offset + z_m], dtype=float)
    R = rot_x(beta) @ rot_y(gamma)   # keep the convention you already validated
    A = (C.reshape(3, 1) + R @ d_pts).T  # (4,3)
    return A, g_pts

class PlatformAndTelemetry(QtWidgets.QMainWindow):
    def __init__(self, source: PlaybackSource, *,
                 past_sec=2.5, future_sec=2.5,
                 realtime_factor=1.0,
                 ylims=None,
                 dx=0.5, dy=0.2, t_plate=0.10, gx=0.5, gy=0.2,
                 z_offset=0.30):
        super().__init__()
        self.setWindowTitle("Platform + Telemetry (Synced)")

        if not GL_OK:
            QtWidgets.QMessageBox.critical(
                self, "OpenGL import failed",
                f"pyqtgraph.opengl failed to import:\n{GL_IMPORT_ERR}\n\n"
                f"Install dependencies:\n  pip install pyopengl"
            )

        self.source = source
        self.past_sec = float(past_sec)
        self.future_sec = float(future_sec)
        self.realtime_factor = float(realtime_factor)
        self.ylims = ylims

        self.dx = float(dx)
        self.dy = float(dy)
        self.t_plate = float(t_plate)
        self.gx = float(gx)
        self.gy = float(gy)
        self.z_offset = float(z_offset)

        self.d_pts = np.array([
            [ self.dx, -self.dy, -self.t_plate/2],
            [-self.dx, -self.dy, -self.t_plate/2],
            [ self.dx,  self.dy, -self.t_plate/2],
            [-self.dx,  self.dy, -self.t_plate/2],
        ], dtype=float).T  # (3,4)

        self.g_pts = np.array([
            [ self.gx, -self.gy, 0.0],
            [-self.gx, -self.gy, 0.0],
            [ self.gx,  self.gy, 0.0],
            [-self.gx,  self.gy, 0.0],
        ], dtype=float)

        self.t0_data = None
        self.t0_wall = None
        self.t_now = None

        self._build_ui()
        self._apply_static_ylims()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(16)

    def _build_ui(self):
        cw = QtWidgets.QWidget()
        self.setCentralWidget(cw)

        layout = QtWidgets.QGridLayout(cw)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(10)

        # CRITICAL: give the 3D column space
        layout.setColumnStretch(0, 3)
        layout.setColumnStretch(1, 2)

        # ---------- Left: 3D ----------
        self.view = gl.GLViewWidget()
        self.view.setMinimumSize(650, 650)  # CRITICAL: prevents it collapsing
        self.view.opts["distance"] = 2.2
        self.view.opts["azimuth"] = 135
        self.view.opts["elevation"] = 25
        layout.addWidget(self.view, 0, 0, 3, 1)

        # Make it obvious the GL view is alive
        axis_item = gl.GLAxisItem()
        axis_item.setSize(0.3, 0.3, 0.3)
        self.view.addItem(axis_item)

        grid = gl.GLGridItem()
        grid.setSize(2, 2)
        grid.setSpacing(0.1, 0.1)
        self.view.addItem(grid)

        # Ground points as short vertical lines
        for Gi in self.g_pts:
            pts = np.array([Gi, Gi + np.array([0, 0, 0.03])], dtype=float)
            self.view.addItem(gl.GLLinePlotItem(pos=pts, width=2, antialias=True))

        self.plate = gl.GLLinePlotItem(width=3, antialias=True)
        self.view.addItem(self.plate)

        self.act = []
        # after creating self.plate / self.act in _build_ui()
        self.nose_marker = gl.GLLinePlotItem(width=4, antialias=True)
        self.view.addItem(self.nose_marker)


        for _ in range(4):
            li = gl.GLLinePlotItem(width=2, antialias=True)
            self.view.addItem(li)
            self.act.append(li)

        # ---------- Right: telemetry ----------
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
            p.setLabel("bottom", "Time", units="s")
            p.setXRange(-self.past_sec, self.future_sec, padding=0)

        self.p_surge.setLabel("left", "m/s²")
        self.p_sway.setLabel("left", "m/s²")
        self.p_heave.setLabel("left", "m/s²")

        layout.addWidget(self.p_surge, 0, 1)
        layout.addWidget(self.p_sway,  1, 1)
        layout.addWidget(self.p_heave, 2, 1)

        self.c_surge = self.p_surge.plot([], [])
        self.c_sway  = self.p_sway.plot([], [])
        self.c_heave = self.p_heave.plot([], [])

        for pw in (self.p_surge, self.p_sway, self.p_heave):
            pw.addItem(pg.InfiniteLine(pos=0.0, angle=90, movable=False))

        self.m_surge = pg.ScatterPlotItem(size=8)
        self.m_sway  = pg.ScatterPlotItem(size=8)
        self.m_heave = pg.ScatterPlotItem(size=8)
        self.p_surge.addItem(self.m_surge)
        self.p_sway.addItem(self.m_sway)
        self.p_heave.addItem(self.m_heave)

        self.time_label_surge = pg.TextItem(anchor=(0.5, 1.0))
        self.time_label_sway  = pg.TextItem(anchor=(0.5, 1.0))
        self.time_label_heave = pg.TextItem(anchor=(0.5, 1.0))
        self.p_surge.addItem(self.time_label_surge)
        self.p_sway.addItem(self.time_label_sway)
        self.p_heave.addItem(self.time_label_heave)

        # Controls
        row = QtWidgets.QHBoxLayout()
        self.btn_reset = QtWidgets.QPushButton("Reset")
        self.btn_reset.clicked.connect(self._reset)
        row.addWidget(self.btn_reset)

        self.btn_pause = QtWidgets.QPushButton("Pause")
        self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(self._pause_toggled)
        row.addWidget(self.btn_pause)

        layout.addLayout(row, 3, 0, 1, 2)

        # Initial platform draw
        A, G = platform_kinematics(self.source.z_m[0], self.source.beta[0], self.source.gamma[0],
                                   self.d_pts, self.g_pts, z_offset=self.z_offset)
        corners = np.array([A[0], A[1], A[3], A[2], A[0]], dtype=float)
        self.plate.setData(pos=corners)
        for i in range(4):
            self.act[i].setData(pos=np.array([G[i], A[i]], dtype=float))

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
            ys = self.ylims["surge"]
            yw = self.ylims["sway"]
            yh = self.ylims["heave"]

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
            self.t0_data = self.t_now

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

        t_now, z_m, beta, gamma, surge_now, sway_now, heave_now = last
        self.t_now = t_now

        # Platform update
        A, G = platform_kinematics(z_m, beta, gamma, self.d_pts, self.g_pts, z_offset=self.z_offset)
        corners = np.array([A[0], A[1], A[3], A[2], A[0]], dtype=float)
        self.plate.setData(pos=corners)
        for i in range(4):
            self.act[i].setData(pos=np.array([G[i], A[i]], dtype=float))

        nose_mid = 0.5*(A[0] + A[2])          # midpoint of +x edge
        nose_tip = nose_mid + np.array([0,0,0.08])  # small vertical marker
        self.nose_marker.setData(pos=np.array([nose_mid, nose_tip], float))


        # Telemetry window
        i0, i1 = self.source.window_indices(t_now, self.past_sec, self.future_sec)
        tw = self.source.t[i0:i1] - t_now
        surge = self.source.surge[i0:i1]
        sway  = self.source.sway[i0:i1]
        heave = self.source.heave[i0:i1]

        self.c_surge.setData(tw, surge)
        self.c_sway.setData(tw, sway)
        self.c_heave.setData(tw, heave)

        self.m_surge.setData([{"pos": (0.0, surge_now)}])
        self.m_sway.setData([{"pos": (0.0, sway_now)}])
        self.m_heave.setData([{"pos": (0.0, heave_now)}])

        # Absolute-time tick labels
        for ax in (self.ax_surge, self.ax_sway, self.ax_heave):
            ax.set_offset(t_now)
            ax.picture = None
            ax.update()

        # Time labels
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

    # csv_path = "motion_telemetry_export_FlippedSign.csv"
    source = PlaybackSource(csv_path, degrees=True)

    ylims = {"surge": (-25, 25), "sway": (-25, 25), "heave": (-25, 25)}

    app = QtWidgets.QApplication(sys.argv)
    w = PlatformAndTelemetry(
        source,
        past_sec=2.5, future_sec=2.5,
        realtime_factor=1.0,
        ylims=ylims,
        dx=0.5, dy=0.2, t_plate=0.10, gx=0.5, gy=0.2,
        z_offset=0.30
    )
    w.resize(1400, 850)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
