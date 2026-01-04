import sys
import time
import numpy as np
import pandas as pd

from PyQt6 import QtCore, QtWidgets
import pyqtgraph.opengl as gl


# ---------------------------
# Rotation helpers
# ---------------------------
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

def rot_z(a: float) -> np.ndarray:
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[ca, -sa, 0],
                     [sa,  ca, 0],
                     [ 0,   0, 1]], dtype=float)


# ---------------------------
# Kinematics (matches MATLAB spirit)
# ---------------------------
def platform_kinematics(z_m, beta, gamma, d_pts, g_pts, z_offset=0.30):
    """
    Returns:
      A: (4,3) platform attachment points in world frame
      G: (4,3) ground points
      C: (3,)  platform center
      L: (4,)  actuator lengths
    Convention (adjust if needed):
      beta  = roll  about X
      gamma = pitch about Y
      R = Rx(beta) @ Ry(gamma)
    """
    C = np.array([0.0, 0.0, z_offset + z_m], dtype=float)
    R = rot_x(beta) @ rot_y(gamma)

    # d_pts: (3,4)
    A = (C.reshape(3, 1) + R @ d_pts).T  # -> (4,3)
    L = np.linalg.norm(A - g_pts, axis=1)
    return A, g_pts, C, L


class CSVPlayback:
    def __init__(self, csv_path: str):
        df = pd.read_csv(csv_path)

        # required columns
        req = ["time", "z_m", "beta", "gamma"]
        for c in req:
            if c not in df.columns:
                raise ValueError(f"CSV missing required column: {c}")

        self.t = df["time"].to_numpy(dtype=float)
        self.z_m = df["z_m"].to_numpy(dtype=float)
        self.beta = df["beta"].to_numpy(dtype=float)
        self.gamma = df["gamma"].to_numpy(dtype=float)

        n = min(len(self.t), len(self.z_m), len(self.beta), len(self.gamma))
        self.t = self.t[:n]
        self.z_m = self.z_m[:n]
        self.beta = self.beta[:n]
        self.gamma = self.gamma[:n]
        self.beta = np.deg2rad(self.beta)
        self.gamma = np.deg2rad(self.gamma)


        if np.any(np.diff(self.t) <= 0):
            raise ValueError("time must be strictly increasing")

        self.i = 0

    def reset(self):
        self.i = 0

    def has_next(self):
        return self.i < len(self.t)

    def next(self):
        i = self.i
        self.i += 1
        return self.t[i], self.z_m[i], self.beta[i], self.gamma[i]


class Platform3DApp(QtWidgets.QMainWindow):
    def __init__(self, source: CSVPlayback, *,
                 dx=0.5, dy=0.2, t_plate=0.10, gx=0.5, gy=0.2,
                 z_offset=0.30, realtime_factor=1.0):
        super().__init__()
        self.setWindowTitle("3DOF Platform 3D Viewer")

        self.source = source
        self.dx = float(dx)
        self.dy = float(dy)
        self.t_plate = float(t_plate)
        self.gx = float(gx)
        self.gy = float(gy)
        self.z_offset = float(z_offset)
        self.realtime_factor = float(realtime_factor)

        # d points in platform frame (3x4) and ground points (4x3)
        self.d_pts = np.array([
            [ self.dx, -self.dy, -self.t_plate/2],
            [-self.dx, -self.dy, -self.t_plate/2],
            [ self.dx,  self.dy, -self.t_plate/2],
            [-self.dx,  self.dy, -self.t_plate/2],
        ], dtype=float).T

        self.g_pts = np.array([
            [ self.gx, -self.gy, 0.0],
            [-self.gx, -self.gy, 0.0],
            [ self.gx,  self.gy, 0.0],
            [-self.gx,  self.gy, 0.0],
        ], dtype=float)

        self._build_ui()
        self._build_scene()

        self.t0_data = None
        self.t0_wall = None
        self.last_sample_t = None

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(16)  # ~60 FPS UI tick


    def _build_ui(self):
        cw = QtWidgets.QWidget()
        self.setCentralWidget(cw)

        layout = QtWidgets.QVBoxLayout(cw)
        layout.setContentsMargins(8, 8, 8, 8)

        self.view = gl.GLViewWidget()
        self.view.opts["distance"] = 2.2
        self.view.opts["azimuth"] = 135
        self.view.opts["elevation"] = 25
        layout.addWidget(self.view, stretch=1)

        row = QtWidgets.QHBoxLayout()
        self.btn_reset = QtWidgets.QPushButton("Reset")
        self.btn_reset.clicked.connect(self._reset)
        row.addWidget(self.btn_reset)

        self.btn_pause = QtWidgets.QPushButton("Pause")
        self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(self._pause_toggled)
        row.addWidget(self.btn_pause)

        layout.addLayout(row)

    def _build_scene(self):
        # Grid
        grid = gl.GLGridItem()
        grid.setSize(2, 2)
        grid.setSpacing(0.1, 0.1)
        self.view.addItem(grid)

        # Ground points markers
        for Gi in self.g_pts:
            pts = np.array([Gi, Gi + np.array([0, 0, 0.03])], dtype=float)
            self.view.addItem(gl.GLLinePlotItem(pos=pts, width=2, antialias=True))

        # Plate outline
        self.plate = gl.GLLinePlotItem(width=3, antialias=True)
        self.view.addItem(self.plate)

        # Actuator lines
        self.act = []
        for _ in range(4):
            li = gl.GLLinePlotItem(width=2, antialias=True)
            self.view.addItem(li)
            self.act.append(li)

        # Initial render from first sample (peek without consuming)
        t0, z0, b0, g0 = self.source.t[0], self.source.z_m[0], self.source.beta[0], self.source.gamma[0]
        self._update_scene(z0, b0, g0)
        self.statusBar().showMessage(f"t = {t0:.3f} s")

    def _update_scene(self, z_m, beta, gamma):
        A, G, C, L = platform_kinematics(z_m, beta, gamma, self.d_pts, self.g_pts, z_offset=self.z_offset)

        # Plate order matching your MATLAB patch: [1 2 4 3] and close
        corners = np.array([A[0], A[1], A[3], A[2], A[0]], dtype=float)
        self.plate.setData(pos=corners)

        # Actuators
        for i in range(4):
            pts = np.array([G[i], A[i]], dtype=float)
            self.act[i].setData(pos=pts)

    def _reset(self):
        self.source.reset()
        self.t0_data = None
        self.t0_wall = None
        self.last_sample_t = None
        self.btn_pause.setChecked(False)

    def _pause_toggled(self, paused: bool):
        self.btn_pause.setText("Resume" if paused else "Pause")
        if not paused and self.last_sample_t is not None:
            # resync so we don't jump
            self.t0_wall = time.perf_counter()
            self.t0_data = self.last_sample_t

    def _tick(self):
        if self.btn_pause.isChecked():
            return
        if not self.source.has_next():
            return

        # Initialize time bases on first tick
        if self.t0_data is None:
            # NOTE: use the FIRST sample time in the file as data start
            self.t0_data = float(self.source.t[0])
            self.t0_wall = time.perf_counter()

        wall_elapsed = time.perf_counter() - self.t0_wall

        # Desired data time at this wall time (realtime_factor=1 => real time)
        t_play = self.t0_data + wall_elapsed * self.realtime_factor

        # Consume samples until we reach t_play (catch up)
        # Keep the last sample we consumed, and render that.
        updated = False
        while self.source.has_next():
            t, z_m, beta, gamma = self.source.next()
            if t > t_play:
                # We stepped one sample too far; step back one index
                self.source.i -= 1
                break

            self.last_sample_t = t
            self._update_scene(z_m, beta, gamma)
            self.statusBar().showMessage(f"t = {t:.3f} s")
            updated = True

        # If we're exactly between samples, no need to redraw.
        # But if you want smoother, you could interpolate here later.
        if updated:
            pass



def main():
    # Change this to your export file
    csv_path = "MCA_Output.csv"

    src = CSVPlayback(csv_path)

    app = QtWidgets.QApplication(sys.argv)

    # TODO: set these to your actual geometry if different
    w = Platform3DApp(
        src,
        dx=0.5, dy=0.2, t_plate=0.10, gx=0.5, gy=0.2,
        z_offset=0.30,
        realtime_factor=1.0
    )
    w.resize(1000, 800)
    w.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
