# ============================================================
# Author      : Baptiste Poncet
# Date        : 04/08/2025
# File        : trajectory.py
# Description : Trajectory estimation, plotting and saving utilities
# ============================================================


from matplotlib.figure import Figure
import numpy as np


class TrajectoryPlotter:
    def __init__(self):
        self.fig = Figure(figsize=(5.5, 5.5))
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Trajectory")
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.grid(True)
        self.ax.axis("equal")

        self.trajectory_line, = self.ax.plot([], [], 'g-', label='VO')
        self.ekf_line, = self.ax.plot([], [], 'r-', label='EKF')
        self.encoder_line, = self.ax.plot([], [], 'b--', label="Encoders")

        self.ax.legend()
        self.canvas = self.fig.canvas

    def update(self, vo_trajectory, ekf_trajectory, encoder_trajectory, mode="VO + EKF + Encoders"):
        show_vo = "VO" in mode
        show_ekf = "EKF" in mode
        show_enc = "Encodeurs" in mode

        if show_vo and len(vo_trajectory) > 1:
            x_vo = vo_trajectory[:, 0]
            y_vo = vo_trajectory[:, 1]
            self.trajectory_line.set_data(x_vo, y_vo)
        else:
            self.trajectory_line.set_data([], [])

        if show_ekf and len(ekf_trajectory) > 1:
            x_ekf = ekf_trajectory[:, 0]
            y_ekf = ekf_trajectory[:, 1]
            self.ekf_line.set_data(x_ekf, y_ekf)
        else:
            self.ekf_line.set_data([], [])

        if show_enc and len(encoder_trajectory) > 1:
            x_enc = [p[0] for p in encoder_trajectory]
            y_enc = [p[1] for p in encoder_trajectory]
            self.encoder_line.set_data(x_enc, y_enc)
        else:
            self.encoder_line.set_data([], [])

        
        all_x, all_y = [], []
        if show_vo and len(vo_trajectory) > 1:
            all_x += list(x_vo)
            all_y += list(y_vo)
        if show_ekf and len(ekf_trajectory) > 1:
            all_x += list(x_ekf)
            all_y += list(y_ekf)
        if show_enc and len(encoder_trajectory) > 1:
            all_x += x_enc
            all_y += y_enc

        if all_x and all_y:
            max_range = max(np.max(np.abs(all_x)), np.max(np.abs(all_y))) + 0.5
        else:
            max_range = 1

        self.ax.set_xlim(-max_range, max_range)
        self.ax.set_ylim(-max_range, max_range)

        if self.ax.legend_:
            self.ax.legend_.remove()

        visible_lines = []
        labels = []

        if show_vo:
            visible_lines.append(self.trajectory_line)
            labels.append("VO")
        if show_ekf:
            visible_lines.append(self.ekf_line)
            labels.append("EKF")
        if show_enc:
            visible_lines.append(self.encoder_line)
            labels.append("Encoders")

        if visible_lines:
            self.ax.legend(visible_lines, labels)

        self.canvas.draw()


    def reset(self):
        self.trajectory_line.set_data([], [])
        self.ekf_line.set_data([], [])
        self.encoder_line.set_data([], [])
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.canvas.draw()

    def save(self, filepath):
        self.fig.savefig(filepath, bbox_inches='tight')
