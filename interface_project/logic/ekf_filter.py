# ============================================================
# Author      : Baptiste Poncet
# Date        : 30/07/2025
# File        : ekf_filter.py
# Description : Extended Kalman Filter for fusing camera and wheel odometry data
# ============================================================

import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from tkinter import filedialog


class EKFFusion:
    def __init__(self):
        self.ekf = ExtendedKalmanFilter(dim_x=3, dim_z=2)
        self.ekf.x = np.array([0., 0., 0.])  # x, y, θ
        self.ekf.P = np.eye(3) * 1.0
        self.ekf.Q = np.diag([0.1, 0.1, 0.05])
        self.ekf.R = np.diag([0.01, 0.01])
        self.trajectory = []
        self.encoder_trajectory = []

    def fx(self, x, u, dt):
        v, omega = u
        theta = x[2]
        dx = v * np.cos(theta) * dt
        dy = v * np.sin(theta) * dt
        dtheta = omega * dt
        return np.array([
            x[0] + dx,
            x[1] + dy,
            x[2] + dtheta
        ])

    def Fx(self, x, u, dt):
        v = u[0]
        theta = x[2]
        return np.array([
            [1, 0, -v * np.sin(theta) * dt],
            [0, 1,  v * np.cos(theta) * dt],
            [0, 0, 1]
        ])

    def predict(self, delta_d, delta_theta, dt):
        v = delta_d / dt
        omega = delta_theta / dt
        u = [v, omega]

        x = self.ekf.x
        P = self.ekf.P
        Q = self.ekf.Q

        # Prédiction de l’état par équation non linéaire
        x_pred = self.fx(x, u, dt)
        x_enc, y_enc = x_pred[0], x_pred[1]
        self.encoder_trajectory.append((x_enc, y_enc))


        F = self.Fx(x, u, dt)
        P_pred = F @ P @ F.T + Q

        self.ekf.x = x_pred
        self.ekf.P = P_pred

        
    def get_pose(self):
        return self.ekf.x
    
    def hx(self, x):
        return np.array([x[0], x[1]])  # Mesure : position x, y uniquement

    def Hx(self, x):
        return np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])  # Jacobienne H
    
    def update(self, x_cam, y_cam):
        z = np.array([x_cam, y_cam])
        self.ekf.update(z, HJacobian=self.Hx, Hx=self.hx)
        self.trajectory.append((self.ekf.x[0], self.ekf.x[1]))

    def save_trajectory_to_csv(self, filepath=None):
        if filepath is None:
            filepath = filedialog.asksaveasfilename(defaultextension=".csv",
                                                    filetypes=[("CSV files", "*.csv")])
        if filepath:
            np.savetxt(filepath, self.trajectory, delimiter=",", header="x,y", comments="")
            return True
        return False
    
    def save_encoder_trajectory_to_csv(self, filepath=None):
        if filepath is None:
            filepath = filedialog.asksaveasfilename(defaultextension=".csv",
                                                    filetypes=[("CSV files", "*.csv")])
        if filepath :
            np.savetxt(filepath, np.array(self.encoder_trajectory), delimiter=",", header="x,y", comments="")
            return True
        return False

    def reset(self):
        self.ekf.x = np.array([0., 0., 0.])
        self.ekf.P = np.eye(3) * 1.0
        self.trajectory.clear()
        self.encoder_trajectory.clear()