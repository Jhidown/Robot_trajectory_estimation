import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from tkinter import filedialog
import math

class EKFFusion:
    def __init__(self):
        self.ekf = ExtendedKalmanFilter(dim_x=3, dim_z=2)
        self.ekf.x = np.array([0., 0., 0.])  # x, y, θ
        self.ekf.P = np.eye(3) * 1.0
        self.ekf.Q = np.diag([0.01, 0.01, 0.001])
        self.ekf.R = np.diag([0.01, 0.01])
        self.trajectory = []

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

        # Jacobienne F(x, u, dt)
        F = self.Fx(x, u, dt)

        # Mise à jour de la covariance
        P_pred = F @ P @ F.T + Q

        # Mise à jour de l'état dans le filtre
        self.ekf.x = x_pred
        self.ekf.P = P_pred

        self.trajectory.append((x_pred[0], x_pred[1]))

        
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

    def save_trajectory_to_csv(self, filepath=None):
        if filepath is None:
            filepath = filedialog.asksaveasfilename(defaultextension=".csv",
                                                    filetypes=[("CSV files", "*.csv")])
        if filepath:
            np.savetxt(filepath, self.trajectory, delimiter=",", header="x,y", comments="")
            return True
        return False
    