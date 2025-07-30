# ============================================================
# Author      : Baptiste Poncet
# Date        : 07/07/2025
# File        : vision_utils.py
# Description : Visual Odometry from the video stream to calculate displacement
# ============================================================


import cv2
import numpy as np
import time
from tkinter import filedialog

# ===== Camera Parameters =====
CAMERA_INDEX = 1      # 0 for default camera, 1 for USB camera
FOCAL_MATRIX = np.array([[825.7095, 0, 416.1148],
                         [0, 837.0046, 490.3529],
                         [0, 0, 1]])
DEFAULT_SCALE = 0.3
CAPTURE_INTERVAL = 0.30 / 0.035

class VisualOdometry:
    def __init__(self, camera_index=CAMERA_INDEX, focal_matrix=FOCAL_MATRIX, scale=DEFAULT_SCALE):
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        self.focal_matrix = focal_matrix
        self.scale = scale
        self.pose = np.eye(4)
        self.trajectory = []
        self.prev_img = None
        self.prev_kp = None
        self.prev_des = None
        self.origin_point = None
        self.last_capture_time = time.time()

        # ===== Feature detection =====
        self.sift = cv2.SIFT_create()
        self.bf = cv2.BFMatcher(cv2.NORM_L2)

        self.capture_requested = False

    def release(self):
        self.cap.release()

    def read_frame(self):
        now = time.time()
        if now - self.last_capture_time >= 10.0:  # Trigger capture even without instruction after 10 seconds
            ret, frame = self.cap.read()
            if not ret:
                return None
            self.last_capture_time = now  # Updates the last capture time
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return gray, rgb
        if self.capture_requested:
            self.capture_requested = False
            self.last_trigger_time = now
            ret, frame = self.cap.read()
            if not ret:
                return None
            self.last_capture_time = now  # Updates the last capture time
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            return gray, rgb

    def update_pose(self, gray_frame):
        kp, des = self.sift.detectAndCompute(gray_frame, None)

        if self.prev_img is None or des is None or self.prev_des is None:
            self.prev_img, self.prev_kp, self.prev_des = gray_frame, kp, des
            return None  # Not enough data

        matches = self.bf.knnMatch(self.prev_des, des, k=2)
        pts1, pts2 = [], []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                pts1.append(self.prev_kp[m.queryIdx].pt)
                pts2.append(kp[m.trainIdx].pt)

        if len(pts1) < 8:
            self.prev_img, self.prev_kp, self.prev_des = gray_frame, kp, des
            return None

        pts1 = np.array(pts1, dtype=np.float32)
        pts2 = np.array(pts2, dtype=np.float32)

        E, mask = cv2.findEssentialMat(pts2, pts1, self.focal_matrix, cv2.RANSAC, 0.999, 1.0)
        if E is None:
            return None

        pts1_inliers = pts1[mask.ravel() == 1]
        pts2_inliers = pts2[mask.ravel() == 1]
        retval, R, t, _ = cv2.recoverPose(E, pts1_inliers, pts2_inliers, self.focal_matrix)

        if retval < 30 or np.linalg.norm(t) < 1e-3:
            return None

        scale = self.scale / (np.linalg.norm(t) + 1e-8)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = (scale * t).reshape(3)
        self.pose = self.pose @ T

        trans = self.pose[:3, 3]
        point = np.array([-trans[2], -trans[0]])

        if self.origin_point is None:
            self.origin_point = point

        shifted_point = point - self.origin_point
        self.trajectory.append(shifted_point)

        self.prev_img, self.prev_kp, self.prev_des = gray_frame, kp, des
        return shifted_point

    def get_rgb_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def get_trajectory(self):
        return np.array(self.trajectory)

    def reset_trajectory(self):
        self.trajectory.clear()
        self.pose = np.eye(4)
        self.prev_img = None
        self.prev_kp = None
        self.prev_des = None
        self.origin_point = None

    def save_trajectory_to_csv(self, filepath=None):
        if filepath is None:
            filepath = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV files", "*.csv")])
        if filepath:
            np.savetxt(filepath, self.get_trajectory(), delimiter=",", header="x,y", comments="")
            return True
        return False
    
    def trigger_capture(self):
        self.capture_requested = True
        self.last_capture_time = time.time()

    def get_position(self):
        result = self.read_frame()
        if result is None:
            return None, None

        gray, _ = result
        position = self.update_pose(gray)
        if position is not None:
            return position[0], position[1]  # x, y
        else:
            return None, None