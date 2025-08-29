# ============================================================
# Author      : Baptiste Poncet
# Date        : 18/08/2025
# File        : vision_utils.py
# Description : Visual Odometry from the video stream to calculate displacement
# ============================================================


import cv2
import numpy as np
import time
import threading
from tkinter import filedialog

# ===== Camera Parameters =====
FOCAL_MATRIX = np.array([[825.7095, 0, 416.1148],
                         [0, 837.0046, 490.3529],
                         [0, 0, 1]])
DEFAULT_SCALE = 0.3


# ===== Pipeline GStreamer =====
gst_pipeline = (
    "v4l2src device=/dev/video0 ! "
    "video/x-raw, width=640, height=480, framerate=30/1 ! "
    "videoconvert ! appsink drop=1 max-buffers=1"   # ⚡ Ajout : ne bufferise qu'une seule frame
)

class VisualOdometry:
    def __init__(self, focal_matrix=FOCAL_MATRIX, scale=DEFAULT_SCALE):
        self.cap = None   # caméra pas encore ouverte
        self.running = False
        self.thread = None
        self.lock = threading.Lock()

        self.focal_matrix = focal_matrix
        self.scale = scale
        self.pose = np.eye(4)
        self.trajectory = []
        self.prev_img = None
        self.prev_kp = None
        self.prev_des = None
        self.origin_point = None

        self.sift = cv2.ORB_create()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)

        # ===== Capture control =====
        self.frame = None
        self.capture_requested = False
        self.last_capture_time = time.time()
        self.min_interval = 10.0  # secondes (fallback)
        self.min_distance = 0.30  # 30 cm
        self.last_position = np.array([0.0, 0.0])

    def open_camera(self):
        if self.cap is None:
            self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            if not self.cap.isOpened():
                raise RuntimeError("Impossible d'ouvrir la caméra via GStreamer.")

            self.running = True
            self.thread = threading.Thread(target=self._update_camera, daemon=True)
            self.thread.start()

    def close_camera(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.cap:
            self.cap.close()
            self.cap = None

    def _update_camera(self):
        while self.running and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                continue
            with self.lock:
                self.frame = frame


    def read_frame(self):
        """Retourne (gray, rgb) uniquement si condition respectée"""
        now = time.time()
        gray, rgb = None, None

        with self.lock:
            if self.frame is None:
                return None

            # condition temps écoulé
            if self.capture_requested:
                frame_copy = self.frame.copy()
                gray = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)
                rgb = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2RGB)

                self.last_capture_time = now
                self.capture_requested = False
                return gray, rgb

        return None  # sinon rien

    def trigger_capture(self):
        """Force la prochaine image à être prise"""
        self.capture_requested = True

    def update_pose(self, gray_frame):
        kp, des = self.sift.detectAndCompute(gray_frame, None)

        if self.prev_img is None or des is None or self.prev_des is None:
            self.prev_img, self.prev_kp, self.prev_des = gray_frame, kp, des
            return None

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

        # === Déclenche la capture auto si distance > 30 cm ===
        if np.linalg.norm(shifted_point - self.last_position) >= self.min_distance:
            self.capture_requested = True
            self.last_position = shifted_point

        self.prev_img, self.prev_kp, self.prev_des = gray_frame, kp, des
        return shifted_point

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