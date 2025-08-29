# ============================================================
# Author      : Baptiste Poncet
# Date        : 18/08/2025
# File        : camera_window.py
# Description : Camera display window and trajectory display (VO)
# ============================================================

import tkinter as tk
from tkinter import filedialog
from tkinter import ttk
from PIL import Image, ImageTk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from logic.vision_utils import VisualOdometry
from logic.trajectory import TrajectoryPlotter
import numpy as np



def open_camera_window(root,ekf,vo):
    running = True

    camera_window = tk.Toplevel(root)
    camera_window.title("Camera window")

    # ===== Initialisation of VO =====
    vo.open_camera()  # Open the camera here
    
    # ===== UI Layout =====
    camera_button_frame = tk.Frame(camera_window)
    camera_button_frame.grid(row=0, column=0, columnspan=3, sticky="nsew", padx=10, pady=5)

    camera_figure_frame = tk.Frame(camera_window)
    camera_figure_frame.grid(row=1, column=0, columnspan=3, sticky="nsew", padx=10, pady=10)

    plot_frame = tk.Frame(camera_figure_frame)
    plot_frame.grid(row=0, column=0, padx=10, pady=10)

    video_frame = tk.Frame(camera_figure_frame)
    video_frame.grid(row=0, column=1, padx=10, pady=10)

    # ===== Matplotlib Plot =====
    plotter = TrajectoryPlotter()
    canvas = FigureCanvasTkAgg(plotter.fig, master=plot_frame)
    canvas.draw()
    canvas.get_tk_widget().pack()
    plotter.canvas = canvas

    # ===== Video label =====
    video_label = tk.Label(video_frame)
    video_label.pack()

    # ===== Actions =====
    def camera_save_data():
        vo.save_trajectory_to_csv()

    def save_ekf_csv():
        ekf.save_trajectory_to_csv()

    def save_wheel_csv():
        ekf.save_encoder_trajectory_to_csv()

    def camera_save_plot():
        file_path = filedialog.asksaveasfilename(defaultextension=".png", filetypes=[("PNG files", "*.png")])
        if file_path:
            plotter.save(file_path)

    def reset_trajectory_camera():
        vo.reset_trajectory()
        ekf.reset()
        plotter.reset()

    def open_camera_window(root, ekf):
        vo.open_camera()  # ouverture réelle ici
        # afficher les frames, etc.

    def close_camera_window():
        vo.close_camera()


    def update_loop():

        if not running:
            return

        gray_rgb = vo.read_frame()
        if gray_rgb is None:
            camera_window.after(50, update_loop)
            return

        gray, rgb = gray_rgb
        img = ImageTk.PhotoImage(Image.fromarray(rgb))
        video_label.imgtk = img
        video_label.configure(image=img)

        point = vo.update_pose(gray)
        if point is not None:
            vo_traj = vo.get_trajectory()
            ekf_traj = np.array(ekf.trajectory)
            enc_traj = getattr(ekf, "encoder_trajectory", [])
            plotter.update(vo_traj, ekf_traj, enc_traj, modes_combobox.get())
        camera_window.after(50, update_loop)

    def on_close():
        nonlocal running
        running = False
        vo.close_camera()
        camera_window.destroy()

    def update_buttons(event=None):
            
        mode = modes_combobox.get()
        vo_traj = vo.get_trajectory()
        ekf_traj = np.array(ekf.trajectory)
        enc_traj = getattr(ekf,"encoder_trajectory", [])
        plotter.update(vo_traj, ekf_traj, enc_traj, mode)

        vo_button.grid() if mode in ["VO", "VO + EKF", "VO + Encoders", "VO + EKF + Encoders"] else vo_button.grid_remove()
        ekf_button.grid() if mode in ["EKF", "VO + EKF", "EKF + Encoders", "VO + EKF + Encoders"] else ekf_button.grid_remove()
        enc_button.grid() if mode in ["Encoders", "VO + Encoders", "EKF + Encoders", "VO + EKF + Encoders"] else enc_button.grid_remove()

    camera_window.protocol("WM_DELETE_WINDOW", on_close)

    # ===== Buttons =====
    modes = ["VO", "EKF", "Encoders", "VO + EKF", "VO + Encoders", "EKF + Encoders", "VO + EKF + Encoders"]
    modes_combobox = ttk.Combobox(camera_button_frame, values = modes, state="readonly",width=20)
    modes_combobox.grid(row=0, column=0, padx=5, pady=5)
    modes_combobox.set("EFK")
    modes_combobox.bind("<<ComboboxSelected>>", update_buttons)

    tk.Button(camera_button_frame, text="Save Plot", width=15, command=camera_save_plot).grid(row=0, column=1, padx=5, pady=5)
    tk.Button(camera_button_frame, text="Reset Trajectory", width=15, command=reset_trajectory_camera).grid(row=0, column=2, padx=5, pady=5)
    tk.Button(camera_button_frame, text="Stop Camera", width=15, fg="red", command=vo.close_camera).grid(row=0, column=3, padx=5, pady=5)
    
    data_text = tk.Label(camera_button_frame, text="Save Data :")
    data_text.grid(row=1, column=0, padx=5, pady=5)

    vo_button = tk.Button(camera_button_frame, text="VO", width=15, command=camera_save_data)
    vo_button.grid(row=1, column=1, padx=5, pady=5)
    ekf_button = tk.Button(camera_button_frame, text="EKF", width=15, command=save_ekf_csv)
    ekf_button.grid(row=1, column=2, padx=5, pady=5)
    enc_button = tk.Button(camera_button_frame, text="Encoders", width=15, command=save_wheel_csv)
    enc_button.grid(row=1, column=3, padx=5, pady=5)

    update_buttons()
    update_loop()