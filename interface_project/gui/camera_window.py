# ============================================================
# Author      : Baptiste Poncet
# Date        : 07/07/2025
# File        : camera_window.py
# Description : Camera display window and trajectory display (VO)
# ============================================================

import tkinter as tk
from tkinter import filedialog
from PIL import Image, ImageTk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from logic.vision_utils import VisualOdometry
import numpy as np



def open_camera_window(root):
    running = True

    camera_window = tk.Toplevel(root)
    camera_window.title("Camera window")

    # ===== Initialisation de la VO =====
    vo = VisualOdometry()
    
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
    fig = Figure(figsize=(5.5, 5.5))
    ax = fig.add_subplot(111)
    ax.set_title("Trajectory")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.grid(True)
    ax.axis("equal")
    trajectory_line, = ax.plot([], [], 'g-')

    canvas = FigureCanvasTkAgg(fig, master=plot_frame)
    canvas.draw()
    canvas.get_tk_widget().pack()

    # ===== Video label =====
    video_label = tk.Label(video_frame)
    video_label.pack()

    # ===== Actions =====
    def camera_save_data():
        vo.save_trajectory_to_csv()

    def camera_save_plot():
        file_path = filedialog.asksaveasfilename(defaultextension=".png", filetypes=[("PNG files", "*.png")])
        if file_path:
            fig.savefig(file_path, bbox_inches='tight')

    def reset_trajectory_camera():
        vo.reset_trajectory()
        trajectory_line.set_data([], [])
        ax.set_xlim(-1, 1)
        ax.set_ylim(-1, 1)
        canvas.draw()

    def camera_stop():
        vo.release()

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
            trajectory = vo.get_trajectory()
            x_shifted = trajectory[:, 0]
            y_shifted = trajectory[:, 1]
            trajectory_line.set_data(x_shifted, y_shifted)

            max_range = max(np.max(np.abs(x_shifted)), np.max(np.abs(y_shifted))) + 0.5
            ax.set_xlim(-max_range, max_range)
            ax.set_ylim(-max_range, max_range)
            canvas.draw()

        camera_window.after(50, update_loop)

    def on_close():
        nonlocal running
        running = False
        vo.release()
        camera_window.destroy()

    camera_window.protocol("WM_DELETE_WINDOW", on_close)

    # ===== Buttons =====
    tk.Button(camera_button_frame, text="Save Data", width=15, command=camera_save_data).grid(row=0, column=0, padx=10, pady=10)
    tk.Button(camera_button_frame, text="Save Plot", width=15, command=camera_save_plot).grid(row=0, column=1, padx=10, pady=10)
    tk.Button(camera_button_frame, text="Reset Trajectory", width=15, command=reset_trajectory_camera).grid(row=0, column=2, padx=10, pady=10)
    tk.Button(camera_button_frame, text="Stop Camera", width=15, fg="red", command=camera_stop).grid(row=0, column=3, padx=10, pady=10)

    update_loop()