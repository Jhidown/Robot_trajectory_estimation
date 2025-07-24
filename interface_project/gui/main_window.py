# ============================================================
# Author      : Baptiste Poncet
# Date        : 07/07/2025
# File        : main_window.py
# Description : Main graphical interface of the robot control project
# ============================================================


import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), r'C:\Users\bapti\Desktop\Stage Vietnam appli doc\Program\interface_project')))


from .camera_window import open_camera_window
from .styles import toggle_dark_mode
from communication.serial_comm import start_connection, send_custom_message, close_connection
from logic.trajectory import update_trajectory
from communication.flask_server import start_flask_server, register_command_handler, register_log_handler
from logic.vision_utils import VisualOdometry
import tkinter as tk
from tkinter import ttk
from tkinter import filedialog
import serial
import serial.tools.list_ports
from datetime import datetime
import re
"""
    
    Code general structure : 
    root
    |
    |---port_frame
    |   |---top_left_button_frame
    |   |   |---refresh_button
    |   |   |---COM_port_label
    |   |   |---COM_port_combobox
    |   |   |---baud_label
    |   |   |---baud_combobox
    |   |   |---connect_button
    |   |---top_right_button_frame
    |   |   |---dark_button
    |   |   |---open_camera_button
    |---send_frame
    |   |---send_entry
    |   |---send_button
    |---cross_frame
    |   |---forward_button
    |   |---left_button
    |   |---stop_button
    |   |---right_button
    |   |---backward_button
    |   |---left_speed_label
    |   |---left_speed_entry
    |   |---right_speed_label
    |   |---right_speed_entry
    |---info_frame
    |   |---status_section_frame
    |   |   |---status_label
    |   |   |---output_label
    |   |---history_section_frame
    |   |   |---top_row_frame
    |   |   |   |---label_container_frame
    |   |   |   |   |---text_label
    |   |   |   |---button_container_frame
    |   |   |   |   |---save_button
    |   |   |   |   |---reset_button
    |   |   |---history_frame
    |   |   |   |---text_history
    |   |   |   |---scrollbar
    
    
    """
def run_gui():

    # ===== Creation of main window 'root' =====
    root = tk.Tk()
    root.title("Interface Projet")

    # ===== Grid =====
    root.grid_rowconfigure(0, weight=0)  # port_frame
    root.grid_rowconfigure(1, weight=0)  # empty
    root.grid_rowconfigure(2, weight=0)  # send_frame
    root.grid_rowconfigure(3, weight=0)  # cross_frame 
    root.grid_rowconfigure(4, weight=1)  # info_frame 
    for i in range(3):
        root.grid_columnconfigure(i, weight=1, uniform="equal")

    dark_mode = False
    vo = VisualOdometry()

    def insert_custom_message(message: str, tag: str):

        timestamp = datetime.now().strftime("%H:%M:%S")
        full_msg = f"[{timestamp}] {message}"
        text_history.insert(tk.END,full_msg+"\n",tag)
        text_history.see(tk.END)

        match = re.search(r"angle:\s*(\d+)", message)
        if match :
            vo.trigger_capture()



    def connect_serial():
        port = port_var.get()
        baud = int(baud_var.get())

        def log_to_history(msg):
            insert_custom_message(msg,"received")

        success = start_connection(port, baud, log_to_history, update_status)
        if success:
            insert_custom_message(f"[PC] Connection established with {port}","sent")

    def reset_history():
        close_connection()
        text_history.delete("1.0",tk.END)
        text_history.see(tk.END)

    def save_history_as():
        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title="Save history as"
        )
        if file_path:
            with open(file_path, "w", encoding="utf-8") as f:
                f.write(text_history.get("1.0", tk.END))
            output_label.config(text=f"History saved to {file_path}", fg="green")


    def refresh_ports():
        com_ports = [port.device for port in serial.tools.list_ports.comports()]
        COM_port_combobox['values'] = com_ports
        if com_ports:
            COM_port_combobox.current(0)

    def send_movement_command(left: int, right: int):
        message = f"SPEED {right} {left}"

        def update_status(text, color):
            output_label.config(text=text, fg=color)

        send_custom_message(message, update_status)
        update_trajectory(left, right)
        insert_custom_message(f"[PC ➜ LoRa] {message}","sent")
        

    def forward():
        try:
            L = int(left_speed_entry.get().strip())
            R = int(right_speed_entry.get().strip())
            send_movement_command(L, -R)
        except ValueError:
            output_label.config(text="Invalid speed input", fg="red")

    def backward():
        try:
            L = int(left_speed_entry.get().strip())
            R = int(right_speed_entry.get().strip())
            send_movement_command(-L, R)
        except ValueError:
            output_label.config(text="Invalid speed input", fg="red")

    def left():
        try:
            L = int(left_speed_entry.get().strip())
            R = int(right_speed_entry.get().strip())
            send_movement_command(-L, -R)
        except ValueError:
            output_label.config(text="Invalid speed input", fg="red")

    def right():
        try:
            L = int(left_speed_entry.get().strip())
            R = int(right_speed_entry.get().strip())
            send_movement_command(L, R)
        except ValueError:
            output_label.config(text="Invalid speed input", fg="red")

    def stop():
        send_movement_command(0, 0)



    def toggle_theme():
        nonlocal dark_mode
        dark_mode = toggle_dark_mode(root, dark_mode)

    def handle_flask_command(command: str) -> bool:
        """
        Handles a command received by Flask. Returns True if recognised.
        """
        if command == "SPEED 0 0":
            stop()
        elif command == "SPEED -15000 15000":
            forward()
        elif command == "SPEED 15000 -15000":
            backward()
        elif command == "SPEED 15000 15000":
            right()
        elif command == "SPEED -15000 -15000":
            left()
        else:
            write_history_custom_message(command)
            return False
        return True

    def handle_flask_log(message: str):
        """
        Displays a Flask log in the history.
        """
        insert_custom_message(message,"received")

    register_command_handler(handle_flask_command)
    register_log_handler(handle_flask_log)
    start_flask_server()

    def update_status(text, color):
        output_label.config(text=text, fg=color)


    def write_history_custom_message(message : str):
        insert_custom_message(f"[PC ➜ LoRa] {message}","sent")
        send_custom_message(message,update_status)
        send_entry.delete(0,tk.END)


    # ===== Setting up frames =====
    port_frame = tk.Frame(root)
    port_frame.grid(row=0, column=0, columnspan=3, sticky="nsew",padx = 10, pady= 5)
    port_frame.grid_rowconfigure(0, weight=1)
    port_frame.grid_columnconfigure(0, weight=1)
    port_frame.grid_columnconfigure(1, weight=1)
    port_frame.configure(height=80)

    top_left_button_frame = tk.Frame(port_frame)
    top_left_button_frame.grid(row=0, column=0, sticky="nw", padx=5, pady=2.5)

    top_right_button_frame = tk.Frame(port_frame)
    top_right_button_frame.grid(row=0, column=1, sticky="ne", padx=5, pady=5)
    

    send_frame = tk.Frame(root)
    send_frame.grid(row=1, column=1, sticky="nsew", padx = 10, pady= 5)
    for i in range(3):
        send_frame.grid_columnconfigure(i, weight=1, uniform="equal")


    cross_frame = tk.Frame(root)
    cross_frame.grid(row=2, column=1, sticky="nsew", padx = 10, pady= 5)
    for i in range(3):
        cross_frame.grid_columnconfigure(i, weight=1, uniform="equal")
    for i in range(4):
        cross_frame.grid_rowconfigure(i, weight=1, uniform="equal")


    info_frame = tk.Frame(root)
    info_frame.grid(row=3, column=0,columnspan=3,sticky="nsew", padx = 10, pady= 5)
    info_frame.grid_columnconfigure(0, weight=1)
    info_frame.grid_rowconfigure(0, weight=0)
    info_frame.grid_rowconfigure(1, weight=1)


    status_section_frame = tk.Frame(info_frame)
    status_section_frame.grid(row=0, column=0, pady=(0, 20))

    history_section_frame = tk.Frame(info_frame)
    history_section_frame.grid(row=1, column=0)

    top_row_frame = tk.Frame(history_section_frame)
    top_row_frame.pack(fill="x", pady=(0, 5))

    label_container_frame = tk.Frame(top_row_frame)
    label_container_frame.pack(side="top", expand=True)

    buttons_container_frame = tk.Frame(top_row_frame)
    buttons_container_frame.pack(side="right")

    history_frame = tk.Frame(history_section_frame)
    history_frame.pack()

    # ===== Widget =====

    refresh_button = tk.Button(top_left_button_frame, text="🔄", command=refresh_ports)
    refresh_button.pack(side="left")

    COM_port_label = tk.Label(top_left_button_frame, text="COM port :")
    COM_port_label.pack(side="left")

    com_ports = [port.device for port in serial.tools.list_ports.comports()]
    port_var = tk.StringVar()
    COM_port_combobox = ttk.Combobox(top_left_button_frame, textvariable=port_var, values=com_ports, state="readonly", width=15)
    COM_port_combobox.pack(side="left", padx=5, pady=5)

    if com_ports:
        COM_port_combobox.current(0)

    baudrates = ["300","1200","2400","4800","9600", "19200", "38400", "57600","74880","115200","230400","250000","500000","1000000","2000000"]
    baud_var = tk.StringVar(value="115200")

    baud_label = tk.Label(top_left_button_frame, text="Baudrate :")
    baud_label.pack(side="left", padx=5, pady=5)

    baud_combobox = ttk.Combobox(top_left_button_frame, textvariable=baud_var, values=baudrates, state="readonly", width=10)
    baud_combobox.pack(side="left", padx=5, pady=5)

    connect_button = tk.Button(top_left_button_frame, text="Connect",width=15, command=connect_serial)
    connect_button.pack(side="left", padx=5, pady=5)

    dark_button = tk.Button(top_right_button_frame, text="Night mode",width=15, command=toggle_theme)
    dark_button.grid(row=0, column=0, padx=5, pady=5)

    open_camera_button = tk.Button(top_right_button_frame,text='Open camera',width=15,command=lambda: open_camera_window(root))
    open_camera_button.grid(row=0,column=1,padx=5,pady=5)

    send_entry = tk.Entry(send_frame, width=50)
    send_entry.grid(row=0, column=1, padx=5, pady=5, sticky="ew")

    send_button = tk.Button(send_frame, text="Send", command=lambda: write_history_custom_message(send_entry.get()),width = 15)
    send_button.grid(row=1, column=1, pady=5)

    forward_button = tk.Button(cross_frame, text="Forward", width=15, command=forward)
    forward_button.grid(row=1, column=1,padx = 2.5, pady= 2)

    left_button = tk.Button(cross_frame, text="Left", width=15, command=left)
    left_button.grid(row=2, column=0,padx = 2.5, pady= 2)

    stop_button = tk.Button(cross_frame, text="Stop", width=15, command=stop)
    stop_button.grid(row=2, column=1,padx = 2.5, pady= 2)

    right_button = tk.Button(cross_frame, text="Right", width=15, command=right)
    right_button.grid(row=2, column=2,padx = 2.5, pady= 2)

    backward_button = tk.Button(cross_frame, text="Backward", width=15, command=backward)
    backward_button.grid(row=3, column=1,padx = 2.5, pady= 2)

    left_speed_label=tk.Label(cross_frame, text="Left Speed:")
    left_speed_label.grid(row=0, column=0,padx = 2.5, pady= 2)

    left_speed_var = tk.StringVar(value="15000")
    left_speed_entry=tk.Entry(cross_frame, textvariable=left_speed_var, width=7)
    left_speed_entry.grid(row=1, column=0,padx = 2.5, pady= 2)

    right_speed_label=tk.Label(cross_frame, text="Right Speed:")
    right_speed_label.grid(row=0, column=2,padx = 2.5, pady= 2)

    right_speed_var = tk.StringVar(value="15000")
    right_speed_entry=tk.Entry(cross_frame, textvariable=right_speed_var, width=7)
    right_speed_entry.grid(row=1, column=2,padx = 2.5, pady= 2)

    status_label = tk.Label(status_section_frame, text="Status :", font=("Arial", 12, "bold"))
    status_label.pack(anchor="center")

    output_label = tk.Label(status_section_frame, text="Waiting for connexion...", anchor="center")
    output_label.pack(anchor="center")

    text_label = tk.Label(label_container_frame, text="History :", font=("Arial", 12, "bold"))
    text_label.pack(anchor="center")

    save_button = tk.Button(buttons_container_frame, text="Save Log", command=save_history_as, width=15)
    save_button.pack(side="left", padx=5)

    reset_button = tk.Button(buttons_container_frame, text="Reset History", command=reset_history, width=15, fg="red")
    reset_button.pack(side="left", padx=5)

    text_history = tk.Text(history_frame, height=12, width=80)
    text_history.tag_config("sent", foreground="blue")
    text_history.tag_config("received", foreground="green")
    text_history.tag_config("error", foreground="red")
    text_history.pack(side="left", fill="both", expand=True)

    scrollbar = tk.Scrollbar(history_frame, command=text_history.yview)
    scrollbar.pack(side="right", fill="y")

    text_history.config(yscrollcommand=scrollbar.set)

    root.bind('<Return>',lambda event: write_history_custom_message(send_entry.get()))
    root.mainloop()