# ============================================================
# Author      : Baptiste Poncet
# Date        : 07/07/2025
# File        : main.py
# Description : Main entry point to the application (launches the GUI interface)
# ============================================================


import sys
import os

interface_project_path = os.path.abspath(
    os.path.join(os.path.dirname(__file__), 'interface_project')
)

if interface_project_path not in sys.path:
    sys.path.append(interface_project_path)

from gui.main_window import run_gui

run_gui()