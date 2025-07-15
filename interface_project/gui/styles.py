# ============================================================
# Author      : Baptiste Poncet
# Date        : 07/07/2025
# File        : style.py
# Description : Management of interface dark mode (Tkinter)
# ============================================================


import tkinter as tk
from tkinter import ttk

def toggle_dark_mode(root, dark_mode_flag: bool) -> bool:
    """
    Applique ou retire le mode sombre à tous les widgets enfants de root.
    Renvoie le nouveau statut de dark_mode_flag (True si activé).
    """

    dark_mode_flag = not dark_mode_flag  # Reverse the flag

    # ===== Colors =====
    bg_color = "#2e2e2e" if dark_mode_flag else "SystemButtonFace"
    fg_color = "#f0f0f0" if dark_mode_flag else "black"
    entry_bg = "#3e3e3e" if dark_mode_flag else "white"
    button_bg = "#444444" if dark_mode_flag else "SystemButtonFace"

    # ===== Applies to all widgets =====
    def apply_theme(widget):
        try:
            cls = widget.__class__.__name__

            if isinstance(widget, (tk.Frame, tk.LabelFrame)):
                widget.configure(bg=bg_color)

            elif isinstance(widget, tk.Label):
                widget.configure(bg=bg_color, fg=fg_color)

            elif isinstance(widget, tk.Entry):
                widget.configure(bg=entry_bg, fg=fg_color, insertbackground=fg_color)

            elif isinstance(widget, tk.Text):
                widget.configure(bg=entry_bg, fg=fg_color, insertbackground=fg_color)

            elif isinstance(widget, tk.Button):
                widget.configure(bg=button_bg, fg=fg_color, activebackground=button_bg, activeforeground=fg_color)

            elif isinstance(widget, tk.Checkbutton) or isinstance(widget, tk.Radiobutton):
                widget.configure(bg=bg_color, fg=fg_color, selectcolor=bg_color)

        except:
            pass  # Certains widgets peuvent ne pas accepter certaines options

        # ===== Recurse on children =====
        for child in widget.winfo_children():
            apply_theme(child)

    # ===== Apply to root and all its children =====
    root.configure(bg=bg_color)
    apply_theme(root)

    # ===== Style ttk =====
    style = ttk.Style()
    style.theme_use("clam")

    if dark_mode_flag:
        style.configure("TCombobox",
                        fieldbackground=entry_bg,
                        background=bg_color,
                        foreground=fg_color,
                        arrowcolor=fg_color)
        style.map("TCombobox",
                  fieldbackground=[("readonly", entry_bg)],
                  background=[("readonly", bg_color)],
                  foreground=[("readonly", fg_color)])
    else:
        style.configure("TCombobox",
                        fieldbackground="white",
                        background="SystemButtonFace",
                        foreground="black",
                        arrowcolor="black")
        style.map("TCombobox",
                  fieldbackground=[("readonly", "white")],
                  background=[("readonly", "SystemButtonFace")],
                  foreground=[("readonly", "black")])

    return dark_mode_flag
