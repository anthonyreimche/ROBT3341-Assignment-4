import tkinter as tk
from tkinter import ttk

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from jointInterpolation import jointInterpolation
from motion import motion
from trajectory import profile


class MotionProfileUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Assignment #4")
        self.root.configure(bg="#1e1e1e")

        # Configure style
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TNotebook", background="#2d2d30", borderwidth=0)
        style.configure(
            "TNotebook.Tab",
            background="#2d2d30",
            foreground="#969696",
            padding=[24, 10],
            borderwidth=0,
            width=20,
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", "#1e1e1e")],
            foreground=[("selected", "#ffffff")],
            padding=[("selected", [24, 10])],
        )
        style.configure("TFrame", background="#252526")
        style.configure(
            "TLabel", background="#252526", foreground="#cccccc", font=("Arial", 10)
        )
        style.configure(
            "TButton",
            background="#0e639c",
            foreground="#ffffff",
            font=("Arial", 10),
            borderwidth=0,
            padding=[16, 8],
        )

        # Header
        header = tk.Frame(root, bg="#2d2d30", height=60)
        header.pack(fill="x", side="top")
        header.pack_propagate(False)

        title = tk.Label(
            header,
            text="Motion Profile Plotter",
            bg="#2d2d30",
            fg="#cccccc",
            font=("Arial", 14),
        )
        title.pack(side="left", padx=24, pady=8)

        subtitle = tk.Label(
            header,
            text="ROBT 3341",
            bg="#2d2d30",
            fg="#858585",
            font=("Arial", 10),
        )
        subtitle.pack(side="left", padx=(0, 24))

        # Notebook (tabs)
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill="both", expand=True)

        # Create tabs
        self.create_profile_tab()
        self.create_motion_tab()
        self.create_joint_tab()

    def create_profile_tab(self):
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Profile")

        # Left panel
        left = tk.Frame(frame, bg="#252526", width=320)
        left.pack(side="left", fill="y", padx=0, pady=0)
        left.pack_propagate(False)

        section = tk.Label(
            left,
            text="Parameters",
            bg="#252526",
            fg="#cccccc",
            font=("Arial", 11),
            anchor="w",
        )
        section.pack(fill="x", padx=24, pady=(24, 16))

        # Inputs
        self.p_displacement = self.create_input(left, "Displacement", "100")
        self.p_start = self.create_input(left, "Start Position", "0")
        self.p_total_time = self.create_input(left, "Total Time", "2")
        self.p_ta = self.create_input(left, "Acceleration Time (Ta)", "0.5")
        self.p_interval = self.create_input(left, "Time Interval", "0.1")

        btn = tk.Button(
            left,
            text="Generate",
            bg="#0e639c",
            fg="#ffffff",
            font=("Arial", 11),
            relief="flat",
            cursor="hand2",
            command=self.plot_profile,
        )
        btn.pack(fill="x", padx=24, pady=(24, 0))

        # Right panel
        right = tk.Frame(frame, bg="#1e1e1e")
        right.pack(side="right", fill="both", expand=True)

        # Create scrollable canvas
        canvas = tk.Canvas(right, bg="#1e1e1e", highlightthickness=0)
        scrollbar = tk.Scrollbar(
            right,
            orient="vertical",
            command=canvas.yview,
            bg="#3e3e42",
            troughcolor="#1e1e1e",
            activebackground="#4e4e52",
        )
        scrollable = tk.Frame(canvas, bg="#1e1e1e")

        scrollable.bind(
            "<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window(
            (0, 0), window=scrollable, anchor="nw", width=canvas.winfo_width()
        )
        canvas.configure(yscrollcommand=scrollbar.set)

        def on_canvas_configure(event):
            canvas.itemconfig(canvas.find_withtag("all")[0], width=event.width)

        canvas.bind("<Configure>", on_canvas_configure)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Enable mousewheel scrolling
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        canvas.bind_all("<MouseWheel>", _on_mousewheel)

        self.profile_canvas_frame = scrollable

    def create_motion_tab(self):
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Motion")

        # Left panel
        left = tk.Frame(frame, bg="#252526", width=320)
        left.pack(side="left", fill="y")
        left.pack_propagate(False)

        section = tk.Label(
            left,
            text="Parameters",
            bg="#252526",
            fg="#cccccc",
            font=("Arial", 11),
            anchor="w",
        )
        section.pack(fill="x", padx=24, pady=(24, 16))

        # Inputs
        self.m_displacement = self.create_input(left, "Displacement", "100")
        self.m_start = self.create_input(left, "Start Position", "0")
        self.m_interval = self.create_input(left, "Time Interval", "0.1")
        self.m_accel_limit = self.create_input(left, "Acceleration Limit", "50")
        self.m_velo_limit = self.create_input(left, "Velocity Limit", "100")

        btn = tk.Button(
            left,
            text="Generate",
            bg="#0e639c",
            fg="#ffffff",
            font=("Arial", 11),
            relief="flat",
            cursor="hand2",
            command=self.plot_motion,
        )
        btn.pack(fill="x", padx=24, pady=(24, 0))

        # Right panel
        right = tk.Frame(frame, bg="#1e1e1e")
        right.pack(side="right", fill="both", expand=True)

        # Create scrollable canvas
        canvas = tk.Canvas(right, bg="#1e1e1e", highlightthickness=0)
        scrollbar = tk.Scrollbar(
            right,
            orient="vertical",
            command=canvas.yview,
            bg="#3e3e42",
            troughcolor="#1e1e1e",
            activebackground="#4e4e52",
        )
        scrollable = tk.Frame(canvas, bg="#1e1e1e")

        scrollable.bind(
            "<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window(
            (0, 0), window=scrollable, anchor="nw", width=canvas.winfo_width()
        )
        canvas.configure(yscrollcommand=scrollbar.set)

        def on_canvas_configure(event):
            canvas.itemconfig(canvas.find_withtag("all")[0], width=event.width)

        canvas.bind("<Configure>", on_canvas_configure)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Enable mousewheel scrolling
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        canvas.bind_all("<MouseWheel>", _on_mousewheel)

        self.motion_info = tk.Label(
            scrollable,
            text="",
            bg="#252526",
            fg="#4fc3f7",
            font=("Arial", 10),
            anchor="w",
            relief="flat",
        )
        self.motion_info.pack(fill="x", padx=24, pady=(24, 16))

        self.motion_canvas_frame = scrollable

    def create_joint_tab(self):
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Joint Coordination")

        # Left panel
        left = tk.Frame(frame, bg="#252526", width=320)
        left.pack(side="left", fill="y")
        left.pack_propagate(False)

        # Scrollable frame
        canvas = tk.Canvas(left, bg="#252526", highlightthickness=0)
        scrollbar = tk.Scrollbar(left, orient="vertical", command=canvas.yview)
        scrollable = tk.Frame(canvas, bg="#252526")

        scrollable.bind(
            "<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        section = tk.Label(
            scrollable,
            text="Parameters",
            bg="#252526",
            fg="#cccccc",
            font=("Arial", 11),
            anchor="w",
        )
        section.pack(fill="x", padx=24, pady=(24, 16))

        # Joint A
        joint_a = tk.Label(
            scrollable,
            text="Joint A",
            bg="#252526",
            fg="#4fc3f7",
            font=("Arial", 10, "bold"),
            anchor="w",
        )
        joint_a.pack(fill="x", padx=24, pady=(0, 12))

        self.j_displacement_a = self.create_input(scrollable, "Displacement A", "100")
        self.j_start_a = self.create_input(scrollable, "Start A", "0")
        self.j_accel_a = self.create_input(scrollable, "Acceleration Limit A", "100")
        self.j_velo_a = self.create_input(scrollable, "Velocity Limit A", "200")

        # Separator
        sep = tk.Frame(scrollable, bg="#3e3e42", height=1)
        sep.pack(fill="x", padx=24, pady=24)

        # Joint B
        joint_b = tk.Label(
            scrollable,
            text="Joint B",
            bg="#252526",
            fg="#4fc3f7",
            font=("Arial", 10, "bold"),
            anchor="w",
        )
        joint_b.pack(fill="x", padx=24, pady=(0, 12))

        self.j_displacement_b = self.create_input(scrollable, "Displacement B", "50")
        self.j_start_b = self.create_input(scrollable, "Start B", "0")
        self.j_accel_b = self.create_input(scrollable, "Acceleration Limit B", "80")
        self.j_velo_b = self.create_input(scrollable, "Velocity Limit B", "150")

        # Separator
        sep2 = tk.Frame(scrollable, bg="#3e3e42", height=1)
        sep2.pack(fill="x", padx=24, pady=24)

        self.j_interval = self.create_input(scrollable, "Time Interval", "0.1")

        btn = tk.Button(
            scrollable,
            text="Generate",
            bg="#0e639c",
            fg="#ffffff",
            font=("Arial", 11),
            relief="flat",
            cursor="hand2",
            command=self.plot_joint,
        )
        btn.pack(fill="x", padx=24, pady=(24, 24))

        # Right panel
        right = tk.Frame(frame, bg="#1e1e1e")
        right.pack(side="right", fill="both", expand=True)

        # Create scrollable canvas
        canvas_right = tk.Canvas(right, bg="#1e1e1e", highlightthickness=0)
        scrollbar_right = tk.Scrollbar(
            right,
            orient="vertical",
            command=canvas_right.yview,
            bg="#3e3e42",
            troughcolor="#1e1e1e",
            activebackground="#4e4e52",
        )
        scrollable_right = tk.Frame(canvas_right, bg="#1e1e1e")

        scrollable_right.bind(
            "<Configure>",
            lambda e: canvas_right.configure(scrollregion=canvas_right.bbox("all")),
        )

        canvas_right.create_window(
            (0, 0),
            window=scrollable_right,
            anchor="nw",
            width=canvas_right.winfo_width(),
        )
        canvas_right.configure(yscrollcommand=scrollbar_right.set)

        def on_canvas_configure(event):
            canvas_right.itemconfig(
                canvas_right.find_withtag("all")[0], width=event.width
            )

        canvas_right.bind("<Configure>", on_canvas_configure)

        canvas_right.pack(side="left", fill="both", expand=True)
        scrollbar_right.pack(side="right", fill="y")

        # Enable mousewheel scrolling
        def _on_mousewheel(event):
            canvas_right.yview_scroll(int(-1 * (event.delta / 120)), "units")

        canvas_right.bind_all("<MouseWheel>", _on_mousewheel)

        self.joint_info = tk.Label(
            scrollable_right,
            text="",
            bg="#252526",
            fg="#4fc3f7",
            font=("Arial", 10),
            anchor="w",
        )
        self.joint_info.pack(fill="x", padx=24, pady=(24, 16))

        self.joint_canvas_frame = scrollable_right

    def create_input(self, parent, label_text, default_value):
        container = tk.Frame(parent, bg="#252526")
        container.pack(fill="x", padx=24, pady=(0, 16))

        label = tk.Label(
            container,
            text=label_text,
            bg="#252526",
            fg="#cccccc",
            font=("Arial", 9),
            anchor="w",
        )
        label.pack(fill="x", pady=(0, 6))

        entry = tk.Entry(
            container,
            bg="#3c3c3c",
            fg="#cccccc",
            font=("Arial", 10),
            relief="flat",
            insertbackground="#cccccc",
            borderwidth=1,
        )
        entry.insert(0, default_value)
        entry.pack(fill="x", ipady=4)

        return entry

    def clear_plots(self, frame):
        for widget in frame.winfo_children():
            if isinstance(widget, tk.Frame) and hasattr(widget, "canvas"):
                widget.destroy()

    def create_plot(self, parent, t, data, title, ylabel, color):
        plot_frame = tk.Frame(
            parent, bg="#252526", highlightbackground="#3e3e42", highlightthickness=1
        )
        plot_frame.pack(fill="both", expand=True, padx=24, pady=(0, 24))

        fig = Figure(figsize=(8, 3), facecolor="#252526")
        ax = fig.add_subplot(111, facecolor="#1e1e1e")

        ax.plot(t, data, color=color, linewidth=2, alpha=0.8)
        ax.fill_between(t, data, alpha=0.1, color=color)

        ax.set_title(title, color="#cccccc", fontsize=11, pad=10)
        ax.set_xlabel("Time (s)", color="#858585", fontsize=9)
        ax.set_ylabel(ylabel, color="#858585", fontsize=9)

        ax.tick_params(colors="#858585", labelsize=8)
        ax.grid(True, color="#3e3e42", linewidth=0.5, alpha=0.5)
        ax.spines["bottom"].set_color("#3e3e42")
        ax.spines["top"].set_color("#3e3e42")
        ax.spines["left"].set_color("#3e3e42")
        ax.spines["right"].set_color("#3e3e42")

        fig.tight_layout()

        canvas = FigureCanvasTkAgg(fig, plot_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)
        plot_frame.canvas = canvas

    def create_dual_plot(self, parent, t, data_a, data_b, title, ylabel):
        plot_frame = tk.Frame(
            parent, bg="#252526", highlightbackground="#3e3e42", highlightthickness=1
        )
        plot_frame.pack(fill="both", expand=True, padx=24, pady=(0, 24))

        fig = Figure(figsize=(8, 3), facecolor="#252526")
        ax = fig.add_subplot(111, facecolor="#1e1e1e")

        ax.plot(t, data_a, color="#4fc3f7", linewidth=2, alpha=0.8, label="Joint A")
        ax.plot(t, data_b, color="#ce93d8", linewidth=2, alpha=0.8, label="Joint B")
        ax.fill_between(t, data_a, alpha=0.1, color="#4fc3f7")
        ax.fill_between(t, data_b, alpha=0.1, color="#ce93d8")

        ax.set_title(title, color="#cccccc", fontsize=11, pad=10)
        ax.set_xlabel("Time (s)", color="#858585", fontsize=9)
        ax.set_ylabel(ylabel, color="#858585", fontsize=9)

        ax.tick_params(colors="#858585", labelsize=8)
        ax.grid(True, color="#3e3e42", linewidth=0.5, alpha=0.5)
        ax.legend(
            facecolor="#252526", edgecolor="#3e3e42", labelcolor="#cccccc", fontsize=8
        )
        ax.spines["bottom"].set_color("#3e3e42")
        ax.spines["top"].set_color("#3e3e42")
        ax.spines["left"].set_color("#3e3e42")
        ax.spines["right"].set_color("#3e3e42")

        fig.tight_layout()

        canvas = FigureCanvasTkAgg(fig, plot_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)
        plot_frame.canvas = canvas

    def plot_profile(self):
        d = float(self.p_displacement.get())
        s = float(self.p_start.get())
        tt = float(self.p_total_time.get())
        ta = float(self.p_ta.get())
        interval = float(self.p_interval.get())

        t = []
        c = 0
        while c <= tt:
            t.append(c)
            c += interval
        if not t or t[-1] < tt:
            t.append(tt)

        pos, vel, acc = profile(d, s, t, ta)

        self.clear_plots(self.profile_canvas_frame)

        self.create_plot(
            self.profile_canvas_frame, t, pos, "Position", "Position", "#4fc3f7"
        )
        self.create_plot(
            self.profile_canvas_frame, t, vel, "Velocity", "Velocity", "#ce93d8"
        )
        self.create_plot(
            self.profile_canvas_frame, t, acc, "Acceleration", "Acceleration", "#80cbc4"
        )

    def plot_motion(self):
        d = float(self.m_displacement.get())
        s = float(self.m_start.get())
        interval = float(self.m_interval.get())
        al = float(self.m_accel_limit.get())
        vl = float(self.m_velo_limit.get())

        t, ta = motion(d, interval, al, vl)
        pos, vel, acc = profile(d, s, t, ta)

        self.motion_info.config(text=f"  Ta: {ta:.3f}s | Total: {t[-1]:.3f}s  ")

        self.clear_plots(self.motion_canvas_frame)

        self.create_plot(
            self.motion_canvas_frame, t, pos, "Position", "Position", "#4fc3f7"
        )
        self.create_plot(
            self.motion_canvas_frame, t, vel, "Velocity", "Velocity", "#ce93d8"
        )
        self.create_plot(
            self.motion_canvas_frame, t, acc, "Acceleration", "Acceleration", "#80cbc4"
        )

    def plot_joint(self):
        da = float(self.j_displacement_a.get())
        sa = float(self.j_start_a.get())
        db = float(self.j_displacement_b.get())
        sb = float(self.j_start_b.get())
        interval = float(self.j_interval.get())
        ala = float(self.j_accel_a.get())
        vla = float(self.j_velo_a.get())
        alb = float(self.j_accel_b.get())
        vlb = float(self.j_velo_b.get())

        eoma, eomb, t = jointInterpolation(da, sa, db, sb, interval, ala, vla, alb, vlb)

        self.joint_info.config(text=f"  Total: {t[-1]:.3f}s  ")

        self.clear_plots(self.joint_canvas_frame)

        self.create_dual_plot(
            self.joint_canvas_frame, t, eoma[0], eomb[0], "Position", "Position"
        )
        self.create_dual_plot(
            self.joint_canvas_frame, t, eoma[1], eomb[1], "Velocity", "Velocity"
        )
        self.create_dual_plot(
            self.joint_canvas_frame, t, eoma[2], eomb[2], "Acceleration", "Acceleration"
        )


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1400x900")
    app = MotionProfileUI(root)

    # Load first plot
    root.after(100, app.plot_profile)

    root.mainloop()
