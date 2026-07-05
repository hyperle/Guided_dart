from __future__ import annotations

import math
import os
import statistics
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import cv2
import numpy as np
from PIL import Image, ImageTk

from guidance_tuner_core import (
    CV_FONT,
    LOCAL_PREVIEW_NOTE,
    ColorSample,
    DetectionDebug,
    DetectorParams,
    SimulationParams,
    SweepParams,
    ValidationFrameResult,
    ValidationRunConfig,
    ValidationSummary,
    apply_simulation,
    clamp,
    gray_to_bgr,
    load_detector_defaults,
    evaluate_host_frame,
    point_in_roi,
    resize_for_display,
    roi_from_drag,
    sample_color_circle,
    scale_for_display,
    threshold_lab_mask,
)
from guidance_tuner_openmv import OpenMvEvaluator
from guidance_tuner_video import VideoSource, load_video_source


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))


def bgr_to_photo(image_bgr: np.ndarray) -> ImageTk.PhotoImage:
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    return ImageTk.PhotoImage(Image.fromarray(image_rgb))


class VideoTunerApp:
    def __init__(self, root: tk.Tk, video_path: str, config_path: str, port: str = "", baudrate: int = 0):
        self.root = root
        self.root.title("Guidance Video Tuner")
        self.root.geometry("1800x980")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.config_path = config_path
        self.video: VideoSource | None = None
        self.expected_roi: tuple[int, int, int, int] | None = None
        self.drag_start: tuple[int, int] | None = None
        self.drag_current: tuple[int, int] | None = None
        self.single_display_scale = 1.0
        self.single_frame_result: DetectionDebug | None = None
        self.host_frame_result: DetectionDebug | None = None
        self.color_sample_center: tuple[int, int] | None = None
        self.color_sample: ColorSample | None = None
        self.validation_summary: ValidationSummary | None = None
        self.openmv_evaluator = OpenMvEvaluator(REPO_ROOT, config_path, port_override=port, baudrate_override=baudrate)

        self._adjusted_photo: ImageTk.PhotoImage | None = None
        self._mask_photo: ImageTk.PhotoImage | None = None
        self._overlay_photo: ImageTk.PhotoImage | None = None
        self._validation_photo: ImageTk.PhotoImage | None = None
        self._refresh_single_job: str | None = None
        self._validation_thread: threading.Thread | None = None
        self._validation_in_progress = False

        self.status_var = tk.StringVar(value="Open a video to start.")
        self.video_info_var = tk.StringVar(value="No video loaded")
        self.validation_progress_var = tk.StringVar(value="")
        self.openmv_status_var = tk.StringVar(value="OpenMV: disconnected")

        defaults = load_detector_defaults(config_path)
        self.frame_index_var = tk.IntVar(value=0)
        self.validation_frame_var = tk.IntVar(value=0)
        self.interaction_mode_var = tk.StringVar(value="roi")
        self.sample_radius_var = tk.IntVar(value=8)

        self.exposure_scale_var = tk.DoubleVar(value=1.0)
        self.r_gain_var = tk.DoubleVar(value=1.0)
        self.g_gain_var = tk.DoubleVar(value=1.0)
        self.b_gain_var = tk.DoubleVar(value=1.0)
        self.gamma_var = tk.DoubleVar(value=1.0)

        self.threshold_l_min_var = tk.IntVar(value=defaults.threshold_l_min)
        self.threshold_l_max_var = tk.IntVar(value=defaults.threshold_l_max)
        self.threshold_a_min_var = tk.IntVar(value=defaults.threshold_a_min)
        self.threshold_a_max_var = tk.IntVar(value=defaults.threshold_a_max)
        self.threshold_b_min_var = tk.IntVar(value=defaults.threshold_b_min)
        self.threshold_b_max_var = tk.IntVar(value=defaults.threshold_b_max)
        self.min_area_var = tk.IntVar(value=defaults.min_area)
        self.max_area_var = tk.IntVar(value=defaults.max_area)
        self.roundness_min_var = tk.IntVar(value=defaults.roundness_min_x1000)
        self.merge_margin_var = tk.IntVar(value=defaults.merge_margin)
        self.track_window_radius_var = tk.IntVar(value=defaults.track_window_radius_px)
        self.center_filter_gain_var = tk.IntVar(value=defaults.center_filter_gain_x100)
        self.max_missed_frames_var = tk.IntVar(value=defaults.max_missed_frames)
        self.ring_detection_enabled_var = tk.IntVar(value=defaults.ring_detection_enabled)
        self.ring_min_roundness_var = tk.IntVar(value=defaults.ring_min_roundness_x1000)
        self.ring_min_aspect_var = tk.IntVar(value=defaults.ring_min_aspect_x100)
        self.ring_min_fill_var = tk.IntVar(value=defaults.ring_min_fill_x100)
        self.ring_max_fill_var = tk.IntVar(value=defaults.ring_max_fill_x100)
        self.ring_min_center_white_var = tk.IntVar(value=defaults.ring_min_center_white_x100)
        self.ring_center_sample_ratio_var = tk.IntVar(value=defaults.ring_center_sample_ratio_x100)
        self.ring_center_min_brightness_var = tk.IntVar(value=defaults.ring_center_min_brightness)
        self.ring_center_max_channel_delta_var = tk.IntVar(value=defaults.ring_center_max_channel_delta)
        self.ring_min_outer_diameter_var = tk.IntVar(value=defaults.ring_min_outer_diameter_px)

        self.sweep_exposure_min_var = tk.DoubleVar(value=0.8)
        self.sweep_exposure_max_var = tk.DoubleVar(value=1.4)
        self.sweep_exposure_step_var = tk.DoubleVar(value=0.05)
        self.sweep_gain_min_var = tk.DoubleVar(value=0.8)
        self.sweep_gain_max_var = tk.DoubleVar(value=1.4)
        self.sweep_gain_step_var = tk.DoubleVar(value=0.05)

        self._build_ui()
        if video_path:
            self.load_video(video_path)

    def _build_ui(self) -> None:
        root_frame = ttk.Frame(self.root)
        root_frame.pack(fill=tk.BOTH, expand=True)
        root_frame.columnconfigure(0, weight=0)
        root_frame.columnconfigure(1, weight=1)
        root_frame.rowconfigure(0, weight=1)

        control_container = ttk.Frame(root_frame)
        control_container.grid(row=0, column=0, sticky="nsw")

        control_canvas = tk.Canvas(control_container, width=380, highlightthickness=0)
        control_scroll = ttk.Scrollbar(control_container, orient=tk.VERTICAL, command=control_canvas.yview)
        self.control_frame = ttk.Frame(control_canvas)
        self.control_frame.bind(
            "<Configure>",
            lambda event: control_canvas.configure(scrollregion=control_canvas.bbox("all")),
        )
        control_canvas.create_window((0, 0), window=self.control_frame, anchor="nw")
        control_canvas.configure(yscrollcommand=control_scroll.set)
        control_canvas.pack(side=tk.LEFT, fill=tk.Y, expand=False)
        control_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        main_panel = ttk.Frame(root_frame)
        main_panel.grid(row=0, column=1, sticky="nsew")
        main_panel.columnconfigure(0, weight=1)
        main_panel.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(main_panel)
        toolbar.grid(row=0, column=0, sticky="ew", padx=8, pady=6)
        toolbar.columnconfigure(4, weight=1)
        ttk.Button(toolbar, text="Open Video", command=self.choose_video).grid(row=0, column=0, padx=(0, 6))
        ttk.Button(toolbar, text="Connect OpenMV", command=self.connect_openmv).grid(row=0, column=1, padx=(0, 6))
        ttk.Button(toolbar, text="Disconnect", command=self.disconnect_openmv).grid(row=0, column=2, padx=(0, 6))
        ttk.Button(toolbar, text="Copy YAML Snippet", command=self.copy_yaml_snippet).grid(row=0, column=3, padx=(0, 6))
        ttk.Label(toolbar, textvariable=self.video_info_var).grid(row=0, column=4, sticky="w")
        ttk.Label(toolbar, textvariable=self.openmv_status_var).grid(row=0, column=5, sticky="e")

        self.notebook = ttk.Notebook(main_panel)
        self.notebook.grid(row=1, column=0, sticky="nsew", padx=8, pady=(0, 8))
        self.single_tab = ttk.Frame(self.notebook)
        self.validation_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.single_tab, text="Single Frame")
        self.notebook.add(self.validation_tab, text="Validation")

        self._build_control_panel()
        self._build_single_tab()
        self._build_validation_tab()

        status_bar = ttk.Label(self.root, textvariable=self.status_var, anchor="w")
        status_bar.pack(fill=tk.X, padx=8, pady=(0, 6))

    def _build_control_panel(self) -> None:
        file_box = ttk.LabelFrame(self.control_frame, text="Video / Tools")
        file_box.pack(fill=tk.X, padx=8, pady=8)
        ttk.Button(file_box, text="Open Video...", command=self.choose_video).pack(fill=tk.X, padx=6, pady=4)
        ttk.Button(file_box, text="Clear ROI", command=self.clear_roi).pack(fill=tk.X, padx=6, pady=4)
        ttk.Button(file_box, text="Clear Sample", command=self.clear_sample).pack(fill=tk.X, padx=6, pady=4)

        mode_frame = ttk.Frame(file_box)
        mode_frame.pack(fill=tk.X, padx=6, pady=(4, 0))
        ttk.Label(mode_frame, text="Mouse Mode", width=18).pack(side=tk.LEFT)
        ttk.Radiobutton(
            mode_frame,
            text="ROI",
            value="roi",
            variable=self.interaction_mode_var,
        ).pack(side=tk.LEFT)
        ttk.Radiobutton(
            mode_frame,
            text="Sample",
            value="sample",
            variable=self.interaction_mode_var,
        ).pack(side=tk.LEFT, padx=(8, 0))
        self._add_scale(file_box, "Sample Radius", self.sample_radius_var, 0, 40, 1)
        ttk.Label(file_box, text=f"Config: {self.config_path}", wraplength=340).pack(fill=tk.X, padx=6, pady=(4, 6))
        ttk.Label(file_box, text=LOCAL_PREVIEW_NOTE, wraplength=340).pack(fill=tk.X, padx=6, pady=(0, 6))

        sim_box = ttk.LabelFrame(self.control_frame, text="Camera Simulation")
        sim_box.pack(fill=tk.X, padx=8, pady=8)
        self._add_scale(sim_box, "Exposure Scale", self.exposure_scale_var, 0.2, 3.0, 0.01)
        self._add_scale(sim_box, "R Gain", self.r_gain_var, 0.2, 3.0, 0.01)
        self._add_scale(sim_box, "G Gain", self.g_gain_var, 0.2, 3.0, 0.01)
        self._add_scale(sim_box, "B Gain", self.b_gain_var, 0.2, 3.0, 0.01)
        self._add_scale(sim_box, "Gamma", self.gamma_var, 0.2, 3.0, 0.01)

        detector_box = ttk.LabelFrame(self.control_frame, text="Detector Thresholds")
        detector_box.pack(fill=tk.X, padx=8, pady=8)
        self._add_scale(detector_box, "L Min", self.threshold_l_min_var, 0, 100, 1)
        self._add_scale(detector_box, "L Max", self.threshold_l_max_var, 0, 100, 1)
        self._add_scale(detector_box, "A Min", self.threshold_a_min_var, -128, 127, 1)
        self._add_scale(detector_box, "A Max", self.threshold_a_max_var, -128, 127, 1)
        self._add_scale(detector_box, "B Min", self.threshold_b_min_var, -128, 127, 1)
        self._add_scale(detector_box, "B Max", self.threshold_b_max_var, -128, 127, 1)
        self._add_scale(detector_box, "Min Area", self.min_area_var, 1, 20000, 1)
        self._add_scale(detector_box, "Max Area", self.max_area_var, 0, 50000, 1)
        self._add_scale(detector_box, "Roundness x1000", self.roundness_min_var, 0, 1000, 1)
        self._add_scale(detector_box, "Merge Margin", self.merge_margin_var, 0, 50, 1)

        ring_box = ttk.LabelFrame(self.control_frame, text="Saturated Ring")
        ring_box.pack(fill=tk.X, padx=8, pady=8)
        self._add_scale(ring_box, "Ring Enabled", self.ring_detection_enabled_var, 0, 1, 1)
        self._add_scale(ring_box, "Ring Round x1000", self.ring_min_roundness_var, 0, 1000, 1)
        self._add_scale(ring_box, "Ring Aspect x100", self.ring_min_aspect_var, 1, 100, 1)
        self._add_scale(ring_box, "Ring Fill Min", self.ring_min_fill_var, 0, 100, 1)
        self._add_scale(ring_box, "Ring Fill Max", self.ring_max_fill_var, 0, 100, 1)
        self._add_scale(ring_box, "White Center Min", self.ring_min_center_white_var, 0, 100, 1)
        self._add_scale(ring_box, "Center Sample", self.ring_center_sample_ratio_var, 5, 100, 1)
        self._add_scale(ring_box, "White Brightness", self.ring_center_min_brightness_var, 0, 255, 1)
        self._add_scale(ring_box, "White Delta Max", self.ring_center_max_channel_delta_var, 0, 255, 1)
        self._add_scale(ring_box, "Outer Diameter", self.ring_min_outer_diameter_var, 1, 240, 1)

        tracking_box = ttk.LabelFrame(self.control_frame, text="Tracking / Validation")
        tracking_box.pack(fill=tk.X, padx=8, pady=8)
        self._add_scale(tracking_box, "Track Radius", self.track_window_radius_var, 1, 200, 1)
        self._add_scale(tracking_box, "Center Filter x100", self.center_filter_gain_var, 0, 100, 1)
        self._add_scale(tracking_box, "Max Missed Frames", self.max_missed_frames_var, 1, 20, 1)

        sweep_box = ttk.LabelFrame(self.control_frame, text="Sweep Debug Mode")
        sweep_box.pack(fill=tk.X, padx=8, pady=8)
        self._add_scale(sweep_box, "Sweep Exposure Min", self.sweep_exposure_min_var, 0.2, 3.0, 0.01)
        self._add_scale(sweep_box, "Sweep Exposure Max", self.sweep_exposure_max_var, 0.2, 3.0, 0.01)
        self._add_scale(sweep_box, "Sweep Exposure Step", self.sweep_exposure_step_var, 0.01, 1.0, 0.01)
        self._add_scale(sweep_box, "Sweep Gain Min", self.sweep_gain_min_var, 0.2, 3.0, 0.01)
        self._add_scale(sweep_box, "Sweep Gain Max", self.sweep_gain_max_var, 0.2, 3.0, 0.01)
        self._add_scale(sweep_box, "Sweep Gain Step", self.sweep_gain_step_var, 0.01, 1.0, 0.01)

    def _add_scale(self, parent, label: str, variable: tk.Variable, minimum: float, maximum: float, resolution: float) -> None:
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, padx=6, pady=3)
        ttk.Label(frame, text=label, width=18).pack(side=tk.LEFT)
        scale = tk.Scale(
            frame,
            from_=minimum,
            to=maximum,
            resolution=resolution,
            orient=tk.HORIZONTAL,
            showvalue=False,
            variable=variable,
            command=lambda _value: self.schedule_single_refresh(),
            length=180,
        )
        scale.pack(side=tk.LEFT, fill=tk.X, expand=True)
        entry = ttk.Entry(frame, textvariable=variable, width=8)
        entry.pack(side=tk.LEFT, padx=(6, 0))
        entry.bind("<Return>", lambda _event: self.schedule_single_refresh())
        entry.bind("<FocusOut>", lambda _event: self.schedule_single_refresh())

    def _build_single_tab(self) -> None:
        self.single_tab.columnconfigure(0, weight=1)
        self.single_tab.rowconfigure(1, weight=1)
        top_bar = ttk.Frame(self.single_tab)
        top_bar.grid(row=0, column=0, sticky="ew", padx=8, pady=8)
        top_bar.columnconfigure(1, weight=1)
        ttk.Label(top_bar, text="Frame").grid(row=0, column=0, sticky="w")
        self.frame_slider = tk.Scale(
            top_bar,
            from_=0,
            to=0,
            resolution=1,
            orient=tk.HORIZONTAL,
            variable=self.frame_index_var,
            command=lambda _value: self.schedule_single_refresh(),
        )
        self.frame_slider.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Label(top_bar, textvariable=self.frame_index_var, width=8).grid(row=0, column=2, sticky="e")

        body = ttk.Frame(self.single_tab)
        body.grid(row=1, column=0, sticky="nsew", padx=8, pady=(0, 8))
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)
        body.columnconfigure(2, weight=1)
        body.rowconfigure(0, weight=1)
        body.rowconfigure(1, weight=0)

        adjusted_box = ttk.LabelFrame(body, text="Transformed Image / ROI")
        adjusted_box.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        adjusted_box.rowconfigure(0, weight=1)
        adjusted_box.columnconfigure(0, weight=1)
        self.adjusted_canvas = tk.Canvas(adjusted_box, bg="#202020", highlightthickness=0)
        self.adjusted_canvas.grid(row=0, column=0, sticky="nsew")
        self.adjusted_canvas.bind("<ButtonPress-1>", self.on_canvas_press)
        self.adjusted_canvas.bind("<B1-Motion>", self.on_canvas_drag)
        self.adjusted_canvas.bind("<ButtonRelease-1>", self.on_canvas_release)

        mask_box = ttk.LabelFrame(body, text="Threshold Mask (Host Preview)")
        mask_box.grid(row=0, column=1, sticky="nsew", padx=6)
        mask_box.rowconfigure(0, weight=1)
        mask_box.columnconfigure(0, weight=1)
        self.mask_label = ttk.Label(mask_box)
        self.mask_label.grid(row=0, column=0, sticky="nsew")

        overlay_box = ttk.LabelFrame(body, text="Host Detection Overlay")
        overlay_box.grid(row=0, column=2, sticky="nsew", padx=(6, 0))
        overlay_box.rowconfigure(0, weight=1)
        overlay_box.columnconfigure(0, weight=1)
        self.overlay_label = ttk.Label(overlay_box)
        self.overlay_label.grid(row=0, column=0, sticky="nsew")

        info_box = ttk.LabelFrame(body, text="Detection Status")
        info_box.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        self.single_info_text = tk.Text(info_box, height=10, wrap=tk.WORD)
        self.single_info_text.grid(row=0, column=0, sticky="ew")
        self.single_info_text.configure(state=tk.DISABLED)

    def _build_validation_tab(self) -> None:
        self.validation_tab.columnconfigure(0, weight=1)
        self.validation_tab.rowconfigure(2, weight=1)

        button_bar = ttk.Frame(self.validation_tab)
        button_bar.grid(row=0, column=0, sticky="ew", padx=8, pady=8)
        button_bar.columnconfigure(3, weight=1)
        ttk.Button(button_bar, text="Run Frozen Validation", command=lambda: self.run_validation(False)).grid(row=0, column=0, padx=(0, 6))
        ttk.Button(button_bar, text="Run Sweep Validation", command=lambda: self.run_validation(True)).grid(row=0, column=1, padx=(0, 6))
        ttk.Button(button_bar, text="Show Failed Frames", command=self.focus_first_failed_frame).grid(row=0, column=2, padx=(0, 6))
        ttk.Label(button_bar, textvariable=self.validation_progress_var).grid(row=0, column=3, sticky="w")

        preview_bar = ttk.Frame(self.validation_tab)
        preview_bar.grid(row=1, column=0, sticky="ew", padx=8, pady=(0, 8))
        preview_bar.columnconfigure(1, weight=1)
        ttk.Label(preview_bar, text="Preview Frame").grid(row=0, column=0, sticky="w")
        self.validation_slider = tk.Scale(
            preview_bar,
            from_=0,
            to=0,
            resolution=1,
            orient=tk.HORIZONTAL,
            variable=self.validation_frame_var,
            command=lambda _value: self.update_validation_preview(),
        )
        self.validation_slider.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Label(preview_bar, textvariable=self.validation_frame_var, width=8).grid(row=0, column=2, sticky="e")

        content = ttk.Frame(self.validation_tab)
        content.grid(row=2, column=0, sticky="nsew", padx=8, pady=(0, 8))
        content.columnconfigure(0, weight=0)
        content.columnconfigure(1, weight=1)
        content.rowconfigure(0, weight=1)

        summary_box = ttk.LabelFrame(content, text="Summary")
        summary_box.grid(row=0, column=0, sticky="nsw", padx=(0, 8))
        summary_box.rowconfigure(0, weight=1)
        summary_box.columnconfigure(0, weight=1)
        self.validation_text = tk.Text(summary_box, width=58, wrap=tk.WORD)
        self.validation_text.grid(row=0, column=0, sticky="nsew")
        self.validation_text.configure(state=tk.DISABLED)

        preview_box = ttk.LabelFrame(content, text="Annotated Preview")
        preview_box.grid(row=0, column=1, sticky="nsew")
        preview_box.rowconfigure(0, weight=1)
        preview_box.columnconfigure(0, weight=1)
        self.validation_preview_label = ttk.Label(preview_box)
        self.validation_preview_label.grid(row=0, column=0, sticky="nsew")

    def on_close(self) -> None:
        if self._validation_in_progress:
            messagebox.showwarning("Validation running", "Wait for validation to finish before closing the tuner.")
            return
        try:
            self.disconnect_openmv()
        finally:
            if self.video is not None:
                self.video.close()
            self.root.destroy()

    def connect_openmv(self) -> None:
        try:
            self.status_var.set("Connecting to OpenMV...")
            self.root.update_idletasks()
            self.openmv_evaluator.connect()
        except Exception as exc:
            self.openmv_status_var.set("OpenMV: disconnected")
            self.status_var.set("OpenMV connection failed.")
            messagebox.showerror("OpenMV connection failed", str(exc))
            return
        self.openmv_status_var.set(f"OpenMV: connected ({self.openmv_evaluator.workspace})")
        self.status_var.set("OpenMV connected. Single-frame verdicts and validation now run on device.")
        self.schedule_single_refresh()

    def disconnect_openmv(self) -> None:
        if self._validation_in_progress:
            messagebox.showwarning("Validation running", "Cannot disconnect OpenMV while validation is running.")
            return
        self.openmv_evaluator.disconnect()
        self.openmv_status_var.set("OpenMV: disconnected")
        self.status_var.set("OpenMV disconnected. Single-frame tab now shows host preview only.")
        self.schedule_single_refresh()

    def choose_video(self) -> None:
        path = filedialog.askopenfilename(
            title="Open MJPEG / Video",
            initialdir=os.path.join(REPO_ROOT, "record"),
            filetypes=[("Video files", "*.mjpeg *.avi *.mp4 *.mov *.mjpg"), ("All files", "*.*")],
        )
        if path:
            self.load_video(path)

    def clear_roi(self) -> None:
        self.expected_roi = None
        self.drag_start = None
        self.drag_current = None
        self.schedule_single_refresh()

    def clear_sample(self) -> None:
        self.color_sample_center = None
        self.color_sample = None
        self.schedule_single_refresh()

    def load_video(self, path: str) -> None:
        if self._validation_in_progress:
            messagebox.showwarning("Validation running", "Cannot load another video while validation is running.")
            return
        previous_video = self.video
        try:
            self.status_var.set(f"Loading video: {path}")
            self.root.update_idletasks()
            self.video = load_video_source(path)
        except Exception as exc:
            self.video = previous_video
            messagebox.showerror("Load video failed", str(exc))
            self.status_var.set("Failed to load video.")
            return

        if previous_video is not None and previous_video is not self.video:
            previous_video.close()

        self.frame_index_var.set(0)
        self.validation_frame_var.set(0)
        self.frame_slider.configure(to=max(self.video.frame_count - 1, 0))
        self.validation_slider.configure(to=max(self.video.frame_count - 1, 0))
        self.expected_roi = None
        self.drag_start = None
        self.drag_current = None
        self.host_frame_result = None
        self.single_frame_result = None
        self.color_sample_center = None
        self.color_sample = None
        self.validation_summary = None
        self.video_info_var.set(f"{os.path.basename(path)} | {self.video.frame_count} frames | {self.video.width}x{self.video.height}")
        self.status_var.set("Video loaded. Drag on the transformed image to mark the expected target region.")
        self.schedule_single_refresh()
        self.update_validation_preview()
        self._set_text(self.validation_text, "Run frozen validation or sweep validation to see whole-video results.\n")
        if self.openmv_evaluator.connected:
            try:
                self.openmv_evaluator.clear_workspace()
            except Exception:
                pass

    def schedule_single_refresh(self) -> None:
        if self._validation_in_progress:
            return
        if self._refresh_single_job is not None:
            self.root.after_cancel(self._refresh_single_job)
        self._refresh_single_job = self.root.after(200, self.refresh_single_frame)

    def snapshot_simulation_params(self) -> SimulationParams:
        return SimulationParams(
            exposure_scale=float(self.exposure_scale_var.get()),
            r_gain=float(self.r_gain_var.get()),
            g_gain=float(self.g_gain_var.get()),
            b_gain=float(self.b_gain_var.get()),
            gamma=float(self.gamma_var.get()),
        )

    def snapshot_detector_params(self) -> DetectorParams:
        l_min = int(self.threshold_l_min_var.get())
        l_max = int(self.threshold_l_max_var.get())
        a_min = int(self.threshold_a_min_var.get())
        a_max = int(self.threshold_a_max_var.get())
        b_min = int(self.threshold_b_min_var.get())
        b_max = int(self.threshold_b_max_var.get())
        return DetectorParams(
            threshold_l_min=min(l_min, l_max),
            threshold_l_max=max(l_min, l_max),
            threshold_a_min=min(a_min, a_max),
            threshold_a_max=max(a_min, a_max),
            threshold_b_min=min(b_min, b_max),
            threshold_b_max=max(b_min, b_max),
            min_area=max(1, int(self.min_area_var.get())),
            max_area=max(0, int(self.max_area_var.get())),
            roundness_min_x1000=int(clamp(int(self.roundness_min_var.get()), 0, 1000)),
            merge_margin=max(0, int(self.merge_margin_var.get())),
            track_window_radius_px=max(1, int(self.track_window_radius_var.get())),
            center_filter_gain_x100=int(clamp(int(self.center_filter_gain_var.get()), 0, 100)),
            max_missed_frames=max(1, int(self.max_missed_frames_var.get())),
            ring_detection_enabled=1 if int(self.ring_detection_enabled_var.get()) != 0 else 0,
            ring_min_roundness_x1000=int(clamp(int(self.ring_min_roundness_var.get()), 0, 1000)),
            ring_min_aspect_x100=int(clamp(int(self.ring_min_aspect_var.get()), 1, 100)),
            ring_min_fill_x100=int(clamp(int(self.ring_min_fill_var.get()), 0, 100)),
            ring_max_fill_x100=int(clamp(int(self.ring_max_fill_var.get()), 0, 100)),
            ring_min_center_white_x100=int(clamp(int(self.ring_min_center_white_var.get()), 0, 100)),
            ring_center_sample_ratio_x100=int(clamp(int(self.ring_center_sample_ratio_var.get()), 5, 100)),
            ring_center_min_brightness=int(clamp(int(self.ring_center_min_brightness_var.get()), 0, 255)),
            ring_center_max_channel_delta=int(clamp(int(self.ring_center_max_channel_delta_var.get()), 0, 255)),
            ring_min_outer_diameter_px=max(1, int(self.ring_min_outer_diameter_var.get())),
        )

    def snapshot_sweep_params(self) -> SweepParams:
        return SweepParams(
            exposure_min=float(self.sweep_exposure_min_var.get()),
            exposure_max=float(self.sweep_exposure_max_var.get()),
            exposure_step=float(self.sweep_exposure_step_var.get()),
            gain_min=float(self.sweep_gain_min_var.get()),
            gain_max=float(self.sweep_gain_max_var.get()),
            gain_step=float(self.sweep_gain_step_var.get()),
        )

    def snapshot_validation_run_config(self, use_sweep: bool) -> ValidationRunConfig:
        return ValidationRunConfig(
            sim_params=self.snapshot_simulation_params(),
            detector_params=self.snapshot_detector_params(),
            sweep_params=self.snapshot_sweep_params(),
            expected_roi=self.expected_roi,
            use_sweep=use_sweep,
        )

    def apply_run_config_to_controls(self, run_config: ValidationRunConfig, refresh: bool = True) -> None:
        self.exposure_scale_var.set(run_config.sim_params.exposure_scale)
        self.r_gain_var.set(run_config.sim_params.r_gain)
        self.g_gain_var.set(run_config.sim_params.g_gain)
        self.b_gain_var.set(run_config.sim_params.b_gain)
        self.gamma_var.set(run_config.sim_params.gamma)

        self.threshold_l_min_var.set(run_config.detector_params.threshold_l_min)
        self.threshold_l_max_var.set(run_config.detector_params.threshold_l_max)
        self.threshold_a_min_var.set(run_config.detector_params.threshold_a_min)
        self.threshold_a_max_var.set(run_config.detector_params.threshold_a_max)
        self.threshold_b_min_var.set(run_config.detector_params.threshold_b_min)
        self.threshold_b_max_var.set(run_config.detector_params.threshold_b_max)
        self.min_area_var.set(run_config.detector_params.min_area)
        self.max_area_var.set(run_config.detector_params.max_area)
        self.roundness_min_var.set(run_config.detector_params.roundness_min_x1000)
        self.merge_margin_var.set(run_config.detector_params.merge_margin)
        self.track_window_radius_var.set(run_config.detector_params.track_window_radius_px)
        self.center_filter_gain_var.set(run_config.detector_params.center_filter_gain_x100)
        self.max_missed_frames_var.set(run_config.detector_params.max_missed_frames)
        self.ring_detection_enabled_var.set(run_config.detector_params.ring_detection_enabled)
        self.ring_min_roundness_var.set(run_config.detector_params.ring_min_roundness_x1000)
        self.ring_min_aspect_var.set(run_config.detector_params.ring_min_aspect_x100)
        self.ring_min_fill_var.set(run_config.detector_params.ring_min_fill_x100)
        self.ring_max_fill_var.set(run_config.detector_params.ring_max_fill_x100)
        self.ring_min_center_white_var.set(run_config.detector_params.ring_min_center_white_x100)
        self.ring_center_sample_ratio_var.set(run_config.detector_params.ring_center_sample_ratio_x100)
        self.ring_center_min_brightness_var.set(run_config.detector_params.ring_center_min_brightness)
        self.ring_center_max_channel_delta_var.set(run_config.detector_params.ring_center_max_channel_delta)
        self.ring_min_outer_diameter_var.set(run_config.detector_params.ring_min_outer_diameter_px)

        self.sweep_exposure_min_var.set(run_config.sweep_params.exposure_min)
        self.sweep_exposure_max_var.set(run_config.sweep_params.exposure_max)
        self.sweep_exposure_step_var.set(run_config.sweep_params.exposure_step)
        self.sweep_gain_min_var.set(run_config.sweep_params.gain_min)
        self.sweep_gain_max_var.set(run_config.sweep_params.gain_max)
        self.sweep_gain_step_var.set(run_config.sweep_params.gain_step)

        self.expected_roi = run_config.expected_roi
        self.drag_start = None
        self.drag_current = None
        if refresh:
            self.schedule_single_refresh()

    def current_frame(self) -> np.ndarray | None:
        if self.video is None or self.video.frame_count == 0:
            return None
        index = int(clamp(self.frame_index_var.get(), 0, self.video.frame_count - 1))
        self.frame_index_var.set(index)
        return self.video.get_frame(index)

    def ensure_openmv_connection(self, auto_connect: bool = False) -> bool:
        if self.openmv_evaluator.connected:
            return True
        if not auto_connect:
            return False
        self.connect_openmv()
        return self.openmv_evaluator.connected

    def _fallback_result(self) -> DetectionDebug:
        return DetectionDebug(
            candidates=[],
            best_candidate=None,
            mask=np.zeros((1, 1), dtype=np.uint8),
            search_roi=None,
            fallback_used=False,
            reason="OpenMV 未连接，当前仅显示主机预览",
            detected=False,
            locked=False,
            background_misdetect=False,
            raw_center=None,
            filtered_center=None,
            area=0,
            radius_px=0,
        )

    def refresh_single_frame(self) -> None:
        self._refresh_single_job = None
        frame = self.current_frame()
        if frame is None:
            self.adjusted_canvas.delete("all")
            self.mask_label.configure(image="")
            self.overlay_label.configure(image="")
            self._set_text(self.single_info_text, "Open a video to start tuning.\n")
            return

        sim_params = self.snapshot_simulation_params()
        detector_params = self.snapshot_detector_params()
        adjusted = apply_simulation(frame, sim_params)
        preview_mask = threshold_lab_mask(adjusted, detector_params)
        self.host_frame_result = evaluate_host_frame(adjusted, detector_params, self.expected_roi, preview_mask)
        self.color_sample = sample_color_circle(adjusted, self.color_sample_center, int(self.sample_radius_var.get()))

        if self._validation_in_progress:
            self.single_frame_result = self._fallback_result()
            self.single_frame_result.reason = "Validation running on OpenMV; single-frame verdict paused."
        elif self.ensure_openmv_connection(auto_connect=False):
            try:
                cache_key = (
                    int(self.frame_index_var.get()),
                    round(sim_params.exposure_scale, 4),
                    round(sim_params.r_gain, 4),
                    round(sim_params.g_gain, 4),
                    round(sim_params.b_gain, 4),
                    round(sim_params.gamma, 4),
                )
                remote_path = self.openmv_evaluator.upload_single_frame(adjusted, cache_key)
                self.single_frame_result = self.openmv_evaluator.evaluate_single_frame(remote_path, detector_params, self.expected_roi)
            except Exception as exc:
                self.single_frame_result = self._fallback_result()
                self.single_frame_result.reason = f"OpenMV 单帧判定失败: {exc}"
                self.status_var.set("Single-frame evaluation failed.")
        else:
            self.single_frame_result = self._fallback_result()

        self.single_frame_result.mask = preview_mask
        self.host_frame_result.mask = preview_mask
        self.single_display_scale = scale_for_display(adjusted.shape[1], adjusted.shape[0])
        adjusted_display = resize_for_display(self.draw_adjusted_view(adjusted), self.single_display_scale)
        mask_display = resize_for_display(self.draw_mask_view(preview_mask), self.single_display_scale)
        overlay_display = resize_for_display(self.draw_overlay_view(adjusted, self.host_frame_result, "HOST PREVIEW"), self.single_display_scale)

        self._adjusted_photo = bgr_to_photo(adjusted_display)
        self._mask_photo = bgr_to_photo(mask_display)
        self._overlay_photo = bgr_to_photo(overlay_display)

        self.adjusted_canvas.configure(width=adjusted_display.shape[1], height=adjusted_display.shape[0])
        self.adjusted_canvas.delete("all")
        self.adjusted_canvas.create_image(0, 0, image=self._adjusted_photo, anchor="nw")

        roi_to_draw = self.expected_roi
        if self.drag_start is not None and self.drag_current is not None and self.video is not None and self.interaction_mode_var.get() == "roi":
            roi_to_draw = roi_from_drag(self.drag_start, self.drag_current, self.video.width, self.video.height)
        if roi_to_draw is not None:
            x, y, w, h = roi_to_draw
            scale = self.single_display_scale
            self.adjusted_canvas.create_rectangle(
                x * scale,
                y * scale,
                (x + w) * scale,
                (y + h) * scale,
                outline="#00ffff",
                width=2,
            )

        self.draw_sample_canvas_overlay()
        self.mask_label.configure(image=self._mask_photo)
        self.overlay_label.configure(image=self._overlay_photo)
        self._set_text(self.single_info_text, self.format_single_frame_info(self.host_frame_result, self.single_frame_result))

    def draw_adjusted_view(self, adjusted_bgr: np.ndarray) -> np.ndarray:
        image = adjusted_bgr.copy()
        self.draw_sample_marker(image)
        cv2.putText(image, f"frame={self.frame_index_var.get()}", (10, 24), CV_FONT, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        return image

    def draw_sample_marker(self, image: np.ndarray) -> None:
        if self.color_sample_center is None:
            return
        center_x, center_y = self.color_sample_center
        radius = max(0, int(self.sample_radius_var.get()))
        cv2.circle(image, (center_x, center_y), radius, (255, 0, 255), 2)
        cv2.circle(image, (center_x, center_y), 2, (255, 0, 255), -1)

    def draw_sample_canvas_overlay(self) -> None:
        if self.color_sample_center is None:
            return
        center_x, center_y = self.color_sample_center
        radius = max(0, int(self.sample_radius_var.get()))
        scale = self.single_display_scale
        self.adjusted_canvas.create_oval(
            (center_x - radius) * scale,
            (center_y - radius) * scale,
            (center_x + radius) * scale,
            (center_y + radius) * scale,
            outline="#ff00ff",
            width=2,
        )
        self.adjusted_canvas.create_oval(
            (center_x - 2) * scale,
            (center_y - 2) * scale,
            (center_x + 2) * scale,
            (center_y + 2) * scale,
            fill="#ff00ff",
            outline="#ff00ff",
        )

    def draw_mask_view(self, mask: np.ndarray) -> np.ndarray:
        image = gray_to_bgr(mask)
        if self.expected_roi is not None:
            x, y, w, h = self.expected_roi
            cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 0), 2)
        self.draw_sample_marker(image)
        cv2.putText(image, "HOST MASK", (10, 24), CV_FONT, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        return image

    def draw_overlay_view(self, adjusted_bgr: np.ndarray, result: DetectionDebug, label: str) -> np.ndarray:
        image = adjusted_bgr.copy()
        self.draw_sample_marker(image)
        if self.expected_roi is not None:
            x, y, w, h = self.expected_roi
            cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 0), 2)

        for candidate in result.candidates:
            color = (0, 0, 255)
            if candidate.passes_area and candidate.passes_roundness:
                color = (0, 200, 255)
            if candidate.passes_ring:
                color = (255, 160, 0)
            if result.best_candidate is candidate:
                color = (0, 255, 0) if result.locked else (0, 128, 255)
            x, y, w, h = candidate.bbox
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            cv2.circle(image, (candidate.center_x, candidate.center_y), 3, color, -1)

        if result.search_roi is not None:
            x, y, w, h = result.search_roi
            cv2.rectangle(image, (x, y), (x + w, y + h), (180, 180, 180), 1)

        if result.raw_center is not None:
            center_x, center_y = result.raw_center
            cv2.circle(image, (center_x, center_y), max(result.radius_px, 4), (0, 255, 0) if result.locked else (0, 128, 255), 2)
        cv2.putText(
            image,
            "LOCK" if result.locked else "MISS",
            (10, 24),
            CV_FONT,
            0.75,
            (0, 255, 0) if result.locked else (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(image, result.reason, (10, 48), CV_FONT, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(image, label, (10, 72), CV_FONT, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        return image

    @staticmethod
    def _format_float_triplet(values: tuple[float, float, float]) -> str:
        return ", ".join(f"{value:.1f}" for value in values)

    @staticmethod
    def _format_int_triplet(values) -> str:
        return ", ".join(str(int(value)) for value in values)

    def format_detection_summary(self, label: str, result: DetectionDebug) -> list[str]:
        locked = "YES" if result.locked else "NO"
        detected = "YES" if result.detected else "NO"
        background = "YES" if result.background_misdetect else "NO"
        source = result.source if result.source else "-"
        lines = [
            f"{label}:",
            f"  locked={locked} lock_signal={1 if result.locked else 0} detected={detected}",
            f"  reason={result.reason} background={background} source={source}",
        ]
        if result.raw_center is not None:
            lines.append(f"  center=({result.raw_center[0]}, {result.raw_center[1]}) area={result.area} radius={result.radius_px}")
        else:
            lines.append("  center=- area=- radius=-")
        lines.append(f"  candidates={len(result.candidates)}")
        return lines

    def format_sample_info(self) -> list[str]:
        lines = ["Color sample:"]
        sample = self.color_sample
        if sample is None:
            lines.append("  center=- radius=%d pixels=0" % int(self.sample_radius_var.get()))
            return lines

        l_min = int(clamp(math.floor(sample.lab_min[0]), 0, 100))
        l_max = int(clamp(math.ceil(sample.lab_max[0]), 0, 100))
        a_min = int(clamp(sample.lab_min[1], -128, 127))
        a_max = int(clamp(sample.lab_max[1], -128, 127))
        b_min = int(clamp(sample.lab_min[2], -128, 127))
        b_max = int(clamp(sample.lab_max[2], -128, 127))
        lines.extend(
            [
                f"  center=({sample.center_x}, {sample.center_y}) radius={sample.radius_px} pixels={sample.pixel_count}",
                f"  RGB mean=({self._format_float_triplet(sample.rgb_mean)}) min=({self._format_int_triplet(sample.rgb_min)}) max=({self._format_int_triplet(sample.rgb_max)})",
                f"  LAB mean=({self._format_float_triplet(sample.lab_mean)}) min=({l_min}, {a_min}, {b_min}) max=({l_max}, {a_max}, {b_max})",
                f"  sample_lab_threshold: L=({l_min},{l_max}) A=({a_min},{a_max}) B=({b_min},{b_max})",
            ]
        )
        return lines

    def format_single_frame_info(self, host_result: DetectionDebug, openmv_result: DetectionDebug) -> str:
        sim_params = self.snapshot_simulation_params()
        detector_params = self.snapshot_detector_params()
        lines = [f"Frame: {self.frame_index_var.get()}"]
        lines.extend(self.format_detection_summary("Host preview", host_result))
        lines.append("")
        lines.extend(self.format_detection_summary("OpenMV verdict", openmv_result))
        lines.append(LOCAL_PREVIEW_NOTE)
        lines.append("")
        lines.extend(self.format_sample_info())
        lines.extend(
            [
                "",
                "Current simulation params:",
                f"  exposure_scale={sim_params.exposure_scale:.4f}",
                f"  r_gain={sim_params.r_gain:.4f}",
                f"  g_gain={sim_params.g_gain:.4f}",
                f"  b_gain={sim_params.b_gain:.4f}",
                f"  gamma={sim_params.gamma:.4f}",
                "Current detector params:",
                f"  threshold_l_min={detector_params.threshold_l_min}",
                f"  threshold_l_max={detector_params.threshold_l_max}",
                f"  threshold_a_min={detector_params.threshold_a_min}",
                f"  threshold_a_max={detector_params.threshold_a_max}",
                f"  threshold_b_min={detector_params.threshold_b_min}",
                f"  threshold_b_max={detector_params.threshold_b_max}",
                f"  min_area={detector_params.min_area}",
                f"  max_area={detector_params.max_area}",
                f"  roundness_min_x1000={detector_params.roundness_min_x1000}",
                f"  merge_margin={detector_params.merge_margin}",
                f"  track_window_radius_px={detector_params.track_window_radius_px}",
                f"  center_filter_gain_x100={detector_params.center_filter_gain_x100}",
                f"  max_missed_frames={detector_params.max_missed_frames}",
                f"  ring_detection_enabled={detector_params.ring_detection_enabled}",
                f"  ring_min_roundness_x1000={detector_params.ring_min_roundness_x1000}",
                f"  ring_min_aspect_x100={detector_params.ring_min_aspect_x100}",
                f"  ring_min_fill_x100={detector_params.ring_min_fill_x100}",
                f"  ring_max_fill_x100={detector_params.ring_max_fill_x100}",
                f"  ring_min_center_white_x100={detector_params.ring_min_center_white_x100}",
                f"  ring_center_sample_ratio_x100={detector_params.ring_center_sample_ratio_x100}",
                f"  ring_center_min_brightness={detector_params.ring_center_min_brightness}",
                f"  ring_center_max_channel_delta={detector_params.ring_center_max_channel_delta}",
                f"  ring_min_outer_diameter_px={detector_params.ring_min_outer_diameter_px}",
            ]
        )
        if self.expected_roi is not None:
            rx, ry, rw, rh = self.expected_roi
            lines.append(f"Expected ROI: x={rx}, y={ry}, w={rw}, h={rh}")
        if host_result.candidates:
            lines.append("")
            lines.append("Host preview candidates:")
            ranked = sorted(host_result.candidates, key=lambda candidate: candidate.roundness_x1000, reverse=True)[:6]
            for index, candidate in enumerate(ranked, start=1):
                inside = point_in_roi((candidate.center_x, candidate.center_y), self.expected_roi)
                source = candidate.source if candidate.source else "-"
                ring = "yes" if candidate.passes_ring else "no"
                inside_text = "yes" if inside else "no"
                lines.append(
                    f"{index}. center=({candidate.center_x},{candidate.center_y}) area={candidate.area} "
                    f"roundness={candidate.roundness_x1000} source={source} "
                    f"ring={ring} fill={candidate.green_fill_x100} "
                    f"center_white={candidate.center_white_x100} inside_roi={inside_text}"
                )
        return "\n".join(lines) + "\n"

    def on_canvas_press(self, event: tk.Event) -> None:
        if self.video is None:
            return
        point = self.canvas_to_image_coords(event.x, event.y)
        if self.interaction_mode_var.get() == "sample":
            self.color_sample_center = point
            self.drag_start = None
            self.drag_current = None
        else:
            self.drag_start = point
            self.drag_current = point
        self.schedule_single_refresh()

    def on_canvas_drag(self, event: tk.Event) -> None:
        if self.video is None:
            return
        point = self.canvas_to_image_coords(event.x, event.y)
        if self.interaction_mode_var.get() == "sample":
            self.color_sample_center = point
        elif self.drag_start is not None:
            self.drag_current = point
        self.schedule_single_refresh()

    def on_canvas_release(self, event: tk.Event) -> None:
        if self.video is None:
            return
        point = self.canvas_to_image_coords(event.x, event.y)
        if self.interaction_mode_var.get() == "sample":
            self.color_sample_center = point
            self.drag_start = None
            self.drag_current = None
        elif self.drag_start is not None:
            self.drag_current = point
            self.expected_roi = roi_from_drag(self.drag_start, self.drag_current, self.video.width, self.video.height)
            self.drag_start = None
            self.drag_current = None
        self.schedule_single_refresh()

    def canvas_to_image_coords(self, canvas_x: int, canvas_y: int) -> tuple[int, int]:
        if self.video is None:
            return (0, 0)
        scale = max(self.single_display_scale, 1e-6)
        image_x = int(clamp(round(canvas_x / scale), 0, max(self.video.width - 1, 0)))
        image_y = int(clamp(round(canvas_y / scale), 0, max(self.video.height - 1, 0)))
        return (image_x, image_y)

    def run_validation(self, use_sweep: bool) -> None:
        if self.video is None:
            messagebox.showwarning("No video", "Open a video before running validation.")
            return
        if self._validation_in_progress:
            messagebox.showinfo("Validation running", "A validation run is already in progress.")
            return
        if not self.ensure_openmv_connection(auto_connect=True):
            return

        run_config = self.snapshot_validation_run_config(use_sweep)
        self._validation_in_progress = True
        self.validation_progress_var.set("Running OpenMV validation...")
        self.status_var.set("Validation started in background.")
        self._set_text(self.validation_text, "Validation is running in the background...\n")
        self.schedule_single_refresh()

        def worker() -> None:
            try:
                frame_results = self.openmv_evaluator.stream_validation(
                    self.video,
                    run_config,
                    progress_callback=self._validation_progress,
                )
            except Exception as exc:
                self.root.after(0, lambda: self._finish_validation_error(str(exc)))
                return
            self.root.after(0, lambda: self._finish_validation_success(frame_results, run_config))

        self._validation_thread = threading.Thread(target=worker, name="openmv-validation", daemon=True)
        self._validation_thread.start()

    def _validation_progress(self, completed: int, total: int) -> None:
        self.root.after(0, lambda: self.validation_progress_var.set(f"Running OpenMV validation... {completed}/{total}"))

    def _finish_validation_success(self, frame_results: list[ValidationFrameResult], run_config: ValidationRunConfig) -> None:
        summary = self.build_validation_summary(frame_results, run_config)
        self.validation_summary = summary
        self.validation_slider.configure(to=max(summary.total_frames - 1, 0))
        self.validation_frame_var.set(0)
        self.update_validation_preview()
        self._set_text(self.validation_text, self.format_validation_summary(summary))
        mode_text = "Sweep validation completed." if run_config.use_sweep else "Frozen validation completed."
        self.validation_progress_var.set(mode_text)
        self.status_var.set(mode_text)
        self._validation_in_progress = False
        self._validation_thread = None
        self.schedule_single_refresh()

    def _finish_validation_error(self, error_text: str) -> None:
        self.validation_progress_var.set("Validation failed.")
        self.status_var.set("Validation failed.")
        self._validation_in_progress = False
        self._validation_thread = None
        self.schedule_single_refresh()
        messagebox.showerror("Validation failed", error_text)

    def build_validation_summary(self, frame_results: list[ValidationFrameResult], run_config: ValidationRunConfig) -> ValidationSummary:
        hit_frames = 0
        longest_miss_streak = 0
        current_miss_streak = 0
        first_miss_frame: int | None = None
        centers: list[tuple[int, int]] = []
        misdetected_frames: list[int] = []
        recognition_log: list[str] = []

        detector_params = run_config.detector_params
        for result in frame_results:
            if result.locked:
                hit_frames += 1
                current_miss_streak = 0
                if result.filtered_center is not None:
                    centers.append(result.filtered_center)
            else:
                current_miss_streak += 1
                if first_miss_frame is None:
                    first_miss_frame = result.frame_index
                longest_miss_streak = max(longest_miss_streak, current_miss_streak)

            recognition_log.append(
                f"frame={result.frame_index} lock={1 if result.locked else 0} detected={1 if result.detected else 0} "
                f"background={1 if result.background_misdetect else 0} exp={result.exposure_scale:.4f} gain={result.gain_scale:.4f} "
                f"L=({detector_params.threshold_l_min},{detector_params.threshold_l_max}) "
                f"A=({detector_params.threshold_a_min},{detector_params.threshold_a_max}) "
                f"B=({detector_params.threshold_b_min},{detector_params.threshold_b_max}) "
                f"min_area={detector_params.min_area} max_area={detector_params.max_area} "
                f"roundness={detector_params.roundness_min_x1000} merge={detector_params.merge_margin} "
                f"track_radius={detector_params.track_window_radius_px} filter={detector_params.center_filter_gain_x100} "
                f"max_missed={detector_params.max_missed_frames} center={result.filtered_center} area={result.area} radius={result.radius_px} reason={result.reason}"
            )

            if result.background_misdetect:
                misdetected_frames.append(result.frame_index)

        step_lengths = []
        for previous, current in zip(centers, centers[1:]):
            step_lengths.append(math.hypot(current[0] - previous[0], current[1] - previous[1]))

        jitter_mean = float(statistics.mean(step_lengths)) if step_lengths else 0.0
        jitter_std = float(statistics.pstdev(step_lengths)) if len(step_lengths) > 1 else 0.0
        jitter_max = float(max(step_lengths)) if step_lengths else 0.0
        total_frames = len(frame_results)
        return ValidationSummary(
            total_frames=total_frames,
            hit_frames=hit_frames,
            hit_rate=(0.0 if total_frames == 0 else hit_frames / float(total_frames)),
            longest_miss_streak=max(longest_miss_streak, current_miss_streak),
            first_miss_frame=first_miss_frame,
            jitter_mean=jitter_mean,
            jitter_std=jitter_std,
            jitter_max=jitter_max,
            misdetected_frames=misdetected_frames,
            recognition_log=recognition_log,
            frames=frame_results,
            run_config=run_config,
        )

    def format_validation_summary(self, summary: ValidationSummary) -> str:
        run_config = summary.run_config
        lines = [
            f"Mode: {'Sweep Validation' if (run_config.use_sweep if run_config is not None else False) else 'Frozen Validation'}",
            f"Total frames: {summary.total_frames}",
            f"Hit frames: {summary.hit_frames}",
            f"Hit rate: {summary.hit_rate * 100.0:.2f}%",
            f"Longest miss streak: {summary.longest_miss_streak}",
            f"First miss frame: {summary.first_miss_frame if summary.first_miss_frame is not None else '-'}",
            f"Center jitter mean: {summary.jitter_mean:.2f}px",
            f"Center jitter std: {summary.jitter_std:.2f}px",
            f"Center jitter max: {summary.jitter_max:.2f}px",
            "Validation verdicts are produced on OpenMV.",
        ]
        if run_config is not None and run_config.expected_roi is not None:
            lines.append(f"Background misdetect frames: {len(summary.misdetected_frames)}")
            if summary.misdetected_frames:
                preview = ", ".join(str(index) for index in summary.misdetected_frames[:50])
                if len(summary.misdetected_frames) > 50:
                    preview += ", ..."
                lines.append(f"Misdetected frame indices: {preview}")
        else:
            lines.append("Background misdetect frames: N/A (set an ROI to enable this check)")

        lines.append("")
        lines.append("Recognition log:")
        if summary.recognition_log:
            lines.extend(summary.recognition_log[:200])
            if len(summary.recognition_log) > 200:
                lines.append("... truncated ...")
        else:
            lines.append("No successful lock frames.")
        return "\n".join(lines) + "\n"

    def update_validation_preview(self) -> None:
        if self.video is None or self.validation_summary is None or not self.validation_summary.frames:
            self.validation_preview_label.configure(image="")
            return

        index = int(clamp(self.validation_frame_var.get(), 0, len(self.validation_summary.frames) - 1))
        self.validation_frame_var.set(index)
        frame_result = self.validation_summary.frames[index]
        run_config = self.validation_summary.run_config
        if run_config is None:
            return
        frame = self.video.get_frame(frame_result.frame_index)
        preview = apply_simulation(
            frame,
            run_config.sim_params,
            dynamic_exposure_scale=frame_result.exposure_scale,
            dynamic_gain_scale=frame_result.gain_scale,
        )
        preview = self.draw_validation_preview(preview, frame_result, run_config.expected_roi)
        display_scale = scale_for_display(preview.shape[1], preview.shape[0])
        preview_display = resize_for_display(preview, display_scale)
        self._validation_photo = bgr_to_photo(preview_display)
        self.validation_preview_label.configure(image=self._validation_photo)

    def draw_validation_preview(
        self,
        frame_bgr: np.ndarray,
        frame_result: ValidationFrameResult,
        expected_roi: tuple[int, int, int, int] | None,
    ) -> np.ndarray:
        image = frame_bgr.copy()
        if expected_roi is not None:
            x, y, w, h = expected_roi
            cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 0), 2)
        if frame_result.raw_center is not None:
            radius = max(frame_result.radius_px, 4)
            color = (0, 255, 0) if frame_result.locked else (0, 128, 255)
            cv2.circle(image, frame_result.raw_center, radius, color, 2)
            cv2.circle(image, frame_result.raw_center, 3, color, -1)

        status = "LOCK" if frame_result.locked else "MISS"
        cv2.putText(image, f"frame={frame_result.frame_index} {status}", (10, 24), CV_FONT, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(image, f"exp={frame_result.exposure_scale:.2f} gain={frame_result.gain_scale:.2f}", (10, 48), CV_FONT, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(image, frame_result.reason, (10, 72), CV_FONT, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        if frame_result.filtered_center is not None:
            cv2.putText(image, f"center={frame_result.filtered_center} area={frame_result.area} radius={frame_result.radius_px}", (10, 96), CV_FONT, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(image, "OPENMV VERDICT", (10, 120), CV_FONT, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        return image

    def focus_first_failed_frame(self) -> None:
        if self.validation_summary is None or not self.validation_summary.frames:
            return
        for frame_result in self.validation_summary.frames:
            if not frame_result.locked:
                if self.validation_summary.run_config is not None:
                    self.apply_run_config_to_controls(self.validation_summary.run_config, refresh=False)
                self.validation_frame_var.set(frame_result.frame_index)
                self.frame_index_var.set(frame_result.frame_index)
                self.update_validation_preview()
                self.schedule_single_refresh()
                self.notebook.select(self.single_tab)
                self.status_var.set("Restored validation snapshot and focused the first failed frame.")
                return
        messagebox.showinfo("Validation", "No failed frames in the current validation result.")

    def copy_yaml_snippet(self) -> None:
        params = self.snapshot_detector_params()
        snippet = "\n".join(
            [
                "green_light:",
                f"  openmv_threshold_l_min: {params.threshold_l_min}",
                f"  openmv_threshold_l_max: {params.threshold_l_max}",
                f"  openmv_threshold_a_min: {params.threshold_a_min}",
                f"  openmv_threshold_a_max: {params.threshold_a_max}",
                f"  openmv_threshold_b_min: {params.threshold_b_min}",
                f"  openmv_threshold_b_max: {params.threshold_b_max}",
                f"  openmv_min_area: {params.min_area}",
                f"  openmv_max_area: {params.max_area}",
                f"  openmv_roundness_min_x1000: {params.roundness_min_x1000}",
                f"  openmv_merge_margin: {params.merge_margin}",
                f"  openmv_track_window_radius_px: {params.track_window_radius_px}",
                f"  openmv_center_filter_gain_x100: {params.center_filter_gain_x100}",
                f"  openmv_max_missed_frames: {params.max_missed_frames}",
                f"  openmv_ring_detection_enabled: {params.ring_detection_enabled}",
                f"  openmv_ring_min_roundness_x1000: {params.ring_min_roundness_x1000}",
                f"  openmv_ring_min_aspect_x100: {params.ring_min_aspect_x100}",
                f"  openmv_ring_min_fill_x100: {params.ring_min_fill_x100}",
                f"  openmv_ring_max_fill_x100: {params.ring_max_fill_x100}",
                f"  openmv_ring_min_center_white_x100: {params.ring_min_center_white_x100}",
                f"  openmv_ring_center_sample_ratio_x100: {params.ring_center_sample_ratio_x100}",
                f"  openmv_ring_center_min_brightness: {params.ring_center_min_brightness}",
                f"  openmv_ring_center_max_channel_delta: {params.ring_center_max_channel_delta}",
                f"  openmv_ring_min_outer_diameter_px: {params.ring_min_outer_diameter_px}",
                "",
            ]
        )
        self.root.clipboard_clear()
        self.root.clipboard_append(snippet)
        self.status_var.set("Current detector parameters copied to clipboard as a YAML snippet.")

    @staticmethod
    def _set_text(widget: tk.Text, text: str) -> None:
        widget.configure(state=tk.NORMAL)
        widget.delete("1.0", tk.END)
        widget.insert(tk.END, text)
        widget.configure(state=tk.DISABLED)


def launch_tuner(video_path: str, config_path: str, port: str = "", baudrate: int = 0) -> int:
    root = tk.Tk()
    app = VideoTunerApp(root, video_path, config_path, port=port, baudrate=baudrate)
    del app
    root.mainloop()
    return 0
