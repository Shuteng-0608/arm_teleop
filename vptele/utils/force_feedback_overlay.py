from __future__ import annotations

from typing import Any, Dict, Optional

import cv2
import numpy as np


class ForceFeedbackConfig:
    def __init__(
        self,
        enabled: bool = False,
        display_mode: str = "overlay",
        overlay_cameras=None,
        low_threshold: float = 10.0,
        medium_threshold: float = 40.0,
        high_threshold: float = 80.0,
        excessive_threshold: float = 100.0,
        show_numbers: bool = True,
        show_axial_lateral: bool = True,
        show_trend: bool = True,
        show_contact_state: bool = True,
        show_arrow: bool = False,
        smoothing_alpha: float = 0.25,
        window_name: str = "Force Feedback HUD",
        insertion_axis_world=None,
        use_compensated_wrench: bool = True,
        trend_window_sec: float = 0.8,
        trend_rising_threshold: float = 5.0,
        trend_falling_threshold: float = -5.0,
        contact_free_threshold: float = 5.0,
        contact_light_threshold: float = 15.0,
        axial_high_threshold: float = 20.0,
        lateral_high_threshold: float = 10.0,
        jam_force_threshold: float = 25.0,
        jam_lateral_threshold: float = 12.0,
        enable_task_force_guidance_hud: bool = False,
        task_force_guidance_mode: str = "ring",
        force_guidance_overlay_cameras=None,
        show_translation_ring: bool = True,
        show_torque_ring: bool = True,
        show_axial_core: bool = True,
        force_guidance_plane_right_world=None,
        force_guidance_plane_up_world=None,
        force_guidance_ring_radius_px: int = 70,
        force_guidance_max_vector_px: int = 55,
        force_guidance_max_force_n: float = 40.0,
        force_guidance_max_torque_nm: float = 2.0,
        torque_guidance_mode: str = "tilt_axes",
        torque_guidance_min_force_n: float = 5.0,
        torque_guidance_min_torque_nm: float = 0.05,
        torque_guidance_max_torque_nm=None,
        torque_guidance_label_as_posture: bool = True,
        torque_guidance_show_numeric_values: bool = True,
        force_guidance_draw_numeric_values: bool = True,
        force_guidance_show_caveat_label: bool = True,
        force_guidance_hud_anchor: str = "top_right",
        force_guidance_hud_margin_px=None,
        force_guidance_hud_offset_px=None,
        force_guidance_hud_center_norm=None,
        force_guidance_basis_mode: str = "camera_screen",
        force_guidance_basis_camera: str = "cctv_cam",
        force_guidance_screen_right_sign: float = 1.0,
        force_guidance_screen_up_sign: float = 1.0,
        force_guidance_vector_semantics: str = "contact",
        force_guidance_correction_sign: float = -1.0,
        wrench_label: str = "comp",
    ):
        self.enabled = bool(enabled)
        self.display_mode = str(display_mode)
        self.overlay_cameras = _string_list(overlay_cameras, ["cctv_cam"])
        self.low_threshold = float(low_threshold)
        self.medium_threshold = float(medium_threshold)
        self.high_threshold = float(high_threshold)
        self.excessive_threshold = float(excessive_threshold)
        self.show_numbers = bool(show_numbers)
        self.show_axial_lateral = bool(show_axial_lateral)
        self.show_trend = bool(show_trend)
        self.show_contact_state = bool(show_contact_state)
        self.show_arrow = bool(show_arrow)
        self.smoothing_alpha = float(np.clip(smoothing_alpha, 0.0, 1.0))
        self.window_name = str(window_name)
        self.insertion_axis_world = _normalized(
            insertion_axis_world if insertion_axis_world is not None else [0.0, -1.0, 0.0]
        )
        self.use_compensated_wrench = bool(use_compensated_wrench)
        self.trend_window_sec = max(0.0, float(trend_window_sec))
        self.trend_rising_threshold = float(trend_rising_threshold)
        self.trend_falling_threshold = float(trend_falling_threshold)
        self.contact_free_threshold = max(0.0, float(contact_free_threshold))
        self.contact_light_threshold = max(
            self.contact_free_threshold,
            float(contact_light_threshold),
        )
        self.axial_high_threshold = max(0.0, float(axial_high_threshold))
        self.lateral_high_threshold = max(0.0, float(lateral_high_threshold))
        self.jam_force_threshold = max(0.0, float(jam_force_threshold))
        self.jam_lateral_threshold = max(0.0, float(jam_lateral_threshold))
        self.enable_task_force_guidance_hud = bool(enable_task_force_guidance_hud)
        self.task_force_guidance_mode = str(task_force_guidance_mode)
        if self.task_force_guidance_mode not in {"ring", "bars"}:
            self.task_force_guidance_mode = "ring"
        self.force_guidance_overlay_cameras = _string_list(
            force_guidance_overlay_cameras,
            self.overlay_cameras,
        )
        self.show_translation_ring = bool(show_translation_ring)
        self.show_torque_ring = bool(show_torque_ring)
        self.show_axial_core = bool(show_axial_core)
        self.force_guidance_plane_right_world = _normalized(
            force_guidance_plane_right_world
            if force_guidance_plane_right_world is not None
            else [1.0, 0.0, 0.0]
        )
        self.force_guidance_plane_up_world = _normalized(
            force_guidance_plane_up_world
            if force_guidance_plane_up_world is not None
            else [0.0, 0.0, 1.0]
        )
        self.force_guidance_ring_radius_px = max(20, int(force_guidance_ring_radius_px))
        self.force_guidance_max_vector_px = max(10, int(force_guidance_max_vector_px))
        self.force_guidance_max_force_n = max(1e-9, float(force_guidance_max_force_n))
        torque_max = (
            force_guidance_max_torque_nm
            if torque_guidance_max_torque_nm is None
            else torque_guidance_max_torque_nm
        )
        self.force_guidance_max_torque_nm = max(1e-9, float(torque_max))
        self.torque_guidance_mode = str(torque_guidance_mode)
        if self.torque_guidance_mode not in {"vector", "tilt_axes", "off"}:
            self.torque_guidance_mode = "tilt_axes"
        self.torque_guidance_min_force_n = max(0.0, float(torque_guidance_min_force_n))
        self.torque_guidance_min_torque_nm = max(0.0, float(torque_guidance_min_torque_nm))
        self.torque_guidance_max_torque_nm = self.force_guidance_max_torque_nm
        self.torque_guidance_label_as_posture = bool(torque_guidance_label_as_posture)
        self.torque_guidance_show_numeric_values = bool(torque_guidance_show_numeric_values)
        self.force_guidance_draw_numeric_values = bool(force_guidance_draw_numeric_values)
        self.force_guidance_show_caveat_label = bool(force_guidance_show_caveat_label)
        self.force_guidance_hud_anchor = str(force_guidance_hud_anchor)
        if self.force_guidance_hud_anchor not in {
            "top_left",
            "top_right",
            "bottom_left",
            "bottom_right",
            "center",
            "custom",
        }:
            self.force_guidance_hud_anchor = "top_right"
        self.force_guidance_hud_margin_px = _pair(
            force_guidance_hud_margin_px,
            [40, 40],
        )
        self.force_guidance_hud_offset_px = _pair(
            force_guidance_hud_offset_px,
            [0, 0],
        )
        self.force_guidance_hud_center_norm = _pair(
            force_guidance_hud_center_norm,
            [0.78, 0.28],
        )
        self.force_guidance_basis_mode = str(force_guidance_basis_mode)
        if self.force_guidance_basis_mode not in {
            "camera_screen",
            "task_world",
            "manual_world",
            "sensor_debug",
        }:
            self.force_guidance_basis_mode = "camera_screen"
        self.force_guidance_basis_camera = str(force_guidance_basis_camera)
        self.force_guidance_screen_right_sign = float(force_guidance_screen_right_sign)
        self.force_guidance_screen_up_sign = float(force_guidance_screen_up_sign)
        self.force_guidance_vector_semantics = str(force_guidance_vector_semantics)
        if self.force_guidance_vector_semantics not in {"contact", "correction"}:
            self.force_guidance_vector_semantics = "contact"
        self.force_guidance_correction_sign = float(force_guidance_correction_sign)
        self.wrench_label = str(wrench_label)

        if self.display_mode not in {"overlay", "window", "off"}:
            self.display_mode = "overlay"

        if not (
            0.0 <= self.low_threshold
            <= self.medium_threshold
            <= self.high_threshold
            <= self.excessive_threshold
        ):
            self.low_threshold = 10.0
            self.medium_threshold = 40.0
            self.high_threshold = 80.0
            self.excessive_threshold = 100.0

    @classmethod
    def from_dict(cls, config: Dict[str, Any]) -> "ForceFeedbackConfig":
        return cls(
            enabled=config.get("enable_force_visual_feedback", False),
            display_mode=config.get("force_feedback_display_mode", "overlay"),
            overlay_cameras=config.get("force_feedback_overlay_cameras", ["cctv_cam"]),
            low_threshold=config.get("force_feedback_low_threshold", 10.0),
            medium_threshold=config.get("force_feedback_medium_threshold", 40.0),
            high_threshold=config.get("force_feedback_high_threshold", 80.0),
            excessive_threshold=config.get("force_feedback_excessive_threshold", 100.0),
            show_numbers=config.get("force_feedback_show_numbers", True),
            show_axial_lateral=config.get("force_feedback_show_axial_lateral", True),
            show_trend=config.get(
                "force_feedback_show_trend",
                config.get("show_trend", True),
            ),
            show_contact_state=config.get(
                "force_feedback_show_contact_state",
                config.get("show_contact_state", True),
            ),
            show_arrow=config.get("force_feedback_show_arrow", False),
            smoothing_alpha=config.get("force_feedback_smoothing_alpha", 0.25),
            window_name=config.get("force_feedback_window_name", "Force Feedback HUD"),
            insertion_axis_world=config.get(
                "force_feedback_insertion_axis_world",
                [0.0, -1.0, 0.0],
            ),
            use_compensated_wrench=config.get(
                "force_feedback_use_compensated_wrench",
                True,
            ),
            trend_window_sec=config.get("force_feedback_trend_window_sec", 0.8),
            trend_rising_threshold=config.get(
                "force_feedback_trend_rising_threshold",
                5.0,
            ),
            trend_falling_threshold=config.get(
                "force_feedback_trend_falling_threshold",
                -5.0,
            ),
            contact_free_threshold=config.get(
                "force_feedback_contact_free_threshold",
                5.0,
            ),
            contact_light_threshold=config.get(
                "force_feedback_contact_light_threshold",
                15.0,
            ),
            axial_high_threshold=config.get(
                "force_feedback_axial_high_threshold",
                20.0,
            ),
            lateral_high_threshold=config.get(
                "force_feedback_lateral_high_threshold",
                10.0,
            ),
            jam_force_threshold=config.get(
                "force_feedback_jam_force_threshold",
                25.0,
            ),
            jam_lateral_threshold=config.get(
                "force_feedback_jam_lateral_threshold",
                12.0,
            ),
            enable_task_force_guidance_hud=config.get(
                "enable_task_force_guidance_hud",
                False,
            ),
            task_force_guidance_mode=config.get(
                "task_force_guidance_mode",
                "ring",
            ),
            force_guidance_overlay_cameras=config.get(
                "force_guidance_overlay_cameras",
                config.get("force_feedback_overlay_cameras", ["cctv_cam"]),
            ),
            show_translation_ring=config.get("show_translation_ring", True),
            show_torque_ring=config.get("show_torque_ring", True),
            show_axial_core=config.get("show_axial_core", True),
            force_guidance_plane_right_world=config.get(
                "force_guidance_plane_right_world",
                [1.0, 0.0, 0.0],
            ),
            force_guidance_plane_up_world=config.get(
                "force_guidance_plane_up_world",
                [0.0, 0.0, 1.0],
            ),
            force_guidance_ring_radius_px=config.get(
                "force_guidance_ring_radius_px",
                70,
            ),
            force_guidance_max_vector_px=config.get(
                "force_guidance_max_vector_px",
                55,
            ),
            force_guidance_max_force_n=config.get(
                "force_guidance_max_force_n",
                40.0,
            ),
            force_guidance_max_torque_nm=config.get(
                "force_guidance_max_torque_nm",
                2.0,
            ),
            torque_guidance_mode=config.get("torque_guidance_mode", "tilt_axes"),
            torque_guidance_min_force_n=config.get("torque_guidance_min_force_n", 5.0),
            torque_guidance_min_torque_nm=config.get(
                "torque_guidance_min_torque_nm",
                0.05,
            ),
            torque_guidance_max_torque_nm=config.get(
                "torque_guidance_max_torque_nm",
                config.get("force_guidance_max_torque_nm", 2.0),
            ),
            torque_guidance_label_as_posture=config.get(
                "torque_guidance_label_as_posture",
                True,
            ),
            torque_guidance_show_numeric_values=config.get(
                "torque_guidance_show_numeric_values",
                True,
            ),
            force_guidance_draw_numeric_values=config.get(
                "force_guidance_draw_numeric_values",
                True,
            ),
            force_guidance_show_caveat_label=config.get(
                "force_guidance_show_caveat_label",
                True,
            ),
            force_guidance_hud_anchor=config.get(
                "force_guidance_hud_anchor",
                "top_right",
            ),
            force_guidance_hud_margin_px=config.get(
                "force_guidance_hud_margin_px",
                [40, 40],
            ),
            force_guidance_hud_offset_px=config.get(
                "force_guidance_hud_offset_px",
                [0, 0],
            ),
            force_guidance_hud_center_norm=config.get(
                "force_guidance_hud_center_norm",
                [0.78, 0.28],
            ),
            force_guidance_basis_mode=config.get(
                "force_guidance_basis_mode",
                "camera_screen",
            ),
            force_guidance_basis_camera=config.get(
                "force_guidance_basis_camera",
                "cctv_cam",
            ),
            force_guidance_screen_right_sign=config.get(
                "force_guidance_screen_right_sign",
                1.0,
            ),
            force_guidance_screen_up_sign=config.get(
                "force_guidance_screen_up_sign",
                1.0,
            ),
            force_guidance_vector_semantics=config.get(
                "force_guidance_vector_semantics",
                "contact",
            ),
            force_guidance_correction_sign=config.get(
                "force_guidance_correction_sign",
                -1.0,
            ),
            wrench_label=config.get("force_feedback_wrench_label", "comp"),
        )


class ForceFeedbackSmoother:
    def __init__(self, alpha: float = 0.25):
        self.alpha = float(np.clip(alpha, 0.0, 1.0))
        self.force_sensor: Optional[np.ndarray] = None

    def update(self, force_sensor: np.ndarray) -> np.ndarray:
        force_sensor = np.asarray(force_sensor, dtype=float).reshape(3)
        if self.force_sensor is None or self.alpha >= 1.0:
            self.force_sensor = force_sensor.copy()
        elif self.alpha <= 0.0:
            pass
        else:
            self.force_sensor = (
                self.alpha * force_sensor
                + (1.0 - self.alpha) * self.force_sensor
            )
        return self.force_sensor.copy()


def compute_force_feedback(
    force_sensor: np.ndarray,
    config: ForceFeedbackConfig,
    source_label: str = "raw",
    R_ws: Optional[np.ndarray] = None,
    trend: str = "STABLE",
    torque_sensor: Optional[np.ndarray] = None,
    guidance_right_world: Optional[np.ndarray] = None,
    guidance_up_world: Optional[np.ndarray] = None,
    guidance_basis_label: Optional[str] = None,
) -> Dict[str, Any]:
    force_sensor = np.asarray(force_sensor, dtype=float).reshape(3)
    force_norm = float(np.linalg.norm(force_sensor))

    feedback: Dict[str, Any] = {
        "force_sensor": force_sensor,
        "force_norm": force_norm,
        "source_label": source_label,
        "band": risk_band(force_norm, config),
        "axial": None,
        "lateral": None,
        "trend": trend,
        "contact_state": "FREE",
        "force_world": None,
        "torque_sensor": None,
        "torque_world": None,
        "lateral_uv": None,
        "torque_uv": None,
        "basis_label": guidance_basis_label or config.force_guidance_basis_mode,
        "vector_semantics": config.force_guidance_vector_semantics,
    }

    if config.force_guidance_basis_mode == "sensor_debug":
        feedback["axial"] = float(force_sensor[2])
        feedback["lateral"] = float(np.linalg.norm(force_sensor[:2]))
        feedback["lateral_uv"] = _apply_vector_semantics(
            np.asarray(force_sensor[:2], dtype=float),
            config,
        )
        if torque_sensor is not None:
            torque_sensor = np.asarray(torque_sensor, dtype=float).reshape(3)
            feedback["torque_sensor"] = torque_sensor
            feedback["torque_uv"] = _apply_vector_semantics(
                np.asarray(torque_sensor[:2], dtype=float),
                config,
            )
        feedback["basis_label"] = "sensor_debug"
        feedback["contact_state"] = contact_state_label(feedback, config)
        return feedback

    if R_ws is not None:
        R_ws = np.asarray(R_ws, dtype=float).reshape(3, 3)
        force_world = R_ws @ force_sensor
        feedback["force_world"] = force_world
        axis = config.insertion_axis_world
        axial = float(np.dot(force_world, axis))
        lateral_vec = force_world - axial * axis
        feedback["axial"] = axial
        feedback["lateral"] = float(np.linalg.norm(lateral_vec))
        right_world = (
            np.asarray(guidance_right_world, dtype=float).reshape(3)
            if guidance_right_world is not None
            else config.force_guidance_plane_right_world
        )
        up_world = (
            np.asarray(guidance_up_world, dtype=float).reshape(3)
            if guidance_up_world is not None
            else config.force_guidance_plane_up_world
        )
        feedback["lateral_uv"] = np.array(
            [
                float(np.dot(lateral_vec, right_world)),
                float(np.dot(lateral_vec, up_world)),
            ],
            dtype=float,
        )
        feedback["lateral_uv"] = _apply_vector_semantics(
            feedback["lateral_uv"],
            config,
        )

        if torque_sensor is not None:
            torque_sensor = np.asarray(torque_sensor, dtype=float).reshape(3)
            torque_world = R_ws @ torque_sensor
            feedback["torque_sensor"] = torque_sensor
            feedback["torque_world"] = torque_world
            feedback["torque_uv"] = np.array(
                [
                    float(np.dot(torque_world, right_world)),
                    float(np.dot(torque_world, up_world)),
                ],
                dtype=float,
            )
            feedback["torque_uv"] = _apply_vector_semantics(
                feedback["torque_uv"],
                config,
            )

    feedback["contact_state"] = contact_state_label(feedback, config)
    return feedback


def contact_state_label(
    feedback: Dict[str, Any],
    config: ForceFeedbackConfig,
) -> str:
    force_norm = float(feedback.get("force_norm", 0.0))
    axial = feedback.get("axial")
    lateral = feedback.get("lateral")
    trend = feedback.get("trend", "STABLE")

    if force_norm < config.contact_free_threshold:
        return "FREE"
    if force_norm < config.contact_light_threshold:
        return "LIGHT CONTACT"

    if force_norm >= config.excessive_threshold:
        return "HIGH FORCE"

    if axial is None or lateral is None:
        if force_norm >= config.high_threshold:
            return "HIGH FORCE"
        return "LIGHT CONTACT"

    axial_abs = abs(float(axial))
    lateral_abs = abs(float(lateral))

    if (
        force_norm >= config.jam_force_threshold
        and lateral_abs >= config.jam_lateral_threshold
        and trend == "RISING"
    ):
        return "JAM RISK"
    if axial_abs >= config.axial_high_threshold and lateral_abs >= config.lateral_high_threshold:
        return "MIXED CONTACT"
    if lateral_abs >= config.lateral_high_threshold:
        return "LATERAL CONTACT"
    if axial_abs >= config.axial_high_threshold:
        return "AXIAL LOAD"
    return "LIGHT CONTACT"


def draw_force_feedback_overlay(
    frame_bgr: np.ndarray,
    feedback: Dict[str, Any],
    config: ForceFeedbackConfig,
    camera_name: str = "",
) -> np.ndarray:
    if frame_bgr is None or feedback is None:
        return frame_bgr

    if (
        config.enable_task_force_guidance_hud
        and config.task_force_guidance_mode == "ring"
    ):
        return draw_force_guidance_ring_overlay(
            frame_bgr=frame_bgr,
            feedback=feedback,
            config=config,
            camera_name=camera_name,
        )

    h, w = frame_bgr.shape[:2]
    x0, y0 = 14, 48
    panel_w = min(390, max(300, w - 28))
    panel_h = 168
    if not config.show_axial_lateral:
        panel_h -= 44
    if not config.show_trend:
        panel_h -= 22
    if not config.show_contact_state:
        panel_h -= 24

    band = feedback["band"]
    color = band["color_bgr"]
    force_norm = float(feedback["force_norm"])
    contact_state = feedback.get("contact_state", "FREE")
    contact_color = contact_state_color(contact_state, band)

    overlay = frame_bgr.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + panel_w, y0 + panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.42, frame_bgr, 0.58, 0.0, frame_bgr)

    cv2.rectangle(frame_bgr, (x0, y0), (x0 + panel_w, y0 + panel_h), color, 2)
    cv2.rectangle(frame_bgr, (x0, y0), (x0 + 10, y0 + panel_h), color, -1)

    title = "Force HUD"
    if camera_name:
        title = f"{title} - {camera_name}"

    y = y0 + 24
    _put_text(frame_bgr, title, (x0 + 20, y), 0.58, (235, 235, 235), 1)
    _put_text(
        frame_bgr,
        band["label"],
        (x0 + panel_w - 118, y),
        0.58,
        color,
        2,
    )

    y += 28
    if config.show_numbers:
        force_text = f"|F| {force_norm:5.1f} N ({feedback['source_label']})"
    else:
        force_text = f"{feedback['source_label']}"

    _put_text(frame_bgr, force_text, (x0 + 20, y), 0.70, color, 2)

    y += 14
    _draw_bar(
        frame_bgr,
        x0 + 20,
        y,
        panel_w - 40,
        10,
        force_norm,
        config.excessive_threshold,
        color,
    )

    if config.show_axial_lateral:
        y += 28
        axial = feedback.get("axial")
        lateral = feedback.get("lateral")
        if axial is None or lateral is None:
            _put_text(
                frame_bgr,
                "Axial/Lateral unavailable",
                (x0 + 20, y),
                0.55,
                (210, 210, 210),
                1,
            )
        else:
            axial_abs = abs(float(axial))
            lateral_abs = abs(float(lateral))
            label_x = x0 + 20
            value_x = x0 + 96
            bar_x = x0 + 178
            bar_w = panel_w - 198

            _put_text(frame_bgr, "Axial", (label_x, y), 0.55, (230, 230, 230), 1)
            _put_text(frame_bgr, f"{axial:+5.1f} N", (value_x, y), 0.55, (230, 230, 230), 1)
            _draw_bar(
                frame_bgr,
                bar_x,
                y - 9,
                bar_w,
                9,
                axial_abs,
                config.jam_force_threshold,
                color,
            )

            y += 22
            _put_text(frame_bgr, "Lateral", (label_x, y), 0.55, (230, 230, 230), 1)
            _put_text(frame_bgr, f"{lateral_abs:5.1f} N", (value_x, y), 0.55, (230, 230, 230), 1)
            _draw_bar(
                frame_bgr,
                bar_x,
                y - 9,
                bar_w,
                9,
                lateral_abs,
                config.jam_lateral_threshold,
                contact_color,
            )

    if config.show_trend:
        y += 24
        trend = feedback.get("trend", "STABLE")
        _put_text(
            frame_bgr,
            f"Trend   {trend}",
            (x0 + 20, y),
            0.58,
            trend_color(trend),
            2 if trend == "RISING" else 1,
        )

    if config.show_contact_state:
        y += 24
        _put_text(
            frame_bgr,
            f"Contact {contact_state}",
            (x0 + 20, y),
            0.60,
            contact_color,
            2,
        )

    return frame_bgr


def draw_force_guidance_ring_overlay(
    frame_bgr: np.ndarray,
    feedback: Dict[str, Any],
    config: ForceFeedbackConfig,
    camera_name: str = "",
) -> np.ndarray:
    h, w = frame_bgr.shape[:2]
    radius = int(config.force_guidance_ring_radius_px)
    band = feedback["band"]
    force_norm = float(feedback["force_norm"])
    axial = feedback.get("axial")
    lateral = feedback.get("lateral")
    torque_uv = feedback.get("torque_uv")
    lateral_uv = feedback.get("lateral_uv")
    trend = feedback.get("trend", "STABLE")
    contact_state = feedback.get("contact_state", "FREE")
    contact_color = contact_state_color(contact_state, band)

    panel_w = max(2 * radius + 62, 300)
    panel_h = 2 * radius + 126
    show_torque_feedback = (
        config.show_torque_ring
        and config.torque_guidance_mode != "off"
        and torque_uv is not None
    )
    if show_torque_feedback and config.torque_guidance_mode == "tilt_axes":
        panel_h += 62
    cx, cy = compute_guidance_hud_center(w, h, config, panel_w, panel_h)
    x0 = int(np.clip(cx - panel_w // 2, 10, max(10, w - panel_w - 10)))
    y0 = max(10, cy - radius - 48)

    overlay = frame_bgr.copy()
    cv2.rectangle(
        overlay,
        (x0, y0),
        (min(w - 1, x0 + panel_w), min(h - 1, y0 + panel_h)),
        (0, 0, 0),
        -1,
    )
    cv2.addWeighted(overlay, 0.40, frame_bgr, 0.60, 0.0, frame_bgr)

    title = "Peg Contact HUD"
    if camera_name:
        title = f"{title} - {camera_name}"
    _put_text(frame_bgr, title, (x0 + 14, y0 + 24), 0.52, (235, 235, 235), 1)
    _put_text(
        frame_bgr,
        f"{feedback.get('source_label', 'raw')}",
        (x0 + panel_w - 62, y0 + 24),
        0.52,
        band["color_bgr"],
        2,
    )
    _put_text(frame_bgr, "Position cue", (x0 + 14, y0 + 44), 0.44, (205, 205, 205), 1)

    cv2.circle(frame_bgr, (cx, cy), radius, (92, 92, 92), 1, cv2.LINE_AA)
    cv2.circle(frame_bgr, (cx, cy), max(8, radius // 2), (68, 68, 68), 1, cv2.LINE_AA)
    cv2.line(frame_bgr, (cx - radius, cy), (cx + radius, cy), (55, 55, 55), 1, cv2.LINE_AA)
    cv2.line(frame_bgr, (cx, cy - radius), (cx, cy + radius), (55, 55, 55), 1, cv2.LINE_AA)

    if config.show_axial_core:
        axial_abs = abs(float(axial)) if axial is not None else 0.0
        core_scale = np.clip(axial_abs / config.force_guidance_max_force_n, 0.0, 1.0)
        core_radius = int(12 + core_scale * 16)
        core_color = risk_band(axial_abs, config)["color_bgr"]
        cv2.circle(frame_bgr, (cx, cy), core_radius, core_color, -1, cv2.LINE_AA)
        cv2.circle(frame_bgr, (cx, cy), core_radius, (240, 240, 240), 1, cv2.LINE_AA)

    if config.show_translation_ring and lateral_uv is not None:
        end = _vector_endpoint(
            cx,
            cy,
            lateral_uv,
            config.force_guidance_max_force_n,
            config.force_guidance_max_vector_px,
        )
        cv2.arrowedLine(
            frame_bgr,
            (cx, cy),
            end,
            contact_color,
            3,
            cv2.LINE_AA,
            tipLength=0.22,
        )
        cv2.circle(frame_bgr, end, 5, contact_color, -1, cv2.LINE_AA)

    if show_torque_feedback and config.torque_guidance_mode == "vector":
        end = _vector_endpoint(
            cx,
            cy,
            torque_uv,
            config.force_guidance_max_torque_nm,
            radius,
        )
        cv2.circle(frame_bgr, (cx, cy), radius + 10, (74, 74, 74), 1, cv2.LINE_AA)
        cv2.line(frame_bgr, (cx, cy), end, (255, 185, 80), 2, cv2.LINE_AA)
        cv2.circle(frame_bgr, end, 4, (255, 185, 80), -1, cv2.LINE_AA)

    y = cy + radius + 24
    _put_text(
        frame_bgr,
        f"|F| {force_norm:4.1f} N  {band['label']}",
        (x0 + 14, y),
        0.53,
        band["color_bgr"],
        2,
    )
    y += 22
    if axial is None or lateral is None:
        _put_text(frame_bgr, "Ax/Lat unavailable", (x0 + 14, y), 0.50, (210, 210, 210), 1)
    else:
        _put_text(
            frame_bgr,
            f"Ax {float(axial):+4.1f} N  Lat {float(lateral):4.1f} N",
            (x0 + 14, y),
            0.50,
            (230, 230, 230),
            1,
        )

    if show_torque_feedback:
        y += 22
        if config.torque_guidance_mode == "tilt_axes":
            y = _draw_torque_posture_panel(
                frame_bgr,
                x0 + 14,
                y,
                panel_w - 28,
                feedback,
                config,
            )
        elif config.force_guidance_draw_numeric_values:
            torque_mag = float(np.linalg.norm(torque_uv))
            _put_text(
                frame_bgr,
                f"Torque vector {torque_mag:4.2f} Nm",
                (x0 + 14, y),
                0.48,
                (255, 210, 130),
                1,
            )

    y += 22
    _put_text(frame_bgr, f"{trend}  {contact_state}", (x0 + 14, y), 0.54, contact_color, 2)

    if config.force_guidance_show_caveat_label:
        y += 20
        basis_label = feedback.get("basis_label", config.force_guidance_basis_mode)
        semantics = feedback.get("vector_semantics", config.force_guidance_vector_semantics)
        _put_text(
            frame_bgr,
            f"{semantics} cue | basis: {basis_label}",
            (x0 + 14, y),
            0.42,
            (190, 190, 190),
            1,
        )

    return frame_bgr


def _draw_torque_posture_panel(
    frame_bgr: np.ndarray,
    x: int,
    y: int,
    width: int,
    feedback: Dict[str, Any],
    config: ForceFeedbackConfig,
) -> int:
    torque_uv = np.asarray(feedback.get("torque_uv"), dtype=float).reshape(2)
    force_norm = float(feedback.get("force_norm", 0.0))
    torque_mag = float(np.linalg.norm(torque_uv))
    force_active = force_norm >= config.torque_guidance_min_force_n
    torque_active = torque_mag >= config.torque_guidance_min_torque_nm
    active = force_active and torque_active
    color = (255, 210, 130) if active else (115, 115, 115)
    text_color = (230, 230, 230) if active else (150, 150, 150)
    label = "Posture cue" if config.torque_guidance_label_as_posture else "Tilt cue"
    status = "" if active else " inactive"

    _put_text(frame_bgr, f"{label}{status}", (x, y), 0.48, text_color, 1)

    bar_x = x + 78
    bar_w = max(74, min(140, width - 118))
    center_x = bar_x + bar_w // 2
    half_w = bar_w // 2
    max_torque = max(config.torque_guidance_max_torque_nm, 1e-9)

    y += 20
    _put_text(frame_bgr, "T right", (x, y + 4), 0.42, text_color, 1)
    _draw_signed_bar(
        frame_bgr,
        center_x,
        y - 8,
        half_w,
        10,
        float(torque_uv[0]),
        max_torque,
        color,
    )
    if config.torque_guidance_show_numeric_values:
        _put_text(
            frame_bgr,
            f"{float(torque_uv[0]):+4.2f}",
            (bar_x + bar_w + 8, y + 4),
            0.38,
            text_color,
            1,
        )

    y += 20
    _put_text(frame_bgr, "T up", (x, y + 4), 0.42, text_color, 1)
    _draw_signed_bar(
        frame_bgr,
        center_x,
        y - 8,
        half_w,
        10,
        float(torque_uv[1]),
        max_torque,
        color,
    )
    if config.torque_guidance_show_numeric_values:
        _put_text(
            frame_bgr,
            f"{float(torque_uv[1]):+4.2f}",
            (bar_x + bar_w + 8, y + 4),
            0.38,
            text_color,
            1,
        )

    return y


def compute_guidance_hud_center(
    canvas_width: int,
    canvas_height: int,
    config: ForceFeedbackConfig,
    panel_width: Optional[int] = None,
    panel_height: Optional[int] = None,
):
    radius = int(config.force_guidance_ring_radius_px)
    panel_width = int(panel_width if panel_width is not None else 2 * radius + 62)
    panel_height = int(panel_height if panel_height is not None else 2 * radius + 126)

    margin_x, margin_y = config.force_guidance_hud_margin_px
    offset_x, offset_y = config.force_guidance_hud_offset_px

    half_w = panel_width / 2.0
    # The ring center is above the panel center because text lives below it.
    top_to_center_y = radius + 48
    min_cx = half_w
    max_cx = max(min_cx, canvas_width - half_w)
    min_cy = top_to_center_y
    max_cy = max(min_cy, canvas_height - (panel_height - top_to_center_y))

    anchor = config.force_guidance_hud_anchor
    if anchor == "custom":
        norm_x, norm_y = config.force_guidance_hud_center_norm
        cx = float(canvas_width) * norm_x
        cy = float(canvas_height) * norm_y
    elif anchor == "center":
        cx = float(canvas_width) * 0.5
        cy = float(canvas_height) * 0.5
    else:
        left = anchor.endswith("left")
        top = anchor.startswith("top")
        cx = margin_x + half_w if left else canvas_width - margin_x - half_w
        cy = margin_y + top_to_center_y if top else canvas_height - margin_y - (panel_height - top_to_center_y)

    cx += offset_x
    cy += offset_y

    cx = int(round(np.clip(cx, min_cx, max_cx)))
    cy = int(round(np.clip(cy, min_cy, max_cy)))
    return cx, cy


def make_force_feedback_hud(
    feedback: Dict[str, Any],
    config: ForceFeedbackConfig,
) -> np.ndarray:
    frame = np.zeros((170, 340, 3), dtype=np.uint8)
    return draw_force_feedback_overlay(
        frame_bgr=frame,
        feedback=feedback,
        config=config,
        camera_name="",
    )


def resize_with_aspect_padding(
    frame_bgr: np.ndarray,
    target_width: int,
    target_height: int,
    padding_color=None,
) -> np.ndarray:
    if frame_bgr is None:
        return frame_bgr

    target_width = max(1, int(target_width))
    target_height = max(1, int(target_height))
    image_height, image_width = frame_bgr.shape[:2]
    if image_width <= 0 or image_height <= 0:
        return np.zeros((target_height, target_width, 3), dtype=np.uint8)

    color = _padding_color_bgr(padding_color)
    scale = min(target_width / image_width, target_height / image_height)
    resized_width = max(1, int(round(image_width * scale)))
    resized_height = max(1, int(round(image_height * scale)))

    resized = cv2.resize(
        frame_bgr,
        (resized_width, resized_height),
        interpolation=cv2.INTER_LINEAR,
    )

    canvas = np.empty((target_height, target_width, 3), dtype=frame_bgr.dtype)
    canvas[:, :] = color

    x0 = (target_width - resized_width) // 2
    y0 = (target_height - resized_height) // 2
    canvas[y0:y0 + resized_height, x0:x0 + resized_width] = resized
    return canvas


def risk_band(force_norm: float, config: ForceFeedbackConfig) -> Dict[str, Any]:
    if force_norm >= config.excessive_threshold:
        return {"label": "EXCESS", "color_bgr": (40, 40, 255)}
    if force_norm >= config.high_threshold:
        return {"label": "HIGH", "color_bgr": (0, 140, 255)}
    if force_norm >= config.medium_threshold:
        return {"label": "MED", "color_bgr": (0, 230, 255)}
    return {"label": "SAFE", "color_bgr": (60, 220, 60)}


def trend_label(delta_force: float, config: ForceFeedbackConfig) -> str:
    if delta_force >= config.trend_rising_threshold:
        return "RISING"
    if delta_force <= config.trend_falling_threshold:
        return "FALLING"
    return "STABLE"


def trend_color(trend: str):
    if trend == "RISING":
        return (0, 180, 255)
    if trend == "FALLING":
        return (180, 220, 120)
    return (220, 220, 220)


def contact_state_color(contact_state: str, band: Dict[str, Any]):
    if contact_state == "FREE":
        return (80, 220, 80)
    if contact_state == "LIGHT CONTACT":
        return (0, 230, 255)
    if contact_state == "AXIAL LOAD":
        return (255, 210, 80)
    if contact_state == "LATERAL CONTACT":
        return (0, 170, 255)
    if contact_state == "MIXED CONTACT":
        return (0, 120, 255)
    if contact_state == "JAM RISK":
        return (0, 0, 255)
    if contact_state == "HIGH FORCE":
        return (40, 40, 255)
    return band["color_bgr"]


def _normalized(vec) -> np.ndarray:
    out = np.asarray(vec, dtype=float).reshape(3)
    norm = float(np.linalg.norm(out))
    if norm < 1e-9:
        return np.array([0.0, -1.0, 0.0], dtype=float)
    return out / norm


def _string_list(value, default):
    if value is None:
        return list(default)
    if isinstance(value, str):
        return [value]
    return [str(item) for item in value]


def _pair(value, default):
    if value is None:
        return [float(default[0]), float(default[1])]
    if isinstance(value, (int, float)):
        scalar = float(value)
        return [scalar, scalar]

    arr = list(value)
    if len(arr) < 2:
        return [float(default[0]), float(default[1])]
    return [float(arr[0]), float(arr[1])]


def _padding_color_bgr(value) -> np.ndarray:
    if value is None:
        return np.array([0, 0, 0], dtype=np.uint8)

    color = np.asarray(value, dtype=float).reshape(-1)
    if color.size < 3:
        return np.array([0, 0, 0], dtype=np.uint8)

    return np.asarray(np.clip(color[:3], 0, 255), dtype=np.uint8)


def _scaled_bar_length(value: float, max_value: float, width: int) -> int:
    if max_value <= 1e-9:
        return 0
    return int(np.clip(value / max_value, 0.0, 1.0) * width)


def _draw_bar(frame, x, y, width, height, value, max_value, color):
    fill = _scaled_bar_length(value, max_value, width)
    cv2.rectangle(frame, (x, y), (x + width, y + height), (70, 70, 70), 1)
    cv2.rectangle(frame, (x, y), (x + fill, y + height), color, -1)


def _draw_signed_bar(frame, center_x, y, half_width, height, value, max_value, color):
    x0 = int(center_x - half_width)
    x1 = int(center_x + half_width)
    cv2.rectangle(frame, (x0, y), (x1, y + height), (70, 70, 70), 1)
    cv2.line(frame, (center_x, y - 2), (center_x, y + height + 2), (155, 155, 155), 1)

    if max_value <= 1e-9:
        return

    magnitude = int(np.clip(abs(float(value)) / max_value, 0.0, 1.0) * half_width)
    if magnitude <= 0:
        return

    if value >= 0.0:
        cv2.rectangle(frame, (center_x, y), (center_x + magnitude, y + height), color, -1)
    else:
        cv2.rectangle(frame, (center_x - magnitude, y), (center_x, y + height), color, -1)


def _apply_vector_semantics(vec, config: ForceFeedbackConfig):
    vec = np.asarray(vec, dtype=float).reshape(2)
    if config.force_guidance_vector_semantics == "correction":
        return vec * config.force_guidance_correction_sign
    return vec


def _vector_endpoint(cx: int, cy: int, vec, max_value: float, max_px: int):
    vec = np.asarray(vec, dtype=float).reshape(2)
    norm = float(np.linalg.norm(vec))
    if norm < 1e-9:
        return int(cx), int(cy)

    scale = min(norm / max(max_value, 1e-9), 1.0)
    disp = vec / norm * scale * float(max_px)
    return int(round(cx + disp[0])), int(round(cy - disp[1]))


def _put_text(frame, text, org, scale, color, thickness):
    cv2.putText(
        frame,
        str(text),
        org,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )
