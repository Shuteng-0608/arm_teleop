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
        show_arrow: bool = False,
        smoothing_alpha: float = 0.25,
        window_name: str = "Force Feedback HUD",
        insertion_axis_world=None,
        use_compensated_wrench: bool = False,
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
        self.show_arrow = bool(show_arrow)
        self.smoothing_alpha = float(np.clip(smoothing_alpha, 0.0, 1.0))
        self.window_name = str(window_name)
        self.insertion_axis_world = _normalized(
            insertion_axis_world if insertion_axis_world is not None else [0.0, -1.0, 0.0]
        )
        self.use_compensated_wrench = bool(use_compensated_wrench)

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
            show_arrow=config.get("force_feedback_show_arrow", False),
            smoothing_alpha=config.get("force_feedback_smoothing_alpha", 0.25),
            window_name=config.get("force_feedback_window_name", "Force Feedback HUD"),
            insertion_axis_world=config.get(
                "force_feedback_insertion_axis_world",
                [0.0, -1.0, 0.0],
            ),
            use_compensated_wrench=config.get(
                "force_feedback_use_compensated_wrench",
                False,
            ),
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
    }

    if config.show_axial_lateral and R_ws is not None:
        R_ws = np.asarray(R_ws, dtype=float).reshape(3, 3)
        force_world = R_ws @ force_sensor
        axis = config.insertion_axis_world
        axial = float(np.dot(force_world, axis))
        lateral_vec = force_world - axial * axis
        feedback["axial"] = axial
        feedback["lateral"] = float(np.linalg.norm(lateral_vec))

    return feedback


def draw_force_feedback_overlay(
    frame_bgr: np.ndarray,
    feedback: Dict[str, Any],
    config: ForceFeedbackConfig,
    camera_name: str = "",
) -> np.ndarray:
    if frame_bgr is None or feedback is None:
        return frame_bgr

    h, w = frame_bgr.shape[:2]
    x0, y0 = 14, 48
    panel_w = min(310, max(240, w - 28))
    panel_h = 118 if config.show_axial_lateral else 86

    band = feedback["band"]
    color = band["color_bgr"]
    force_norm = float(feedback["force_norm"])

    overlay = frame_bgr.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + panel_w, y0 + panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.42, frame_bgr, 0.58, 0.0, frame_bgr)

    cv2.rectangle(frame_bgr, (x0, y0), (x0 + panel_w, y0 + panel_h), color, 2)
    cv2.rectangle(frame_bgr, (x0, y0), (x0 + 10, y0 + panel_h), color, -1)

    title = "Force HUD"
    if camera_name:
        title = f"{title} - {camera_name}"

    _put_text(frame_bgr, title, (x0 + 20, y0 + 24), 0.58, (235, 235, 235), 1)
    if config.show_numbers:
        force_text = f"|F| {force_norm:5.1f} N ({feedback['source_label']})"
    else:
        force_text = f"{feedback['source_label']}"

    _put_text(frame_bgr, force_text, (x0 + 20, y0 + 52), 0.72, color, 2)
    _put_text(
        frame_bgr,
        band["label"],
        (x0 + panel_w - 118, y0 + 24),
        0.58,
        color,
        2,
    )

    bar_x, bar_y = x0 + 20, y0 + 66
    bar_w, bar_h = panel_w - 40, 12
    fill = _scaled_bar_length(force_norm, config.excessive_threshold, bar_w)
    cv2.rectangle(frame_bgr, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (70, 70, 70), 1)
    cv2.rectangle(frame_bgr, (bar_x, bar_y), (bar_x + fill, bar_y + bar_h), color, -1)

    if config.show_axial_lateral:
        axial = feedback.get("axial")
        lateral = feedback.get("lateral")
        if axial is None or lateral is None:
            detail = "axial/lateral: unavailable"
        else:
            detail = f"axial {axial:+5.1f} N   lateral {lateral:5.1f} N"
        _put_text(frame_bgr, detail, (x0 + 20, y0 + 104), 0.55, (230, 230, 230), 1)

    return frame_bgr


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
