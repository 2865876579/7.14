"""Desktop OpenCV tuner for the MaixCAM shape detector.

The ROI is intentionally fixed to the values used by the device application.
All other active detection parameters can be changed while frames are running.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import cv2
import numpy as np


FRAME_W = 320
FRAME_H = 240
DISCOVERY_PORT = 37020
DISCOVERY_MAGIC = "MAIXCAM_TUNER"
DISCOVERY_TIMEOUT_SECONDS = 12.0

# Fixed ROI: not exposed in the tuning UI.
ROI_LEFT_RATIO = 0.25
ROI_RIGHT_RATIO = 0.75
ROI_TOP_RATIO = 0.10
ROI_BOTTOM_RATIO = 1.0

PREVIEW_WINDOW = "MaixCAM - Detection Preview"
MASK_WINDOW = "MaixCAM - Binary Mask"

COLOR_RED = (0, 0, 255)
COLOR_ORANGE = (0, 165, 255)
COLOR_GREEN = (0, 255, 0)
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)
COLOR_ROI = (255, 255, 0)


SLIDER_GROUPS = [
    (
        "LAB 颜色阈值",
        [
            ("L_MIN", "L 最小值", 203, 0, 255, 1),
            ("L_MAX", "L 最大值", 255, 0, 255, 1),
            ("A_MIN", "A 最小值", 98, 0, 255, 1),
            ("A_MAX", "A 最大值", 222, 0, 255, 1),
            ("B_MIN", "B 最小值", 86, 0, 255, 1),
            ("B_MAX", "B 最大值", 166, 0, 255, 1),
        ],
    ),
    (
        "面积和尺寸",
        [
            ("CANDIDATE_MIN_AREA", "候选轮廓最小面积", 40, 0, 5000, 10),
            ("TRIANGLE_MIN_AREA", "三角形最小面积", 80, 0, 5000, 10),
            ("QUADRILATERAL_MIN_AREA", "四边形最小面积", 400, 0, 10000, 10),
            ("ELLIPSE_MIN_AREA", "椭圆最小面积", 40, 0, 5000, 10),
            ("MAX_AREA_RATIO", "最大画面面积占比", 0.85, 0.05, 1.0, 0.01),
            ("MIN_W", "默认最小宽度", 15, 1, 160, 1),
            ("MIN_H", "默认最小高度", 15, 1, 120, 1),
            ("TRIANGLE_MIN_W", "三角形最小宽度", 7, 1, 160, 1),
            ("TRIANGLE_MIN_H", "三角形最小高度", 7, 1, 120, 1),
        ],
    ),
    (
        "形状几何条件",
        [
            ("EDGE_MARGIN", "画面边缘留白", 6, 0, 50, 1),
            ("ROI_EDGE_MARGIN", "ROI 边缘留白", 8, 0, 50, 1),
            ("TRIANGLE_ROI_EDGE_MARGIN", "三角形 ROI 留白", 2, 0, 50, 1),
            ("MAX_ASPECT_RATIO", "最大长宽比", 3.2, 1.0, 10.0, 0.05),
            ("TRIANGLE_MAX_ASPECT_RATIO", "三角形最大长宽比", 4.0, 1.0, 10.0, 0.05),
            ("POLY_AREA_MIN_RATIO", "四边形面积比最小值", 0.78, 0.0, 2.0, 0.01),
            ("POLY_AREA_MAX_RATIO", "四边形面积比最大值", 1.10, 0.0, 2.0, 0.01),
            ("QUAD_EVIDENCE_AREA_RATIO_MIN", "四边形证据面积比最小值", 0.78, 0.0, 2.0, 0.01),
            ("QUAD_EVIDENCE_AREA_RATIO_MAX", "四边形证据面积比最大值", 1.15, 0.0, 2.0, 0.01),
            ("TRI_AREA_MIN_RATIO", "三角形面积比最小值", 0.72, 0.0, 2.0, 0.01),
            ("TRI_AREA_MAX_RATIO", "三角形面积比最大值", 1.12, 0.0, 2.0, 0.01),
            ("TRIANGLE_MIN_SOLIDITY", "三角形最小实心度", 0.72, 0.0, 1.0, 0.01),
            ("TRIANGLE_RECT_EXTENT_MAX", "三角形最大矩形占比", 0.72, 0.0, 1.0, 0.01),
            ("POLYGON_MIN_SOLIDITY", "四边形最小实心度", 0.88, 0.0, 1.0, 0.01),
        ],
    ),
    (
        "二值图和轮廓拟合",
        [
            ("MEDIAN_K", "中值滤波核大小", 3, 1, 15, 2),
            ("MORPH_K", "形态学核大小", 3, 1, 15, 2),
            ("APPROX_EPS_DETAIL", "细节拟合精度", 0.012, 0.001, 0.100, 0.001),
            ("APPROX_EPS", "多边形拟合精度", 0.035, 0.001, 0.150, 0.001),
            ("APPROX_EPS_STRONG", "强拟合精度", 0.055, 0.001, 0.200, 0.001),
        ],
    ),
    (
        "椭圆识别条件",
        [
            ("ELLIPSE_MIN_DETAIL_SIDES", "最少细节边数", 6, 5, 20, 1),
            ("ELLIPSE_FILL_MIN", "椭圆填充比最小值", 0.82, 0.0, 2.0, 0.01),
            ("ELLIPSE_FILL_MAX", "椭圆填充比最大值", 1.08, 0.0, 2.0, 0.01),
            ("ELLIPSE_RECT_EXTENT_MAX", "最大矩形占比", 0.84, 0.0, 1.0, 0.01),
            ("ELLIPSE_AXIS_RATIO_MAX", "最大长短轴比", 2.00, 1.0, 10.0, 0.05),
            ("ELLIPSE_CIRCULARITY_MIN", "最小圆度", 0.76, 0.0, 1.0, 0.01),
            ("ELLIPSE_SOLIDITY_MIN", "最小实心度", 0.93, 0.0, 1.0, 0.01),
            ("ELLIPSE_FAR_AREA_MAX", "远距离判定面积上限", 1600, 50, 5000, 10),
            ("ELLIPSE_FAR_MIN_DETAIL_SIDES", "远距离最少细节边数", 4, 3, 20, 1),
            ("ELLIPSE_FAR_MIN_TIGHT_SIDES", "远距离最少紧密边数", 4, 3, 12, 1),
            ("ELLIPSE_FAR_FILL_MIN", "远距离填充比最小值", 0.65, 0.0, 2.0, 0.01),
            ("ELLIPSE_FAR_FILL_MAX", "远距离填充比最大值", 1.25, 0.0, 2.0, 0.01),
            ("ELLIPSE_FAR_RECT_EXTENT_MAX", "远距离最大矩形占比", 0.93, 0.0, 1.0, 0.01),
            ("ELLIPSE_FAR_AXIS_RATIO_MAX", "远距离最大长短轴比", 2.80, 1.0, 10.0, 0.05),
            ("ELLIPSE_FAR_CIRCULARITY_MIN", "远距离最小圆度", 0.50, 0.0, 1.0, 0.01),
            ("ELLIPSE_FAR_SOLIDITY_MIN", "远距离最小实心度", 0.72, 0.0, 1.0, 0.01),
        ],
    ),
    (
        "识别稳定性",
        [
            ("STABLE_FRAMES_REQUIRED", "连续稳定帧数", 20, 1, 120, 1),
            ("ELLIPSE_STABLE_FRAMES_REQUIRED", "椭圆连续稳定帧数", 8, 1, 120, 1),
        ],
    ),
]

INTEGER_KEYS = {
    "L_MIN",
    "L_MAX",
    "A_MIN",
    "A_MAX",
    "B_MIN",
    "B_MAX",
    "CANDIDATE_MIN_AREA",
    "TRIANGLE_MIN_AREA",
    "QUADRILATERAL_MIN_AREA",
    "ELLIPSE_MIN_AREA",
    "MIN_W",
    "MIN_H",
    "TRIANGLE_MIN_W",
    "TRIANGLE_MIN_H",
    "EDGE_MARGIN",
    "ROI_EDGE_MARGIN",
    "TRIANGLE_ROI_EDGE_MARGIN",
    "MEDIAN_K",
    "MORPH_K",
    "ELLIPSE_MIN_DETAIL_SIDES",
    "ELLIPSE_FAR_AREA_MAX",
    "ELLIPSE_FAR_MIN_DETAIL_SIDES",
    "ELLIPSE_FAR_MIN_TIGHT_SIDES",
    "STABLE_FRAMES_REQUIRED",
    "ELLIPSE_STABLE_FRAMES_REQUIRED",
}

DEFAULTS = {
    key: default
    for _, sliders in SLIDER_GROUPS
    for key, _, default, _, _, _ in sliders
}


def detect_roi_bounds():
    return (
        int(FRAME_W * ROI_LEFT_RATIO),
        int(FRAME_W * ROI_RIGHT_RATIO),
        int(FRAME_H * ROI_TOP_RATIO),
        int(FRAME_H * ROI_BOTTOM_RATIO),
    )


def find_contours(mask):
    result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return result[0] if len(result) == 2 else result[1]


def apply_detect_roi(mask):
    left, right, top, bottom = detect_roi_bounds()
    mask[:top, :] = 0
    if bottom < FRAME_H:
        mask[bottom:, :] = 0
    mask[:, :left] = 0
    mask[:, right:] = 0
    return mask


def clean_mask(mask, params):
    median_k = max(1, int(params["MEDIAN_K"]) | 1)
    morph_k = max(1, int(params["MORPH_K"]) | 1)
    if median_k > 1:
        mask = cv2.medianBlur(mask, median_k)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_k, morph_k))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def approx_hull(contour, eps_ratio):
    hull = cv2.convexHull(contour)
    perimeter = cv2.arcLength(hull, True)
    if perimeter <= 0:
        return hull, perimeter
    return cv2.approxPolyDP(hull, eps_ratio * perimeter, True), perimeter


def contour_center(contour):
    moments = cv2.moments(contour)
    if moments["m00"] != 0:
        return (
            int(moments["m10"] / moments["m00"]),
            int(moments["m01"] / moments["m00"]),
        )
    x, y, w, h = cv2.boundingRect(contour)
    return x + w // 2, y + h // 2


def center_in_detect_roi(contour):
    cx, cy = contour_center(contour)
    left, right, top, bottom = detect_roi_bounds()
    return left <= cx <= right and top <= cy <= bottom


def contour_passes_geometry(
    contour,
    area,
    params,
    min_w=None,
    min_h=None,
    roi_edge_margin=None,
    max_aspect_ratio=None,
):
    min_w = params["MIN_W"] if min_w is None else min_w
    min_h = params["MIN_H"] if min_h is None else min_h
    roi_edge_margin = params["ROI_EDGE_MARGIN"] if roi_edge_margin is None else roi_edge_margin
    max_aspect_ratio = params["MAX_ASPECT_RATIO"] if max_aspect_ratio is None else max_aspect_ratio

    x, y, w, h = cv2.boundingRect(contour)
    left, right, top, bottom = detect_roi_bounds()
    edge_margin = params["EDGE_MARGIN"]

    if w < min_w or h < min_h:
        return False
    if x <= edge_margin or y <= edge_margin:
        return False
    if x + w >= FRAME_W - edge_margin or y + h >= FRAME_H - edge_margin:
        return False
    if x <= left + roi_edge_margin or x + w >= right - roi_edge_margin:
        return False
    if y <= top + roi_edge_margin:
        return False
    if bottom < FRAME_H and y + h >= bottom - roi_edge_margin:
        return False

    aspect = max(w, h) / max(1, min(w, h))
    if aspect > max_aspect_ratio:
        return False
    return w * h > 0 and area / (w * h) >= 0.22


def best_polygon_approx(contour, params):
    candidates = []
    eps_values = (0.025, params["APPROX_EPS"], params["APPROX_EPS_STRONG"], 0.075, 0.10)
    for eps in eps_values:
        approx, _ = approx_hull(contour, eps)
        candidates.append((eps, len(approx), approx))

    first_three = None
    first_four = None
    for eps, sides, approx in candidates:
        if sides == 3 and first_three is None:
            first_three = (eps, approx)
        if sides == 4 and first_four is None:
            first_four = (eps, approx)

    if (
        first_four is not None
        and first_three is not None
        and cv2.contourArea(contour) >= params["QUADRILATERAL_MIN_AREA"]
        and first_four[0] <= params["APPROX_EPS_STRONG"]
        and first_three[0] >= 0.10
    ):
        return first_four[1], 4
    if first_three is not None:
        return first_three[1], 3
    if first_four is not None:
        return first_four[1], 4

    best, best_sides = candidates[0][2], candidates[0][1]
    for _, sides, approx in candidates[1:]:
        if abs(sides - 4) < abs(best_sides - 4):
            best, best_sides = approx, sides
    return best, best_sides


def ellipse_score(contour, params, far_ellipse=False):
    if len(contour) < 5:
        return 0.0
    area = cv2.contourArea(contour)
    (_, _), (axis_a, axis_b), _ = cv2.fitEllipse(contour)
    if axis_a <= 0 or axis_b <= 0:
        return 0.0

    major_axis = max(axis_a, axis_b)
    minor_axis = min(axis_a, axis_b)
    axis_ratio_key = "ELLIPSE_FAR_AXIS_RATIO_MAX" if far_ellipse else "ELLIPSE_AXIS_RATIO_MAX"
    if minor_axis <= 0 or major_axis / minor_axis > params[axis_ratio_key]:
        return 0.0

    ellipse_area = math.pi * axis_a * axis_b * 0.25
    if ellipse_area <= 0:
        return 0.0
    fill_ratio = area / ellipse_area
    fill_min_key = "ELLIPSE_FAR_FILL_MIN" if far_ellipse else "ELLIPSE_FILL_MIN"
    fill_max_key = "ELLIPSE_FAR_FILL_MAX" if far_ellipse else "ELLIPSE_FILL_MAX"
    if not params[fill_min_key] <= fill_ratio <= params[fill_max_key]:
        return 0.0

    perimeter = cv2.arcLength(contour, True)
    if perimeter <= 0:
        return 0.0
    circularity = 4.0 * math.pi * area / (perimeter * perimeter)
    circularity_key = (
        "ELLIPSE_FAR_CIRCULARITY_MIN" if far_ellipse else "ELLIPSE_CIRCULARITY_MIN"
    )
    if circularity < params[circularity_key]:
        return 0.0

    rect_w, rect_h = cv2.minAreaRect(contour)[1]
    rect_area = rect_w * rect_h
    rect_extent_key = (
        "ELLIPSE_FAR_RECT_EXTENT_MAX" if far_ellipse else "ELLIPSE_RECT_EXTENT_MAX"
    )
    if rect_area <= 0 or area / rect_area > params[rect_extent_key]:
        return 0.0
    return fill_ratio * circularity


def classify_shape(contour, params):
    area = cv2.contourArea(contour)
    max_area = FRAME_W * FRAME_H * params["MAX_AREA_RATIO"]
    if area < params["CANDIDATE_MIN_AREA"] or area > max_area:
        return None
    if not center_in_detect_roi(contour):
        return None

    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    if hull_area <= 0:
        return None
    solidity = area / hull_area

    approx_detail, _ = approx_hull(contour, params["APPROX_EPS_DETAIL"])
    detail_sides = len(approx_detail)
    approx_tight, _ = approx_hull(contour, 0.025)
    tight_sides = len(approx_tight)
    approx, sides = best_polygon_approx(contour, params)
    poly_area = cv2.contourArea(approx) if approx is not None else 0
    poly_ratio = poly_area / area if area > 0 else 0
    quadrilateral_evidence = (
        sides == 4
        and tight_sides == 4
        and params["QUAD_EVIDENCE_AREA_RATIO_MIN"]
        <= poly_ratio
        <= params["QUAD_EVIDENCE_AREA_RATIO_MAX"]
    )
    if quadrilateral_evidence:
        if area < params["QUADRILATERAL_MIN_AREA"]:
            return None
        if solidity < params["POLYGON_MIN_SOLIDITY"]:
            return None
        if not contour_passes_geometry(contour, area, params):
            return None
        if not params["POLY_AREA_MIN_RATIO"] <= poly_ratio <= params["POLY_AREA_MAX_RATIO"]:
            return None
        return {
            "name": "Quadrilateral",
            "color": COLOR_RED,
            "draw": "poly",
            "approx": approx,
            "area": area,
        }

    far_ellipse = area < params["ELLIPSE_FAR_AREA_MAX"]
    detail_sides_key = (
        "ELLIPSE_FAR_MIN_DETAIL_SIDES" if far_ellipse else "ELLIPSE_MIN_DETAIL_SIDES"
    )
    min_tight_sides = params["ELLIPSE_FAR_MIN_TIGHT_SIDES"] if far_ellipse else 5
    solidity_key = "ELLIPSE_FAR_SOLIDITY_MIN" if far_ellipse else "ELLIPSE_SOLIDITY_MIN"
    ellipse_like = (
        detail_sides >= params[detail_sides_key]
        and tight_sides >= min_tight_sides
        and solidity >= params[solidity_key]
        and ellipse_score(hull, params, far_ellipse) > 0
    )
    if (
        ellipse_like
        and area >= params["ELLIPSE_MIN_AREA"]
        and contour_passes_geometry(
            contour,
            area,
            params,
            min_w=params["TRIANGLE_MIN_W"],
            min_h=params["TRIANGLE_MIN_H"],
            roi_edge_margin=params["TRIANGLE_ROI_EDGE_MARGIN"],
        )
    ):
        return {"name": "Ellipse", "color": COLOR_GREEN, "draw": "ellipse", "contour": hull, "area": area}
    if ellipse_like:
        return None

    if sides == 3:
        if tight_sides != 3:
            return None
        if area < params["TRIANGLE_MIN_AREA"] or solidity < params["TRIANGLE_MIN_SOLIDITY"]:
            return None
        if not contour_passes_geometry(
            contour,
            area,
            params,
            min_w=params["TRIANGLE_MIN_W"],
            min_h=params["TRIANGLE_MIN_H"],
            roi_edge_margin=params["TRIANGLE_ROI_EDGE_MARGIN"],
            max_aspect_ratio=params["TRIANGLE_MAX_ASPECT_RATIO"],
        ):
            return None
        x, y, w, h = cv2.boundingRect(contour)
        if area / max(1, w * h) > params["TRIANGLE_RECT_EXTENT_MAX"]:
            return None
        poly_area = cv2.contourArea(approx) if approx is not None else 0
        ratio = poly_area / area if area > 0 else 0
        if not params["TRI_AREA_MIN_RATIO"] <= ratio <= params["TRI_AREA_MAX_RATIO"]:
            return None
        return {"name": "Triangle", "color": COLOR_ORANGE, "draw": "poly", "approx": approx, "area": area}

    return None


def process_frame(frame, params):
    frame = cv2.resize(frame, (FRAME_W, FRAME_H), interpolation=cv2.INTER_AREA)
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    lower = np.array([params["L_MIN"], params["A_MIN"], params["B_MIN"]], dtype=np.uint8)
    upper = np.array([params["L_MAX"], params["A_MAX"], params["B_MAX"]], dtype=np.uint8)
    mask = cv2.inRange(lab, lower, upper)
    mask = clean_mask(apply_detect_roi(mask), params)

    detections = []
    for contour in find_contours(mask):
        detection = classify_shape(contour, params)
        if detection is not None:
            detections.append(detection)
    detections.sort(key=lambda item: item["area"], reverse=True)
    return frame, mask, detections


def draw_detection(frame, detection):
    color = detection["color"]
    if detection["draw"] == "poly":
        contour = detection["approx"]
        cv2.drawContours(frame, [contour], -1, color, 2)
        x, y, w, h = cv2.boundingRect(contour)
    else:
        contour = detection["contour"]
        cv2.ellipse(frame, cv2.fitEllipse(contour), color, 2)
        x, y, w, h = cv2.boundingRect(contour)
    label = f'{detection["name"]} A:{int(detection["area"])}'
    cv2.putText(frame, label, (x, max(14, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)


def draw_roi(frame):
    left, right, top, bottom = detect_roi_bounds()
    cv2.rectangle(frame, (left, top), (right - 1, bottom - 1), COLOR_ROI, 1)


def make_demo_frame(timestamp):
    frame = np.full((FRAME_H, FRAME_W, 3), 25, dtype=np.uint8)
    offset = int(5 * math.sin(timestamp * 0.8))
    cv2.fillPoly(frame, [np.array([[105, 155 + offset], [135, 95 + offset], [165, 155 + offset]], np.int32)], (235, 235, 235))
    cv2.rectangle(frame, (180, 95 - offset), (225, 145 - offset), (235, 235, 235), -1)
    cv2.ellipse(frame, (130, 195), (25, 15), 0, 0, 360, (235, 235, 235), -1)
    return frame


class VideoSource:
    def __init__(self):
        self.capture = None
        self.demo = False
        self.source_text = ""

    def close(self):
        if self.capture is not None:
            self.capture.release()
        self.capture = None
        self.demo = False

    def open(self, source_text):
        self.close()
        source_text = source_text.strip()
        if source_text.lower() == "auto":
            source_text = discover_maixcam()
        if source_text.lower() == "demo":
            self.demo = True
            self.source_text = "demo"
            return

        source = int(source_text) if source_text.isdigit() else source_text
        capture = cv2.VideoCapture(source)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"Unable to open video source: {source_text}")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.capture = capture
        self.source_text = source_text

    def read(self):
        if self.demo:
            return make_demo_frame(time.perf_counter())
        if self.capture is None:
            return None
        ok, frame = self.capture.read()
        if ok:
            return frame
        if isinstance(self.source_text, str) and Path(self.source_text).is_file():
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.capture.read()
            if ok:
                return frame
        return None


def discover_maixcam(timeout=DISCOVERY_TIMEOUT_SECONDS):
    """Wait for the UDP beacon emitted by maixcam_stream.py."""
    deadline = time.monotonic() + timeout
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", DISCOVERY_PORT))
        sock.settimeout(0.5)
        while time.monotonic() < deadline:
            try:
                payload, address = sock.recvfrom(256)
            except socket.timeout:
                continue
            parts = payload.decode("ascii", errors="ignore").strip().split()
            if len(parts) == 2 and parts[0] == DISCOVERY_MAGIC and parts[1].isdigit():
                return f"http://{address[0]}:{int(parts[1])}/stream.mjpg"
    finally:
        sock.close()
    raise RuntimeError(
        "MaixCAM was not found. Run maixcam_stream.py and connect both devices to the same network."
    )


class TunerApp:
    def __init__(self, root, source_text):
        self.root = root
        self.root.title("MaixCAM 形状识别调参")
        self.root.geometry("620x900")
        self.root.minsize(520, 600)
        self.settings_path = Path(__file__).with_name("maixcam_tuner_settings.json")
        self.export_path = Path(__file__).with_name("maixcam_tuned_parameters.py")
        self.variables = {}
        self.value_labels = {}
        self.source = VideoSource()
        self.last_name = "None"
        self.stable_frames = 0
        self.last_frame_time = time.perf_counter()
        self.fps = 0.0
        self.closed = False

        self._build_ui(source_text)
        self._load_settings(silent=True)
        self._connect_source(show_error=False)

        cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_NORMAL)
        cv2.namedWindow(MASK_WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(PREVIEW_WINDOW, 640, 480)
        cv2.resizeWindow(MASK_WINDOW, 640, 480)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(10, self.update)

    def _build_ui(self, source_text):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill=tk.X)
        ttk.Label(top, text="视频源").pack(side=tk.LEFT)
        self.source_var = tk.StringVar(value=source_text)
        ttk.Entry(top, textvariable=self.source_var).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        ttk.Button(top, text="连接", command=self._connect_source).pack(side=tk.LEFT)

        button_row = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        button_row.pack(fill=tk.X)
        ttk.Button(button_row, text="保存参数", command=self._save_settings).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(button_row, text="加载参数", command=self._load_settings).pack(side=tk.LEFT, padx=4)
        ttk.Button(button_row, text="恢复默认", command=self._reset_defaults).pack(side=tk.LEFT, padx=4)
        ttk.Button(button_row, text="导出 Python", command=self._export_python).pack(side=tk.LEFT, padx=4)
        ttk.Button(button_row, text="复制参数", command=self._copy_python).pack(side=tk.LEFT, padx=4)

        holder = ttk.Frame(self.root)
        holder.pack(fill=tk.BOTH, expand=True)
        canvas = tk.Canvas(holder, highlightthickness=0)
        scrollbar = ttk.Scrollbar(holder, orient=tk.VERTICAL, command=canvas.yview)
        self.slider_frame = ttk.Frame(canvas, padding=(8, 0, 8, 8))
        self.slider_frame.bind("<Configure>", lambda event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas_window = canvas.create_window((0, 0), window=self.slider_frame, anchor="nw")
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(canvas_window, width=event.width))
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.bind_all("<MouseWheel>", lambda event: canvas.yview_scroll(int(-event.delta / 120), "units"))

        for group_name, sliders in SLIDER_GROUPS:
            group = ttk.LabelFrame(self.slider_frame, text=group_name, padding=6)
            group.pack(fill=tk.X, pady=4)
            for key, label, default, minimum, maximum, resolution in sliders:
                self._add_slider(group, key, label, default, minimum, maximum, resolution)

        self.status_var = tk.StringVar(value="Waiting for frames")
        ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W, padding=5).pack(fill=tk.X)

    def _add_slider(self, parent, key, label, default, minimum, maximum, resolution):
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text=label, width=25).pack(side=tk.LEFT)
        variable = tk.DoubleVar(value=default)
        self.variables[key] = variable
        value_label = ttk.Label(row, width=8, anchor=tk.E)
        value_label.pack(side=tk.RIGHT)
        self.value_labels[key] = value_label

        scale = tk.Scale(
            row,
            variable=variable,
            from_=minimum,
            to=maximum,
            resolution=resolution,
            orient=tk.HORIZONTAL,
            showvalue=False,
            highlightthickness=0,
            command=lambda _value, item=key: self._update_value_label(item),
        )
        scale.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self._update_value_label(key)

    def _update_value_label(self, key):
        value = self.variables[key].get()
        self.value_labels[key].configure(text=str(int(round(value))) if key in INTEGER_KEYS else f"{value:.3f}")

    def get_params(self):
        params = {}
        for key, variable in self.variables.items():
            value = variable.get()
            params[key] = int(round(value)) if key in INTEGER_KEYS else float(value)

        for minimum_key, maximum_key in (
            ("L_MIN", "L_MAX"),
            ("A_MIN", "A_MAX"),
            ("B_MIN", "B_MAX"),
            ("POLY_AREA_MIN_RATIO", "POLY_AREA_MAX_RATIO"),
            ("QUAD_EVIDENCE_AREA_RATIO_MIN", "QUAD_EVIDENCE_AREA_RATIO_MAX"),
            ("TRI_AREA_MIN_RATIO", "TRI_AREA_MAX_RATIO"),
            ("ELLIPSE_FILL_MIN", "ELLIPSE_FILL_MAX"),
            ("ELLIPSE_FAR_FILL_MIN", "ELLIPSE_FAR_FILL_MAX"),
        ):
            if params[minimum_key] > params[maximum_key]:
                params[maximum_key] = params[minimum_key]
        return params

    def _connect_source(self, show_error=True):
        try:
            self.source.open(self.source_var.get())
            self.status_var.set(f"Connected: {self.source.source_text}")
        except Exception as exc:
            self.status_var.set(str(exc))
            if show_error:
                messagebox.showerror("Video source", str(exc))

    def _save_settings(self):
        payload = {"source": self.source_var.get(), "parameters": self.get_params()}
        self.settings_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        self.status_var.set(f"Saved: {self.settings_path.name}")

    def _load_settings(self, silent=False):
        if not self.settings_path.exists():
            if not silent:
                messagebox.showinfo("Load", "No saved settings file exists yet.")
            return
        try:
            payload = json.loads(self.settings_path.read_text(encoding="utf-8"))
            if not silent:
                self.source_var.set(str(payload.get("source", self.source_var.get())))
            for key, value in payload.get("parameters", {}).items():
                if key in self.variables:
                    self.variables[key].set(value)
                    self._update_value_label(key)
            self.status_var.set(f"Loaded: {self.settings_path.name}")
        except Exception as exc:
            if not silent:
                messagebox.showerror("Load", str(exc))

    def _reset_defaults(self):
        for key, value in DEFAULTS.items():
            self.variables[key].set(value)
            self._update_value_label(key)
        self.status_var.set("Default parameters restored")

    def _python_parameter_text(self):
        params = self.get_params()
        lines = [
            "# Generated by maixcam_tuner.py",
            f'L_MIN, L_MAX = {params["L_MIN"]}, {params["L_MAX"]}',
            f'A_MIN, A_MAX = {params["A_MIN"]}, {params["A_MAX"]}',
            f'B_MIN, B_MAX = {params["B_MIN"]}, {params["B_MAX"]}',
            "",
        ]
        for key in (
            "CANDIDATE_MIN_AREA",
            "TRIANGLE_MIN_AREA",
            "QUADRILATERAL_MIN_AREA",
            "ELLIPSE_MIN_AREA",
        ):
            lines.append(f"{key} = {params[key]}")
        lines.append(f'MAX_AREA = FRAME_W * FRAME_H * {params["MAX_AREA_RATIO"]:.3f}')
        for key in (
            "MIN_W",
            "MIN_H",
            "TRIANGLE_MIN_W",
            "TRIANGLE_MIN_H",
            "EDGE_MARGIN",
            "ROI_EDGE_MARGIN",
            "TRIANGLE_ROI_EDGE_MARGIN",
            "MAX_ASPECT_RATIO",
            "TRIANGLE_MAX_ASPECT_RATIO",
            "POLY_AREA_MIN_RATIO",
            "POLY_AREA_MAX_RATIO",
            "TRI_AREA_MIN_RATIO",
            "TRI_AREA_MAX_RATIO",
            "TRIANGLE_MIN_SOLIDITY",
            "TRIANGLE_RECT_EXTENT_MAX",
            "POLYGON_MIN_SOLIDITY",
            "MEDIAN_K",
            "MORPH_K",
            "APPROX_EPS_DETAIL",
            "APPROX_EPS",
            "APPROX_EPS_STRONG",
            "ELLIPSE_MIN_DETAIL_SIDES",
            "ELLIPSE_FILL_MIN",
            "ELLIPSE_FILL_MAX",
            "ELLIPSE_RECT_EXTENT_MAX",
            "ELLIPSE_AXIS_RATIO_MAX",
            "ELLIPSE_CIRCULARITY_MIN",
            "ELLIPSE_SOLIDITY_MIN",
            "STABLE_FRAMES_REQUIRED",
        ):
            value = params[key]
            rendered = str(value) if key in INTEGER_KEYS else f"{value:.3f}"
            lines.append(f"{key} = {rendered}")
        return "\n".join(lines) + "\n"

    def _export_python(self):
        self.export_path.write_text(self._python_parameter_text(), encoding="utf-8")
        self.status_var.set(f"Exported: {self.export_path.name}")

    def _copy_python(self):
        self.root.clipboard_clear()
        self.root.clipboard_append(self._python_parameter_text())
        self.status_var.set("Python parameters copied")

    def update(self):
        if self.closed:
            return
        frame = self.source.read()
        if frame is not None:
            params = self.get_params()
            frame, mask, detections = process_frame(frame, params)
            preview = frame.copy()
            for detection in detections:
                draw_detection(preview, detection)

            current_name = detections[0]["name"] if detections else "None"
            if current_name != "None" and current_name == self.last_name:
                self.stable_frames += 1
            elif current_name != "None":
                self.last_name = current_name
                self.stable_frames = 1
            else:
                self.last_name = "None"
                self.stable_frames = 0

            now = time.perf_counter()
            elapsed = now - self.last_frame_time
            self.last_frame_time = now
            if elapsed > 0:
                instant_fps = 1.0 / elapsed
                self.fps = instant_fps if self.fps == 0 else self.fps * 0.9 + instant_fps * 0.1

            draw_roi(preview)
            stable_required = (
                params["ELLIPSE_STABLE_FRAMES_REQUIRED"]
                if current_name == "Ellipse"
                else params["STABLE_FRAMES_REQUIRED"]
            )
            stable_text = "READY" if self.stable_frames >= stable_required else f"{self.stable_frames}/{stable_required}"
            status = f"{current_name}  {stable_text}  {self.fps:.1f} FPS"
            cv2.putText(preview, status, (5, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_BLACK, 2, cv2.LINE_AA)
            cv2.putText(preview, status, (5, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_WHITE, 1, cv2.LINE_AA)
            self.status_var.set(f"Source: {self.source.source_text} | {status} | Detections: {len(detections)}")

            cv2.imshow(PREVIEW_WINDOW, preview)
            cv2.imshow(MASK_WINDOW, mask)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            self.close()
            return
        self.root.after(10, self.update)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.source.close()
        cv2.destroyAllWindows()
        self.root.destroy()


def parse_args():
    parser = argparse.ArgumentParser(description="Tune MaixCAM shape detection parameters on a PC.")
    parser.add_argument(
        "--source",
        default="auto",
        help="Source: 'auto', camera index, video file, RTSP/HTTP URL, or 'demo'.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    root = tk.Tk()
    TunerApp(root, args.source)
    root.mainloop()


if __name__ == "__main__":
    main()
