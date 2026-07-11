# pyright: reportMissingImports=false
"""
MaixCAM shape detection + UART communication.

- Detect triangle, quadrilateral, and ellipse.
- Only detect inside the center half width and lower two-thirds height ROI.
- Send the detected shape to Arduino over UART2 after 20 stable frames.
"""

from maix import app, camera, display, image, uart, pinmap, time as mtime
import cv2
import numpy as np

# UART2: A28=TX, A29=RX
pinmap.set_pin_function("A28", "UART2_TX")
pinmap.set_pin_function("A29", "UART2_RX")
serial = uart.UART("/dev/ttyS2", 9600)

FRAME_W = 320
FRAME_H = 240

# LAB threshold. Keep the upper L wide open so bright white targets are not clipped.
L_MIN, L_MAX = 185, 255
A_MIN, A_MAX = 98, 222
B_MIN, B_MAX = 86, 166

# Geometry filter
CANDIDATE_MIN_AREA = 80
DEFAULT_MIN_AREA = 400
TRIANGLE_MIN_AREA = 80
QUADRILATERAL_MIN_AREA = 400
ELLIPSE_MIN_AREA = 120
MAX_AREA = FRAME_W * FRAME_H * 0.85
MIN_W = 15
MIN_H = 15
TRIANGLE_MIN_W = 7
TRIANGLE_MIN_H = 7
EDGE_MARGIN = 6
ROI_EDGE_MARGIN = 8
TRIANGLE_ROI_EDGE_MARGIN = 2
MAX_ASPECT_RATIO = 3.2
TRIANGLE_MAX_ASPECT_RATIO = 4.0
POLY_AREA_MIN_RATIO = 0.72
POLY_AREA_MAX_RATIO = 1.18
TRI_AREA_MIN_RATIO = 0.40
TRI_AREA_MAX_RATIO = 1.50
TRIANGLE_MIN_SOLIDITY = 0.68
TRIANGLE_RECT_EXTENT_MAX = 0.76
POLYGON_MIN_SOLIDITY = 0.82

# Detect only inside the center half width and almost the full height.
ROI_LEFT_RATIO = 0.25
ROI_RIGHT_RATIO = 0.75
ROI_TOP_RATIO = 0.10
ROI_BOTTOM_RATIO = 1.0

MEDIAN_K = 3
MORPH_K = 3
APPROX_EPS_DETAIL = 0.012
APPROX_EPS = 0.035
APPROX_EPS_STRONG = 0.055

# Stricter ellipse checks to reduce false positives.
ELLIPSE_MIN_DETAIL_SIDES = 5
ELLIPSE_FILL_MIN = 0.78
ELLIPSE_FILL_MAX = 1.12
ELLIPSE_RECT_EXTENT_MAX = 0.84
ELLIPSE_AXIS_RATIO_MAX = 2.20
ELLIPSE_CIRCULARITY_MIN = 0.72
ELLIPSE_SOLIDITY_MIN = 0.90

COLOR_RED = (255, 0, 0)
COLOR_ORANGE = (255, 165, 0)
COLOR_GREEN = (0, 255, 0)
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)

STABLE_FRAMES_REQUIRED = 20
SEND_INTERVAL_MS = 500


def find_contours(mask):
    result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return result[0] if len(result) == 2 else result[1]


def clean_mask(mask, kernel):
    mask = cv2.medianBlur(mask, MEDIAN_K)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_roi_bounds():
    left = int(FRAME_W * ROI_LEFT_RATIO)
    right = int(FRAME_W * ROI_RIGHT_RATIO)
    top = int(FRAME_H * ROI_TOP_RATIO)
    bottom = int(FRAME_H * ROI_BOTTOM_RATIO)
    return left, right, top, bottom


def apply_detect_roi(mask):
    left, right, top, bottom = detect_roi_bounds()
    mask[:top, :] = 0
    if bottom < FRAME_H:
        mask[bottom:, :] = 0
    mask[:, :left] = 0
    mask[:, right:] = 0
    return mask


def approx_hull(contour, eps_ratio):
    hull = cv2.convexHull(contour)
    peri = cv2.arcLength(hull, True)
    if peri <= 0:
        return hull, peri
    approx = cv2.approxPolyDP(hull, eps_ratio * peri, True)
    return approx, peri


def contour_center(contour):
    moments = cv2.moments(contour)
    if moments["m00"] != 0:
        return int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"])
    x, y, w, h = cv2.boundingRect(contour)
    return x + w // 2, y + h // 2


def center_in_detect_roi(contour):
    cx, cy = contour_center(contour)
    left, right, top, bottom = detect_roi_bounds()
    return left <= cx <= right and top <= cy <= bottom


def contour_passes_geometry(
    contour,
    area,
    min_w=MIN_W,
    min_h=MIN_H,
    roi_edge_margin=ROI_EDGE_MARGIN,
    max_aspect_ratio=MAX_ASPECT_RATIO,
):
    x, y, w, h = cv2.boundingRect(contour)
    left, right, top, bottom = detect_roi_bounds()

    if w < min_w or h < min_h:
        return False
    if x <= EDGE_MARGIN or y <= EDGE_MARGIN:
        return False
    if x + w >= FRAME_W - EDGE_MARGIN or y + h >= FRAME_H - EDGE_MARGIN:
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

    rect_area = w * h
    if rect_area <= 0:
        return False
    if area / rect_area < 0.22:
        return False
    return True


def best_polygon_approx(contour):
    candidates = []
    for eps in (0.025, APPROX_EPS, APPROX_EPS_STRONG, 0.075, 0.10):
        approx, _ = approx_hull(contour, eps)
        candidates.append((eps, len(approx), approx))

    first_three = None
    first_four = None
    for eps, sides, approx in candidates:
        if sides == 3 and first_three is None:
            first_three = (eps, approx)
        if sides == 4 and first_four is None:
            first_four = (eps, approx)

    # If only the loosest approximation turns a clear four-corner contour into
    # three points, keep it as a quadrilateral instead of forcing a triangle.
    if (
        first_four is not None
        and first_three is not None
        and cv2.contourArea(contour) >= QUADRILATERAL_MIN_AREA
        and first_four[0] <= APPROX_EPS_STRONG
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


def ellipse_score(contour):
    if len(contour) < 5:
        return 0

    area = cv2.contourArea(contour)
    ellipse = cv2.fitEllipse(contour)
    (_, _), (axis_a, axis_b), _ = ellipse
    if axis_a <= 0 or axis_b <= 0:
        return 0

    major_axis = max(axis_a, axis_b)
    minor_axis = min(axis_a, axis_b)
    if minor_axis <= 0:
        return 0
    if major_axis / minor_axis > ELLIPSE_AXIS_RATIO_MAX:
        return 0

    ellipse_area = np.pi * (axis_a * 0.5) * (axis_b * 0.5)
    if ellipse_area <= 0:
        return 0
    fill_ratio = area / ellipse_area
    if fill_ratio < ELLIPSE_FILL_MIN or fill_ratio > ELLIPSE_FILL_MAX:
        return 0

    peri = cv2.arcLength(contour, True)
    if peri <= 0:
        return 0
    circularity = 4.0 * np.pi * area / (peri * peri)
    if circularity < ELLIPSE_CIRCULARITY_MIN:
        return 0

    rect = cv2.minAreaRect(contour)
    rect_w, rect_h = rect[1]
    rect_area = rect_w * rect_h
    if rect_area <= 0:
        return 0
    rect_extent = area / rect_area
    if rect_extent > ELLIPSE_RECT_EXTENT_MAX:
        return 0

    return fill_ratio * circularity


def classify_shape(contour):
    area = cv2.contourArea(contour)
    if area < CANDIDATE_MIN_AREA or area > MAX_AREA:
        return None
    if not center_in_detect_roi(contour):
        return None

    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    if hull_area <= 0:
        return None

    solidity = area / hull_area

    approx_detail, _ = approx_hull(contour, APPROX_EPS_DETAIL)
    detail_sides = len(approx_detail)
    ellipse_fit_score = ellipse_score(hull)
    ellipse_like = (
        detail_sides >= ELLIPSE_MIN_DETAIL_SIDES
        and solidity >= ELLIPSE_SOLIDITY_MIN
        and ellipse_fit_score > 0
    )
    if (
        ellipse_like
        and area >= ELLIPSE_MIN_AREA
        and contour_passes_geometry(
            contour,
            area,
            min_w=TRIANGLE_MIN_W,
            min_h=TRIANGLE_MIN_H,
            roi_edge_margin=TRIANGLE_ROI_EDGE_MARGIN,
        )
    ):
        return {
            "name": "Ellipse",
            "side_text": "ellipse",
            "color": COLOR_GREEN,
            "draw": "ellipse",
            "contour": hull,
            "area": area,
        }
    if ellipse_like:
        return None

    approx, sides = best_polygon_approx(contour)
    if sides == 3:
        if area < TRIANGLE_MIN_AREA:
            return None
        if solidity < TRIANGLE_MIN_SOLIDITY:
            return None
        if not contour_passes_geometry(
            contour,
            area,
            min_w=TRIANGLE_MIN_W,
            min_h=TRIANGLE_MIN_H,
            roi_edge_margin=TRIANGLE_ROI_EDGE_MARGIN,
            max_aspect_ratio=TRIANGLE_MAX_ASPECT_RATIO,
        ):
            return None
        x, y, w, h = cv2.boundingRect(contour)
        rect_extent = area / max(1, w * h)
        if rect_extent > TRIANGLE_RECT_EXTENT_MAX:
            return None
        poly_area = cv2.contourArea(approx) if approx is not None else 0
        ratio = poly_area / area if area > 0 else 0
        if ratio < TRI_AREA_MIN_RATIO or ratio > TRI_AREA_MAX_RATIO:
            return None
        return {
            "name": "Triangle",
            "side_text": "3 sides",
            "color": COLOR_ORANGE,
            "draw": "poly",
            "approx": approx,
            "area": area,
        }

    if sides == 4:
        if area < QUADRILATERAL_MIN_AREA:
            return None
        if solidity < POLYGON_MIN_SOLIDITY:
            return None
        if not contour_passes_geometry(contour, area):
            return None
        poly_area = cv2.contourArea(approx) if approx is not None else 0
        ratio = poly_area / area if area > 0 else 0
        if ratio < POLY_AREA_MIN_RATIO or ratio > POLY_AREA_MAX_RATIO:
            return None
        return {
            "name": "Quadrilateral",
            "side_text": "4 sides",
            "color": COLOR_RED,
            "draw": "poly",
            "approx": approx,
            "area": area,
        }

    return None


def draw_detection(out, det):
    color = det["color"]
    if det["draw"] == "poly":
        cv2.drawContours(out, [det["approx"]], -1, color, 2)
        x, y, w, h = cv2.boundingRect(det["approx"])
    else:
        ellipse = cv2.fitEllipse(det["contour"])
        cv2.ellipse(out, ellipse, color, 2)
        x, y, w, h = cv2.boundingRect(det["contour"])

    cv2.putText(
        out,
        det["side_text"],
        (x, max(14, y - 5)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        color,
        1,
        cv2.LINE_AA,
    )


def draw_status(out, name, frame_count):
    text = "{} F:{}".format(name, frame_count)
    cv2.putText(out, text, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_BLACK, 2)
    cv2.putText(out, text, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_WHITE, 1)


def draw_roi(out):
    left, right, top, bottom = detect_roi_bounds()
    cv2.rectangle(out, (left, top), (right - 1, bottom - 1), (255, 255, 0), 1)


def main():
    cam = camera.Camera(FRAME_W, FRAME_H, image.Format.FMT_RGB888)
    disp = display.Display()
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MORPH_K, MORPH_K))

    last_name = "None"
    stable_frames = 0
    last_send_ms = 0

    while not app.need_exit():
        src = cam.read()
        rgb = image.image2cv(src, ensure_bgr=False, copy=True)

        bgr_for_lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        lab = cv2.cvtColor(bgr_for_lab, cv2.COLOR_BGR2LAB)
        mask = cv2.inRange(
            lab,
            np.array([L_MIN, A_MIN, B_MIN], dtype=np.uint8),
            np.array([L_MAX, A_MAX, B_MAX], dtype=np.uint8),
        )
        mask = apply_detect_roi(mask)
        mask = clean_mask(mask, kernel)

        detections = []
        for contour in find_contours(mask):
            det = classify_shape(contour)
            if det is not None:
                detections.append(det)
        detections.sort(key=lambda det: det["area"], reverse=True)

        out = rgb.copy()
        for det in detections:
            draw_detection(out, det)

        if detections:
            current_name = detections[0]["name"]
            if current_name == last_name:
                stable_frames += 1
            else:
                last_name = current_name
                stable_frames = 1
        else:
            current_name = "None"
            last_name = "None"
            stable_frames = 0

        now_ms = mtime.time_ms()
        if (
            stable_frames >= STABLE_FRAMES_REQUIRED
            and current_name != "None"
            and now_ms - last_send_ms >= SEND_INTERVAL_MS
        ):
            shape_map = {
                "Triangle": b"triangle\n",
                "Quadrilateral": b"quadrilateral\n",
                "Ellipse": b"ellipse\n",
            }
            msg = shape_map.get(current_name)
            if msg:
                serial.write(msg)
                last_send_ms = now_ms

        draw_roi(out)
        draw_status(out, current_name, stable_frames)
        disp.show(image.cv2image(out, bgr=False, copy=False))


if __name__ == "__main__":
    main()
