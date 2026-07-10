#!/usr/bin/env python3
"""
Workpiece Inspection Node  v3.0
================================
Lean ROS 2 edge node — no GUI framework.
  • Timer-driven ROS 2 execution (rclpy.spin + create_timer)
  • Shadow Slicer CV math preserved from v2
  • Z_CORRECTED = 0.347 m used for ALL spatial math and publishing
  • Moondream AI with 2 focused prompts + dual confidence scoring
  • Two-tool state machine: gripper / drill / reject
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
from sabry_hardware.srv import ChangeTool
from ament_index_python.packages import get_package_share_directory

import os
import sys
import cv2
import numpy as np
import re
import math
import time
import queue
import threading
from dataclasses import dataclass
import ollama
from ultralytics import YOLO


# ═══════════════════════════════════════════════════════════════════
#  PHYSICAL CONSTANTS
# ═══════════════════════════════════════════════════════════════════

# Camera-to-table = 0.350 m.  Workpiece is 3 mm thick, so the top
# face (where the gripper contacts) sits at 0.347 m from the lens.
# This value is used for EVERY pinhole projection — both X/Y and Z.
Z_CORRECTED = 0.350 - 0.003   # 0.347 m

CAMERA_MATRIX = np.array([
    [542.82128914,   0.0,         317.53314771],
    [  0.0,         542.69877113, 236.44041937],
    [  0.0,           0.0,           1.0      ],
], dtype=np.float32)

DIST_COEFFS = np.array(
    [-0.121078291, 0.191025852, 0.000983879143, -0.000131465926, -0.0698813042],
    dtype=np.float32,
)

YOLO_CONF   = 0.85   # detection confidence threshold
MIN_HOLE_PX = 10     # minimum contour area (px²) to classify as a real hole

AI_FAILURE_MARKER = "AI_INFERENCE_FAILED"
AI_MAX_RETRIES    = 3


# ═══════════════════════════════════════════════════════════════════
#  CONFIDENCE SCORING
# ═══════════════════════════════════════════════════════════════════

def score_cv_confidence(hole_contours: list) -> float:
    """
    Returns 0.0–1.0 based on average hole circularity.

    Circularity = 4π·A / P²  (1.0 = perfect circle, 0.0 = line).
    The more circular the detected holes, the more trustworthy the CV count.

    Thresholds:
      avg ≥ 0.75  →  95%   (clean, round holes — high trust)
      avg  0.50–0.75 →  60–95% (linear ramp)
      avg < 0.50  →  40%   (jagged / shadow blobs — low trust)
    """
    if not hole_contours:
        return 0.50   # neutral: "zero holes" is a valid, confident answer

    circularities = []
    for cnt in hole_contours:
        area  = cv2.contourArea(cnt)
        perim = cv2.arcLength(cnt, True)
        if perim > 0.0:
            circularities.append(min((4.0 * math.pi * area) / (perim ** 2), 1.0))

    if not circularities:
        return 0.40

    avg = sum(circularities) / len(circularities)

    if avg >= 0.75:
        return 0.95
    elif avg >= 0.50:
        return 0.60 + (avg - 0.50) / 0.25 * 0.35   # linear 60→95%
    return 0.40


def score_ai_confidence(raw_text: str, fallback_used: bool) -> float:
    """
    Returns 0.0–1.0 based on how decisively the AI stated a number.

      Direct digit, short response  →  92%  (e.g. the model said "4")
      Direct digit, longer response →  85%  (e.g. "There are 4 holes.")
      Word-to-number fallback used  →  55%  (e.g. "four holes visible")
      Empty / inference error        →  10%
    """
    text = raw_text.strip()
    if not text or text == AI_FAILURE_MARKER:
        return 0.10
    if not fallback_used:
        return 0.92 if len(text) <= 5 else 0.85
    return 0.55


# ═══════════════════════════════════════════════════════════════════
#  DETECTION SNAPSHOT
# ═══════════════════════════════════════════════════════════════════

@dataclass
class DetectionSnapshot:
    """One immutable, fully-formed detection result handed from the
    timer thread to the 'c' handler via a single-slot Queue."""
    frame:    np.ndarray
    bbox:     tuple
    center:   tuple
    cdata:    dict
    coords:   tuple   # (X, Y, Z)
    cls_name: str


# ═══════════════════════════════════════════════════════════════════
#  MAIN NODE
# ═══════════════════════════════════════════════════════════════════

class WorkpieceInspectorNode(Node):

    def __init__(self):
        super().__init__('workpiece_inspector_node')

        # ── ROS 2 publishers ───────────────────────────────────────
        self.coord_pub  = self.create_publisher(Point, 'workpiece_coordinates', 10)
        self.target_holes_pub = self.create_publisher(Int32, 'target_holes_count', 10)
        self.actual_holes_pub = self.create_publisher(Int32, 'actual_holes_count', 10)

        # ── ATC tool-changer service client ───────────────────────
        # Matches the live service server in tool_change_manager.py
        # (ChangeTool.srv: string tool_name -> bool success, string message)
        #
        # The client sits in its own MutuallyExclusiveCallbackGroup so its
        # response callback is never queued behind the 30 Hz timer
        # callback (which lives in the node's default callback group).
        # The node is spun by a MultiThreadedExecutor (see main()), which
        # actually gives each group its own worker thread to run on.
        self._atc_cb_group = MutuallyExclusiveCallbackGroup()
        self.tool_client = self.create_client(
            ChangeTool, 'change_tool', callback_group=self._atc_cb_group)
        if not self.tool_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "ATC service 'change_tool' not available at startup — "
                "will retry per-call.")

        # ── YOLO (circuit breaker #1) ────────────────────────────────
        # A broken model load must not leave the node running with
        # self.yolo unset — that would surface as a confusing crash on
        # the first detected frame instead of an immediate, clear cause.
        pkg_path   = get_package_share_directory('sabry_hardware')
        model_path = os.path.join(pkg_path, 'models', 'best_ver3.pt')
        try:
            self.yolo = YOLO(model_path)
        except Exception as e:
            self.get_logger().fatal(
                f"YOLO model failed to load from '{model_path}': {e}. "
                f"Refusing to start in a broken state.")
            self.destroy_node()
            raise SystemExit(1)
        self.get_logger().info(f"YOLO loaded  →  {model_path}")

        # ── Camera (circuit breaker #2) ───────────────────────────────
        # Try indices 0..2 in order — whichever one actually opens first
        # is the one we use. /dev/videoN assignment shifts around
        # depending on USB enumeration order, so a single hardcoded
        # index is brittle.
        self.cap = None
        opened_index = None
        for idx in range(3):
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                self.cap = cap
                opened_index = idx
                break
            cap.release()
        if self.cap is None:
            self.get_logger().fatal(
                "No camera found on indices 0-2. "
                "Refusing to start in a broken state.")
            self.destroy_node()
            raise SystemExit(1)
        self.get_logger().info(f"Camera opened on index {opened_index}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS,           30)

        # Cached undistortion maps — computed once on the first frame
        self._new_cam_mx = None
        self._roi        = None

        # ── Task state ─────────────────────────────────────────────
        self.is_analyzing    = False
        self.current_task_id = 0
        self._hud_line       = "Ready  |  'c' = Analyze  |  'q' = Quit"

        # Latest detection snapshot — written by the timer thread, popped
        # atomically by the 'c' handler. maxsize=1 means "only the most
        # recent detection matters"; the timer always clears any stale
        # item before pushing a new one so the 'c' handler can never see
        # a half-updated snapshot (no more torn reads across 5 fields).
        self._snap_queue = queue.Queue(maxsize=1)

        # Guards self.current_task_id against the cross-thread cancellation
        # race: a 'c' press increments it on the timer thread while
        # in-flight analysis threads read it to detect staleness.
        self._task_lock = threading.Lock()

        # GUI-based target-hole confirmation — the daemon analysis thread
        # blocks on this event instead of input(), since input() would
        # contend with cv2.waitKey() for the terminal/keyboard.
        self._input_event  = threading.Event()
        self._target_holes = None

        self.get_logger().info(
            "Workpiece Inspector ready.  "
            f"Camera feed: index {opened_index}  |  Publishing: /workpiece_coordinates"
        )
        self.timer = self.create_timer(0.033, self.process_frame)

    # ── Undistort (cached) ─────────────────────────────────────────
    def _undistort(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        if self._new_cam_mx is None:
            self._new_cam_mx, self._roi = cv2.getOptimalNewCameraMatrix(
                CAMERA_MATRIX, DIST_COEFFS, (w, h), 1, (w, h))
        out = cv2.undistort(frame, CAMERA_MATRIX, DIST_COEFFS, None, self._new_cam_mx)
        x, y, wr, hr = self._roi
        if wr > 0 and hr > 0:
            out = out[y:y + hr, x:x + wr]
        return out

    # ── YOLO detection ─────────────────────────────────────────────
    def _detect(self, frame: np.ndarray):
        """Returns ((x1,y1,x2,y2), cls_name) of the largest confident box, or (None, None)."""
        largest, max_a, cls_name = None, 0, None
        for r in self.yolo(frame, stream=True, conf=YOLO_CONF, verbose=False):
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                a = (x2 - x1) * (y2 - y1)
                if a > max_a:
                    max_a, largest = a, (x1, y1, x2, y2)
                    cls_name = r.names[int(box.cls[0])]
        return largest, cls_name

    # ── Shadow Slicer contour analysis ─────────────────────────────
    def _analyze_contours(self, frame: np.ndarray, bbox: tuple) -> dict:
        """
        Applies Otsu×0.65 threshold (Shadow Slicer) to the bounding-box ROI
        then uses RETR_CCOMP hierarchy to separate the outer workpiece contour
        from interior holes.

        Area scale is computed using Z_CORRECTED so the mm² output corresponds
        to the real surface height of the workpiece, not the table.
        """
        x1, y1, x2, y2 = bbox
        roi     = frame[y1:y2, x1:x2]
        gray    = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # ── Shadow Slicer: compute Otsu baseline then clip to pitch-black ──
        otsu_val, _ = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        shadow_val  = otsu_val * 0.65
        _, thresh   = cv2.threshold(
            blurred, shadow_val, 255, cv2.THRESH_BINARY_INV)

        # ── RETR_CCOMP: two-level hierarchy ────────────────────────
        #    Level 0 (parent index == -1)  →  outer contour of the part
        #    Level 1 (has a parent)        →  holes inside the part
        cnts, hier = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        outer_a, outer_c = 0.0, None
        hole_a,  hole_n  = 0.0, 0
        hole_cs          = []

        if cnts and hier is not None:
            h = hier[0]
            for i, c in enumerate(cnts):
                a = cv2.contourArea(c)
                if h[i][3] == -1:       # no parent → outer boundary
                    if a > outer_a:
                        outer_a, outer_c = a, c
                else:                   # has parent → hole
                    perim = cv2.arcLength(c, True)
                    circ  = (4.0 * math.pi * a) / (perim ** 2) if perim > 0.0 else 0.0
                    if a > MIN_HOLE_PX and perim > 0.0 and circ > 0.55:
                        hole_a += a
                        hole_n += 1
                        hole_cs.append(c)

        # ── Physical area at Z_CORRECTED ───────────────────────────
        fx, fy   = CAMERA_MATRIX[0, 0], CAMERA_MATRIX[1, 1]
        mm_per_px_x = Z_CORRECTED * 1000.0 / fx   # mm per pixel (horizontal)
        mm_per_px_y = Z_CORRECTED * 1000.0 / fy   # mm per pixel (vertical)
        scale_mm2   = mm_per_px_x * mm_per_px_y
        area_mm2    = (outer_a - hole_a) * scale_mm2

        return {
            'hole_count':    hole_n,
            'hole_contours': hole_cs,
            'outer_contour': outer_c,
            'area_mm2':      area_mm2,
        }

    # ── Pinhole inverse projection ──────────────────────────────────
    def _pixel_to_world(self, u: int, v: int) -> tuple:
        """
        Projects pixel (u, v) to world coordinates (X, Y, Z).
        Uses Z_CORRECTED (0.347 m) for all three axes so X and Y are
        referenced to the actual workpiece top surface, not the table.

              X = (u − cx) · Z / fx
              Y = (v − cy) · Z / fy
              Z = Z_CORRECTED
        """
        fx = CAMERA_MATRIX[0, 0]
        fy = CAMERA_MATRIX[1, 1]
        cx = CAMERA_MATRIX[0, 2]
        cy = CAMERA_MATRIX[1, 2]
        return ((u - cx) * Z_CORRECTED / fx,
                (v - cy) * Z_CORRECTED / fy,
                Z_CORRECTED)

    # ── Frame annotation ───────────────────────────────────────────
    def _annotate(self, frame: np.ndarray, bbox: tuple,
                  cdata: dict, X: float, Y: float) -> np.ndarray:
        out = frame.copy()
        h, w = out.shape[:2]
        x1, y1, x2, y2 = bbox
        cx_cam = int(CAMERA_MATRIX[0, 2])
        cy_cam = int(CAMERA_MATRIX[1, 2])
        cx_obj = (x1 + x2) // 2
        cy_obj = (y1 + y2) // 2
        fx, fy = CAMERA_MATRIX[0, 0], CAMERA_MATRIX[1, 1]

        # Numbered metric coordinate plane (cm ticks at Z_CORRECTED)
        cv2.line(out, (cx_cam, 0), (cx_cam, h), (40, 40, 40), 1)
        cv2.line(out, (0, cy_cam), (w, cy_cam), (40, 40, 40), 1)
        cv2.circle(out, (cx_cam, cy_cam), 4, (0, 200, 0), -1)

        px50x = int((0.05 * fx) / Z_CORRECTED)   # px per 5 cm, horizontal
        px50y = int((0.05 * fy) / Z_CORRECTED)   # px per 5 cm, vertical
        font  = cv2.FONT_HERSHEY_SIMPLEX

        for i in range(1, 7):
            cm_label = i * 5

            # ── X axis ticks (left/right of optical center) ──────
            for s, label in ((1, f"{cm_label}cm"), (-1, f"-{cm_label}cm")):
                ox = s * i * px50x
                tx = cx_cam + ox
                if 0 < tx < w:
                    cv2.line(out, (tx, cy_cam - 5), (tx, cy_cam + 5), (0, 160, 0), 1)
                    cv2.putText(out, label, (tx - 14, cy_cam + 18),
                                font, 0.4, (0, 200, 0), 1)

            # ── Y axis ticks (above/below optical center) ────────
            for s, label in ((1, f"{cm_label}cm"), (-1, f"-{cm_label}cm")):
                oy = s * i * px50y
                ty = cy_cam + oy
                if 0 < ty < h:
                    cv2.line(out, (cx_cam - 5, ty), (cx_cam + 5, ty), (0, 160, 0), 1)
                    cv2.putText(out, label, (cx_cam + 8, ty + 4),
                                font, 0.4, (0, 200, 0), 1)

        # Contour overlays inside the bounding box sub-image
        sub = out[y1:y2, x1:x2]
        if cdata.get('outer_contour') is not None:
            cv2.drawContours(sub, [cdata['outer_contour']], -1, (0, 255, 0), 2)
        for hc in cdata.get('hole_contours', []):
            cv2.drawContours(sub, [hc], -1, (0, 0, 255), 1)

        # Bounding box + line from principal point to object centre
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 255), 3)
        cv2.line(out, (cx_cam, cy_cam), (cx_obj, cy_obj), (0, 255, 255), 1)
        cv2.circle(out, (cx_obj, cy_obj), 5, (0, 0, 255), -1)

        # Bottom HUD strip
        cv2.rectangle(out, (0, h - 55), (w, h), (0, 0, 0), -1)
        cv2.putText(out,
                    f"X:{X*1000:.1f}mm  Y:{Y*1000:.1f}mm  Z:{Z_CORRECTED*1000:.0f}mm",
                    (10, h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 0), 1)
        cv2.putText(out,
                    f"Area: {cdata.get('area_mm2', 0.0):.1f} mm²   "
                    f"Holes: {cdata.get('hole_count', 0)}",
                    (10, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 150, 0), 1)

        # Top status bar
        bar_col = (0, 180, 255) if self.is_analyzing else (0, 255, 100)
        cv2.rectangle(out, (0, 0), (w, 26), (0, 0, 0), -1)
        cv2.putText(out, self._hud_line, (8, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, bar_col, 1)

        return out

    # ── ATC tool-changer service call ───────────────────────────────
    def _call_tool_changer(self, tool: str, timeout_sec: float = 5.0) -> bool:
        """
        Calls the live 'change_tool' Service (ChangeTool.srv) to physically
        switch the ATC head. Runs from the daemon analysis thread.

        The node is spun by a MultiThreadedExecutor, and self.tool_client
        lives in its own MutuallyExclusiveCallbackGroup, so the response
        callback that completes `future` is dispatched on a dedicated
        executor worker thread — it is never stuck behind the 30 Hz timer
        callback. We block on a threading.Event set by that callback
        instead of busy-polling future.done(), so this thread sleeps
        (using zero CPU) until the result actually arrives or the
        timeout elapses — no executor thread is starved either way.
        """
        if not self.tool_client.service_is_ready():
            if not self.tool_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(
                    "ATC service 'change_tool' unavailable — "
                    "skipping physical tool change.")
                return False

        req = ChangeTool.Request()
        req.tool_name = tool

        done_event = threading.Event()
        future = self.tool_client.call_async(req)
        future.add_done_callback(lambda _f: done_event.set())

        if not done_event.wait(timeout=timeout_sec):
            self.get_logger().error(
                f"ATC service call timed out after {timeout_sec}s.")
            return False

        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"ATC service call raised: {e}")
            return False

        if not resp.success:
            self.get_logger().warn(f"ATC reported failure: {resp.message}")
        return resp.success

    # ── Moondream query with retry (zero empty responses) ──────────
    def _query_moondream(self, prompt: str, image_bytes: bytes) -> str:
        """
        Calls ollama.chat with up to AI_MAX_RETRIES attempts. A 1.8B model
        occasionally returns an empty string or raises — retry before
        giving up so a single hiccup doesn't poison the confidence engine.
        """
        used_retry_line = False
        for attempt in range(1, AI_MAX_RETRIES + 1):
            try:
                resp = ollama.chat(
                    model='moondream',
                    messages=[{'role': 'user', 'content': prompt, 'images': [image_bytes]}],
                    keep_alive=-1,
                )
                txt = resp['message']['content'].strip()
                if txt:
                    if used_retry_line:
                        sys.stdout.write("\n")
                        sys.stdout.flush()
                    return txt
                sys.stdout.write(
                    f"\r  [AI] Retrying inference... (Attempt {attempt}/{AI_MAX_RETRIES})")
                sys.stdout.flush()
                used_retry_line = True
                time.sleep(2.0)   # give the Ollama API time to recover
                continue
            except Exception:
                sys.stdout.write(
                    f"\r  [AI] Retrying inference... (Attempt {attempt}/{AI_MAX_RETRIES})")
                sys.stdout.flush()
                used_retry_line = True
                time.sleep(2.0)   # give the Ollama API time to recover
                continue
        if used_retry_line:
            sys.stdout.write("\n")
            sys.stdout.flush()
        return AI_FAILURE_MARKER

    # ── AI analysis (runs in daemon thread) ────────────────────────
    def _run_analysis(self, crop: np.ndarray, cdata: dict,
                      center: tuple, task_id: int,
                      X: float, Y: float, Z: float, bbox: tuple,
                      cls_name: str) -> None:
        """
        Daemon thread entry point.
        Calls Moondream with two prompts, scores confidence,
        applies the tool state machine, and publishes if appropriate.
        Blocks on self._input_event for the operator's target hole count,
        confirmed via a number key on the OpenCV window — safe because
        this is not the ROS 2 executor thread.
        """
        try:
            SEP = "═" * 62
            sep = "─" * 62
            print(f"\n{SEP}")
            print(f"  ANALYSIS  Task #{task_id}"
                  f"   Centre: ({center[0]}, {center[1]}) px"
                  f"   X:{X*1000:.1f} mm  Y:{Y*1000:.1f} mm")
            print(SEP)
            print("  [DISCLAIMER] Moondream 1.8B is a lightweight edge model. "
                  "Text outputs may occasionally drop context.")

            cv_holes = cdata['hole_count']
            hole_cs  = cdata.get('hole_contours', [])

            # ── Encode crop ──────────────────────────────────────
            _, buf       = cv2.imencode('.jpg', crop, [cv2.IMWRITE_JPEG_QUALITY, 90])
            image_bytes  = buf.tobytes()

            # ── Moondream: two natural-language prompts, scoped by the ──
            # YOLO class name to keep the VLM anchored on MDF wood and
            # stop it hallucinating metal/rust vocabulary on black parts.
            prompts = [
                ("Description", f"This is a flat, black structural component made of "
                                 f"MDF wood, classified as a '{cls_name}'. Act as a "
                                 f"quality inspector. Briefly describe its shape and "
                                 f"surface condition. Since this is wood, do NOT use "
                                 f"words like 'rust', 'metal', or 'plastic'."),
                ("Counting",    f"Carefully count the circular holes in this black MDF "
                                 f"'{cls_name}'.Explain your counting process, and then state the final number of holes."),
            ]

            answers = []
            for label, p in prompts:
                with self._task_lock:
                    stale = task_id != self.current_task_id
                if stale:
                    return    # stale task — a newer 'c' press arrived
                txt = self._query_moondream(p, image_bytes)
                answers.append(txt)
                print(f"\n  [AI:{label}] {p}")
                print(f"             → {txt}")

            description, ai_hole_text = answers

            # ── Parse AI hole count ("double-dip": try the Counting ────
            #    answer first, then fall back to the Description answer
            #    in case the model already counted holes there) ────────
            word_map = {
                'zero': 0, 'one': 1, 'two': 2, 'three': 3, 'four': 4,
                'five': 5, 'six': 6, 'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10,
            }

            def _extract_count(text: str):
                if not text or text.strip() == AI_FAILURE_MARKER:
                    return None
                digit_match = re.search(r'\b(\d+)\b', text)
                if digit_match:
                    return int(digit_match.group(1)), False
                for word, num in word_map.items():
                    if word in text.lower():
                        return num, True
                return None

            extracted = _extract_count(ai_hole_text)
            if extracted is None:
                extracted = _extract_count(description)

            if extracted is not None:
                ai_holes, fallback_used = extracted
                ai_success = True
            else:
                ai_holes      = cv_holes   # safe internal default — display stays honest below
                fallback_used = True
                ai_success    = False

            # ── Confidence scoring ───────────────────────────────
            cv_conf = score_cv_confidence(hole_cs)
            ai_conf = score_ai_confidence(ai_hole_text, fallback_used)

            ai_count_display = f"{ai_holes:>6}" if ai_success else f"{'[ Failed ]':>6}"

            print(f"\n{sep}")
            print(f"  SURFACE  : {description}")
            print(sep)
            print(f"  {'Source':<14}  {'Count':>6}  {'Confidence':>12}")
            print(f"  {'──────────────':<14}  {'─────':>6}  {'──────────':>12}")
            print(f"  {'CV (Shadow Slicer)':<14}  {cv_holes:>6}  {cv_conf*100:>10.0f}%")
            print(f"  {'Moondream 1.8B':<14}  {ai_count_display}  {ai_conf*100:>10.0f}%")
            print(sep)

            # ── Resolution: highest confidence wins ──────────────
            if cv_conf >= ai_conf:
                final_holes = cv_holes
                winner      = f"CV Vision      ({cv_conf*100:.0f}%)"
            else:
                final_holes = ai_holes
                winner      = f"Moondream AI   ({ai_conf*100:.0f}%)"

            print(f"  ✔  FINAL HOLE COUNT : {final_holes}   winner → {winner}")
            print(sep)

            # ── Operator input (GUI, via the OpenCV window) ───────
            # input() would contend with cv2.waitKey() for the terminal,
            # so the operator confirms the target hole count with a
            # number key on the live video window instead; process_frame
            # sets self._target_holes and wakes this thread via the event.
            print()
            print("  >>> Press 1-9 on the camera window to confirm TARGET HOLE COUNT, or 'q' to abort <<<")
            self._hud_line = "Press 1-9 to confirm target holes, or 'q' to abort"
            self._input_event.clear()
            self._input_event.wait()

            with self._task_lock:
                stale = task_id != self.current_task_id
            if stale or self._target_holes is None:
                print("  [INFO] Task aborted or superseded.")
                return  # Safely exit the analysis thread without publishing

            target_holes = self._target_holes

            # ── Publish hole-count topics ─────────────────────────
            self.target_holes_pub.publish(Int32(data=target_holes))
            self.actual_holes_pub.publish(Int32(data=final_holes))

            # ── Tool state machine ───────────────────────────────
            print(f"\n  Detected: {final_holes}   Target: {target_holes}")
            print(sep)

            if final_holes == target_holes:
                tool    = "gripper"
                outcome = f"✔  PICK    → Tool: GRIPPER"

            elif final_holes < target_holes:
                tool    = "drill"
                missing = target_holes - final_holes
                outcome = (f"⚠  DRILL   → {missing} hole(s) missing,"
                           f" routing to drill station.")

            else:
                excess  = final_holes - target_holes
                outcome = (f"✘  REJECT  → {excess} excess hole(s) found."
                           f" Part rejected — not publishing.")
                print(f"  {outcome}")
                print(f"{SEP}\n")
                self._hud_line = (f"Task #{task_id}: REJECTED  "
                                  f"({final_holes} holes > target {target_holes})")
                return   # do NOT publish

            print(f"  {outcome}")

            # ── ATC: physically change tool before publishing ────
            print(f"\n  [ATC] Requesting tool change → {tool.upper()} ...")
            tool_ok = self._call_tool_changer(tool)
            if tool_ok:
                print(f"  [ATC] Tool change confirmed: {tool.upper()}")
            else:
                print(f"  [ATC] WARNING: tool change failed or timed out — "
                      f"publishing coordinates anyway.")

            # ── Publish ──────────────────────────────────────────
            msg = Point()
            msg.x, msg.y, msg.z = X, Y, Z
            self.coord_pub.publish(msg)

            print(f"\n  [ROS2] Published → /workpiece_coordinates")
            print(f"         X: {X*1000:+.2f} mm   Y: {Y*1000:+.2f} mm   Z: {Z*1000:.0f} mm")
            print(f"         Tool dispatched: {tool.upper()}")
            print(f"{SEP}\n")

            self._hud_line = (f"Task #{task_id}: {tool.upper()}"
                              f"  X:{X*1000:.1f}  Y:{Y*1000:.1f}")
            self.get_logger().info(
                f"Task #{task_id} done — tool={tool}  holes={final_holes}/{target_holes}"
                f"  coords=({X:.4f}, {Y:.4f}, {Z:.4f})")

        except Exception as e:
            self.get_logger().error(f"Analysis thread error: {e}")
        finally:
            self.is_analyzing = False

    # ── ROS 2 timer callback (~30 Hz) ──────────────────────────────
    def process_frame(self) -> None:
        """
        Called every 33 ms by the ROS 2 executor.
        Reads a camera frame, runs undistortion + YOLO + contour analysis,
        updates the snapshot, renders the OpenCV window, and handles keys.
        Never blocks — AI is off-loaded to a daemon thread on 'c'.
        """
        try:
            ret, raw = self.cap.read()
            if not ret:
                self.get_logger().warn(
                    "Camera read failed — retrying.", throttle_duration_sec=2.0)
                return

            frame = self._undistort(raw)
            bbox, cls_name = self._detect(frame)

            if bbox is not None:
                x1, y1, x2, y2 = bbox
                cx, cy          = (x1 + x2) // 2, (y1 + y2) // 2
                X, Y, Z         = self._pixel_to_world(cx, cy)
                cdata           = self._analyze_contours(frame, bbox)

                # Update live snapshot for the next 'c' press — clear any
                # stale item first so the queue always holds at most the
                # single most recent detection.
                snapshot = DetectionSnapshot(
                    frame=frame, bbox=bbox, center=(cx, cy),
                    cdata=cdata, coords=(X, Y, Z), cls_name=cls_name,
                )
                try:
                    self._snap_queue.get_nowait()
                except queue.Empty:
                    pass
                self._snap_queue.put_nowait(snapshot)

                display = self._annotate(frame, bbox, cdata, X, Y)
            else:
                # No detection — draw centred search zone
                fb   = frame.copy()
                h, w = fb.shape[:2]
                cv2.rectangle(fb, (0, 0), (w, 26), (0, 0, 0), -1)
                cv2.putText(fb, self._hud_line, (8, 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 80, 200), 1)
                cv2.rectangle(fb, (w//2 - 150, h//2 - 150),
                              (w//2 + 150, h//2 + 150), (0, 0, 180), 2)
                cv2.putText(fb, "NO WORKPIECE DETECTED",
                            (w//2 - 135, h//2 - 162),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 180), 2)
                display = fb

            cv2.imshow('Workpiece Inspection  v3.0', display)
            key = cv2.waitKey(1) & 0xFF

            # ── Key: q → quit ─────────────────────────────────
            if key == ord('q'):
                self.get_logger().info("Quit requested by operator.")
                # Wake any daemon thread blocked on _input_event so it can
                # see the (unchanged) current_task_id / None target_holes,
                # recognize the abort, and exit cleanly before the node
                # is torn down below.
                self._input_event.set()
                # `raise SystemExit` here would be swallowed silently:
                # this callback runs on a MultiThreadedExecutor worker
                # thread, and BaseException raised inside a callback is
                # captured in its Future and never observed, so spin()
                # never returns. Shutting down rclpy directly makes
                # spin()'s `while context.ok()` loop exit instead.
                if rclpy.ok():
                    rclpy.shutdown()
                return

            # ── Key: c → launch analysis thread ───────────────
            elif key == ord('c'):
                # Unblock any daemon thread still waiting on a target-hole
                # key press — its staleness check will make it exit cleanly.
                self._input_event.set()
                if not self.is_analyzing:
                    try:
                        snapshot = self._snap_queue.get_nowait()
                    except queue.Empty:
                        self._hud_line = "No detection yet — waiting for a workpiece."
                    else:
                        self.is_analyzing = True
                        with self._task_lock:
                            self.current_task_id += 1
                            tid = self.current_task_id

                        self._hud_line = f"ANALYZING  Task #{tid} …  (see terminal)"

                        # Capture a clean, independent crop from the snapshot's
                        # own frame — never the timer's current `frame` variable,
                        # which may have moved on by the time the thread runs.
                        bbox  = snapshot.bbox
                        cdata = dict(snapshot.cdata)   # shallow copy of the dict
                        X, Y, Z = snapshot.coords
                        crop = snapshot.frame[
                            max(0, bbox[1] - 10): min(snapshot.frame.shape[0], bbox[3] + 40),
                            max(0, bbox[0] - 10): min(snapshot.frame.shape[1], bbox[2] + 40),
                        ].copy()

                        threading.Thread(
                            target=self._run_analysis,
                            args=(crop, cdata, snapshot.center, tid, X, Y, Z, bbox,
                                  snapshot.cls_name),
                            daemon=True,
                        ).start()

            # ── Key: 1-9 → confirm target hole count ──────────
            elif ord('1') <= key <= ord('9') and self.is_analyzing:
                self._target_holes = key - ord('0')
                self._input_event.set()

        except Exception as e:
            self.get_logger().error(f"process_frame error: {e}")


# ═══════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)

    try:
        node = WorkpieceInspectorNode()
    except SystemExit:
        # Startup circuit breaker tripped (YOLO/camera failure). The node
        # has already torn itself down — just release the rclpy context
        # and propagate the non-zero exit so a launch/supervisor sees it
        # as a failure, not a clean shutdown.
        if rclpy.ok():
            rclpy.shutdown()
        raise

    # MultiThreadedExecutor gives the timer callback (default group) and
    # the ATC service client (its own MutuallyExclusiveCallbackGroup) each
    # a dedicated worker thread, so a pending tool-change response is
    # never queued behind the 30 Hz frame timer.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if hasattr(node, 'cap') and node.cap.isOpened():
            node.cap.release()
        cv2.destroyAllWindows()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
