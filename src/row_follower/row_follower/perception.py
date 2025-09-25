#!/usr/bin/env python3
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, String, Int32
from cv_bridge import CvBridge

def _clip(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

class Perception(Node):
    def __init__(self):
        super().__init__("perception")

        # --- Core parameters ---
        self.declare_parameter("lane_width_m", 1.0)
        self.declare_parameter("follow_mode", "center")   # "center" or "left_offset"
        self.declare_parameter("left_offset_m", 0.20)
        self.declare_parameter("num_bands", 24)
        self.declare_parameter("y_low_frac", 0.00)        # used when roi_enable=false
        self.declare_parameter("y_high_frac", 0.95)

        self.declare_parameter("roi_switch_delta", 0.05)
        self.declare_parameter("roi_hold_frames", 5)
        self.declare_parameter("edge_max_step_px", 40.0)
        self.declare_parameter("edge_alpha", 0.35)
        self.declare_parameter("min_lane_width_px", 120.0)
        self.declare_parameter("max_lane_width_px", 1200.0)
        self.declare_parameter("search_radius_px", 120.0)

        # Adaptive ROI params (no nested lists!)
        self.declare_parameter("roi_enable", True)
        self.declare_parameter("roi_ground_min", 0.15)
        self.declare_parameter("roi_lane_area_min", 0.15)
        self.declare_parameter("roi_band_lows",  [0.0, 0.0, 0.30, 0.50])
        self.declare_parameter("roi_band_highs", [0.70, 0.95, 0.95, 0.95])

        gp = self.get_parameter
        self.lane_width_m = float(gp("lane_width_m").value)
        self.follow_mode = str(gp("follow_mode").value)
        self.left_offset_m = float(gp("left_offset_m").value)
        self.num_bands = int(gp("num_bands").value)
        self.y_low_frac = float(gp("y_low_frac").value)
        self.y_high_frac = float(gp("y_high_frac").value)

        self.roi_enable = bool(gp("roi_enable").value)
        self.roi_ground_min = float(gp("roi_ground_min").value)
        self.roi_lane_area_min = float(gp("roi_lane_area_min").value)
        self.roi_band_lows = list(gp("roi_band_lows").value)
        self.roi_band_highs = list(gp("roi_band_highs").value)

        # pubs/subs
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.cb, 10)

        self.pub_green_lane = self.create_publisher(Float32, "/perception/green_ratio_lane", 10)
        self.pub_ground_lane = self.create_publisher(Float32, "/perception/ground_ratio_lane", 10)
        self.pub_green_full = self.create_publisher(Float32, "/perception/green_ratio_full", 10)
        self.pub_lane_area = self.create_publisher(Float32, "/perception/lane_area_ratio", 10)

        self.pub_sym = self.create_publisher(Float32, "/perception/crop_symmetry", 10)
        self.pub_edges = self.create_publisher(Float32MultiArray, "/perception/lane_edges", 10)
        self.pub_lane_px = self.create_publisher(Float32, "/perception/lane_width_px", 10)
        self.pub_lat_px = self.create_publisher(Float32, "/perception/lateral_px", 10)
        self.pub_lat_m = self.create_publisher(Float32, "/perception/lateral_m", 10)

        self.pub_conf = self.create_publisher(Float32, "/perception/confidence", 10)
        self.pub_conf_lane = self.create_publisher(Float32, "/perception/conf_lane_green", 10)
        self.pub_conf_full = self.create_publisher(Float32, "/perception/conf_full_green", 10)
        self.pub_conf_area = self.create_publisher(Float32, "/perception/conf_lane_area", 10)
        self.pub_conf_level = self.create_publisher(Int32, "/perception/conf_level", 10)
        self.pub_roi_idx = self.create_publisher(Int32, "/perception/roi_band_idx", 10)

        self.pub_row_errors = self.create_publisher(Float32MultiArray, "/row_errors", 10)

        self.pub_state = self.create_publisher(String, "/perception/state_str", 10)
        self.pub_mask = self.create_publisher(Image, "/perception/mask", 10)
        self.pub_overlay = self.create_publisher(Image, "/perception/overlay", 10)

        self.bridge = CvBridge()
        self.prev_s = None

        self.prev_roi_idx = 1
        self.prev_roi_score = 0.0
        self.roi_hold = 0

        self.trk_has_prev = False
        self.xL_prev = None
        self.xR_prev = None
        self.w_prev = None

    # --- segmentation, helpers (unchanged) ---
    def green_mask(self, hsv):
        lower = np.array([30, 40, 30], dtype=np.uint8)
        upper = np.array([90, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)
        return mask

    @staticmethod
    def _x_of_y(line, y):
        vx, vy, x0, y0 = [float(v) for v in line.reshape(-1)]
        if abs(vy) < 1e-6:
            return float(x0)
        return x0 + (y - y0) * vx / vy

    def _clip_x_at_y(self, line, y, W):
        x = self._x_of_y(line, y)
        return None if not np.isfinite(x) else float(np.clip(x, 0.0, W - 1.0))

    def _band_score(self, x, center, tol_low, tol_high):
        if x <= center:
            d = (center - x) / max(1e-6, tol_low)
        else:
            d = (x - center) / max(1e-6, tol_high)
        return float(_clip(1.0 - d, 0.0, 1.0))

    # ---------- per-ROI processing (unchanged) ----------
    def _process_roi_band(self, mask, H, W, y_low_frac, y_high_frac):
        total_px = float(H * W)
        y_low = int(float(y_low_frac) * H)
        y_high = int(float(y_high_frac) * H)
        band_h = max(3, (y_high - y_low) // max(3, self.num_bands))

        pts_left, pts_right, _ = self.collect_inner_edge_points(mask, W, y_low, y_high, band_h)
        if len(pts_left) < 2 or len(pts_right) < 2:
            cl, cr = self.contour_edges_fallback(mask)
            if cl is not None and cr is not None:
                pts_left, pts_right = cl, cr
        if len(pts_left) < 2 or len(pts_right) < 2:
            return False, {"reason": "few edge points"}

        lineL = cv2.fitLine(np.array(pts_left, dtype=np.float32), cv2.DIST_L2, 0, 0.01, 0.01)
        lineR = cv2.fitLine(np.array(pts_right, dtype=np.float32), cv2.DIST_L2, 0, 0.01, 0.01)

        rows = np.linspace(max(0, y_low), min(H - 1, y_high), num=25)
        good_rows = []
        for yy in rows:
            xl = self._clip_x_at_y(lineL, yy, W)
            xr = self._clip_x_at_y(lineR, yy, W)
            if xl is None or xr is None:
                continue
            if xr - xl >= 3.0:
                good_rows.append((yy, xl, xr))
        if len(good_rows) < 2:
            return False, {"reason": "no valid rows"}

        ys_only = np.array([g[0] for g in good_rows], dtype=float)
        mid_idx = int(np.argsort(ys_only)[len(ys_only)//2])
        y_star, xL_star, xR_star = good_rows[mid_idx]
        lane_width_px = float(xR_star - xL_star)
        r = lane_width_px / float(W)

        s_inst = self.lane_width_m / max(1.0, lane_width_px)

        left_pts = np.array([[g[1], g[0]] for g in good_rows], dtype=np.float32)
        right_pts = np.array([[g[2], g[0]] for g in good_rows][::-1], dtype=np.float32)
        poly = np.vstack([left_pts, right_pts])
        lane_mask = np.zeros((H, W), dtype=np.uint8)
        cv2.fillPoly(lane_mask, [poly.astype(np.int32)], 255)
        lane_area = float(cv2.countNonZero(lane_mask))
        if lane_area < 10.0:
            return False, {"reason": "small polygon"}

        green_mask_inside = cv2.bitwise_and(mask, lane_mask)
        green_inside = float(cv2.countNonZero(green_mask_inside))
        green_lane = green_inside / lane_area
        ground_lane = max(0.0, 1.0 - green_lane)

        green_full = float(cv2.countNonZero(mask)) / float(H * W)
        lane_area_ratio = lane_area / float(H * W)

        xmid_star = 0.5 * (xL_star + xR_star)
        lateral_px = float(xmid_star - xmid_star)  # center for scoring
        crop_sym = float(_clip(lateral_px / max(1.0, lane_width_px), -0.5, 0.5))

        xmid_samples = 0.5 * (left_pts[:, 0] + right_pts[::-1][:, 0])
        z = np.polyfit(ys_only, xmid_samples, 1)
        dxdy = float(z[0])
        heading = float(math.atan2(dxdy, 1.0))

        info = dict(
            y_low=y_low, y_high=y_high, y_star=y_star,
            xL_star=xL_star, xR_star=xR_star,
            lane_width_px=lane_width_px, r=r, s_inst=s_inst,
            lane_area=lane_area, lane_area_ratio=lane_area_ratio,
            green_lane=green_lane, ground_lane=ground_lane, green_full=green_full,
            crop_sym=crop_sym, heading=heading,
            good_rows=good_rows, left_pts=left_pts, right_pts=right_pts, poly=poly
        )
        return True, info

    def cb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv bridge error {e}")
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = self.green_mask(hsv)
        H, W, _ = bgr.shape

        # -------- Build ROI candidate list from two arrays --------
        if self.roi_enable:
            lows = [float(x) for x in self.roi_band_lows]
            highs = [float(x) for x in self.roi_band_highs]
            n = min(len(lows), len(highs))
            roi_defs = []
            for i in range(n):
                lo = _clip(lows[i], 0.0, 1.0)
                hi = _clip(highs[i], 0.0, 1.0)
                if hi > lo + 0.05:
                    roi_defs.append((lo, hi))
            if not roi_defs:
                roi_defs = [(0.00, 0.70), (0.00, 0.95), (0.30, 0.95), (0.50, 0.95)]
        else:
            roi_defs = [(self.y_low_frac, self.y_high_frac)]

        # -------- Evaluate candidates --------
        candidates = []
        for idx, (lo, hi) in enumerate(roi_defs):
            ok, info = self._process_roi_band(mask, H, W, lo, hi)
            if not ok:
                candidates.append({"idx": idx, "ok": False, "reason": info.get("reason", "fail")})
                continue

            good_rows = len(info["good_rows"])
            lane_area_ratio = info["lane_area_ratio"]
            green_lane = info["green_lane"]
            ground_lane = info["ground_lane"]
            crop_sym = abs(info["crop_sym"])
            r = info["r"]

            qual_ok = (
                (ground_lane >= self.roi_ground_min) and
                (good_rows >= 2) and
                (lane_area_ratio >= self.roi_lane_area_min)
            )

            geom_rows = min(1.0, max(0.0, (good_rows / 16.0)))
            geom_r = _clip((r - 0.1) / 0.9, 0.0, 1.0)
            geom_area = _clip((lane_area_ratio - 0.15) / 0.35, 0.0, 1.0)
            geom_ok = 0.5*geom_rows + 0.3*geom_r + 0.2*geom_area

            score = (0.50 * green_lane) + (0.20 * lane_area_ratio) + (0.20 * geom_ok) - (0.10 * crop_sym)

            candidates.append({
                "idx": idx, "ok": qual_ok, "score": score,
                "info": info, "good_rows": good_rows, "lane_area_ratio": lane_area_ratio
            })

        # choose winner + hysteresis
        switch_delta = float(self.get_parameter("roi_switch_delta").value)
        hold_frames  = int(self.get_parameter("roi_hold_frames").value)

        ok_candidates = [c for c in candidates if c.get("ok", False)]
        winner = max(ok_candidates, key=lambda c: c["score"]) if ok_candidates else None
        if winner is None:
            # fallback to most geometric
            geom_sorted = sorted([c for c in candidates if "good_rows" in c],
                                 key=lambda c: (c["good_rows"], c["lane_area_ratio"]),
                                 reverse=True)
            winner = geom_sorted[0] if geom_sorted else None

        if winner is None or "info" not in winner:
            return self.fail_and_publish(mask, reason="no ROI workable")

        # hysteresis against flapping
        if (winner["idx"] != self.prev_roi_idx) and (self.roi_hold < hold_frames):
            # force previous if it exists in candidates
            prev_cands = [c for c in candidates if c.get("idx") == self.prev_roi_idx and "info" in c]
            if prev_cands:
                winner = prev_cands[0]
        elif (winner["idx"] != self.prev_roi_idx) and (winner["score"] < self.prev_roi_score + switch_delta):
            prev_cands = [c for c in candidates if c.get("idx") == self.prev_roi_idx and "info" in c]
            if prev_cands:
                winner = prev_cands[0]

        if winner["idx"] == self.prev_roi_idx:
            self.roi_hold += 1
        else:
            self.roi_hold = 0
            self.prev_roi_idx = int(winner["idx"])
        self.prev_roi_score = float(winner["score"])
        self.pub_roi_idx.publish(Int32(data=int(winner["idx"])))

        # -------- Use chosen ROI (rest unchanged, with a guard) --------
        info = winner["info"]
        y_star = info["y_star"]; xL_star = info["xL_star"]; xR_star = info["xR_star"]
        lane_width_px = info["lane_width_px"]; r = info["r"]
        s_inst = info["s_inst"]; left_pts = info["left_pts"]; right_pts = info["right_pts"]
        poly = info["poly"]; lane_area = info["lane_area"]; green_lane = info["green_lane"]
        ground_lane = info["ground_lane"]; green_full = info["green_full"]
        lane_area_ratio = info["lane_area_ratio"]
        ys_only = np.array([g[0] for g in info["good_rows"]], dtype=float)

        # smooth pixel->meter
        self.prev_s = s_inst if self.prev_s is None else (0.8 * self.prev_s + 0.2 * s_inst)
        s = self.prev_s

        # temporal constraints on edges (with None-guard)
        edge_alpha   = float(self.get_parameter("edge_alpha").value)
        edge_maxstep = float(self.get_parameter("edge_max_step_px").value)
        w_min = float(self.get_parameter("min_lane_width_px").value)
        w_max = float(self.get_parameter("max_lane_width_px").value)
        sr    = int(float(self.get_parameter("search_radius_px").value))

        xL_meas, xR_meas = float(xL_star), float(xR_star)
        w_meas = float(lane_width_px)

        w_meas = float(_clip(w_meas, w_min, w_max))
        xmid  = 0.5 * (xL_meas + xR_meas)
        xL_meas = xmid - 0.5*w_meas
        xR_meas = xmid + 0.5*w_meas

        if self.trk_has_prev and (self.xL_prev is not None) and (self.xR_prev is not None):
            xL_meas = self.xL_prev + _clip(xL_meas - self.xL_prev, -edge_maxstep, edge_maxstep)
            xR_meas = self.xR_prev + _clip(xR_meas - self.xR_prev, -edge_maxstep, edge_maxstep)
            xL_s = (1.0-edge_alpha)*self.xL_prev + edge_alpha*xL_meas
            xR_s = (1.0-edge_alpha)*self.xR_prev + edge_alpha*xR_meas
        else:
            xL_s, xR_s = xL_meas, xR_meas
            self.trk_has_prev = True

        xL_star, xR_star = xL_s, xR_s
        lane_width_px = float(xR_star - xL_star)

        # Lateral / symmetry / heading (follow_mode)
        xmid_star = 0.5 * (xL_star + xR_star)
        if self.follow_mode == "left_offset":
            x_tar = xL_star + self.left_offset_m / s
        else:
            x_tar = xmid_star
        lateral_px = float(xmid_star - x_tar)
        lateral_m = float(lateral_px * s)
        crop_sym = float(_clip(lateral_px / max(1.0, lane_width_px), -0.5, 0.5))

        xmid_samples = 0.5 * (left_pts[:, 0] + right_pts[::-1][:, 0])
        z = np.polyfit(ys_only, xmid_samples, 1)
        dxdy = float(z[0])
        heading = float(math.atan2(dxdy, 1.0))

        # confidence (unchanged band logic)
        full_green_target  = 0.10 + 0.40 * max(0.0, (0.90 - r)) / 0.90
        full_green_tol     = 0.20 + 0.05 * max(0.0, (0.50 - r)) / 0.50

        lane_green_target   = 0.08 + 0.06 * max(0.0, (0.30 - r)) / 0.30
        lane_green_tol_low  = 0.06
        lane_green_tol_high = 0.25 + 0.05 * max(0.0, (0.50 - r)) / 0.50

        lane_area_min_ratio = 0.25 if r > 0.30 else 0.20

        c_lane = self._band_score(green_lane, lane_green_target, lane_green_tol_low, lane_green_tol_high)
        c_full = self._band_score(green_full, full_green_target,  full_green_tol,     full_green_tol)

        if lane_area_ratio <= lane_area_min_ratio:
            c_area = 0.0
        elif lane_area_ratio >= 0.50:
            c_area = 1.0
        else:
            c_area = float((lane_area_ratio - lane_area_min_ratio) / (0.50 - lane_area_min_ratio))
        c_area = _clip(c_area, 0.0, 1.0)

        good_rows_count = len(info["good_rows"])
        geom_floor = 0.0
        if r >= 0.90 and lane_area_ratio >= 0.85 and good_rows_count >= 8:
            geom_floor = 0.40 + 0.30 * min(1.0, (good_rows_count - 8) / 8.0)
        c_lane = max(c_lane, geom_floor)

        confidence = float(min(c_lane, c_full, c_area))
        if confidence >= 0.60:   conf_level = 2
        elif confidence >= 0.35: conf_level = 1
        else:                    conf_level = 0

        # publish
        self.pub_green_lane.publish(Float32(data=float(_clip(green_lane, 0.0, 1.0))))
        self.pub_ground_lane.publish(Float32(data=float(_clip(1.0 - green_lane, 0.0, 1.0))))
        self.pub_green_full.publish(Float32(data=float(_clip(green_full, 0.0, 1.0))))
        self.pub_lane_area.publish(Float32(data=float(_clip(lane_area_ratio, 0.0, 1.0))))

        self.pub_lane_px.publish(Float32(data=lane_width_px))
        self.pub_lat_px.publish(Float32(data=lateral_px))
        self.pub_lat_m.publish(Float32(data=lateral_m))
        self.pub_sym.publish(Float32(data=crop_sym))
        self.pub_edges.publish(Float32MultiArray(data=[float(xL_star), float(xR_star)]))

        self.pub_conf_lane.publish(Float32(data=c_lane))
        self.pub_conf_full.publish(Float32(data=c_full))
        self.pub_conf_area.publish(Float32(data=c_area))
        self.pub_conf.publish(Float32(data=confidence))
        self.pub_conf_level.publish(Int32(data=conf_level))
        self.pub_roi_idx.publish(Int32(data=int(self.prev_roi_idx)))

        self.pub_row_errors.publish(Float32MultiArray(data=[crop_sym, heading]))

        sline = (
            f"OK y*={int(y_star)} lane_px={lane_width_px:.1f} "
            f"green_lane={green_lane:.3f} ground_lane={1.0-green_lane:.3f} "
            f"green_full={green_full:.3f} lane_area={lane_area_ratio:.3f} "
            f"c_lane={c_lane:.2f} c_full={c_full:.2f} c_area={c_area:.2f} conf={confidence:.2f} "
            f"sym={crop_sym:.3f} lat_m={lateral_m:.3f} head={heading:.3f} roi={int(self.prev_roi_idx)}"
        )
        self.pub_state.publish(String(data=sline))

        overlay = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.polylines(overlay, [poly.astype(np.int32)], True, (0, 255, 0), 2)
        cv2.circle(overlay, (int(xL_star), int(y_star)), 4, (0, 0, 255), -1)
        cv2.circle(overlay, (int(xR_star), int(y_star)), 4, (0, 0, 255), -1)
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, encoding="mono8"))
        self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8"))

    # ---------- helpers ----------
    def collect_inner_edge_points(self, mask, W, y_low, y_high, band_h):
        xs_left, xs_right, mids = [], [], []
        y = max(0, y_high - band_h)
        while y >= max(0, y_low):
            band = mask[y:y + band_h, :]
            col = band.sum(axis=0)
            mid = W // 2

            left_idx = None
            for x in range(mid, 1, -1):
                if col[x] > 0 and col[x - 1] == 0:
                    left_idx = x; break

            right_idx = None
            for x in range(mid, W - 2):
                if col[x] > 0 and col[x + 1] == 0:
                    right_idx = x; break

            cy = y + band_h // 2
            if left_idx is not None:  xs_left.append([left_idx, cy])
            if right_idx is not None: xs_right.append([right_idx, cy])
            if left_idx is not None and right_idx is not None and right_idx > left_idx + 2:
                mids.append([0.5 * (left_idx + right_idx), cy])

            y -= band_h
        return xs_left, xs_right, mids

    def contour_edges_fallback(self, mask):
        H, W = mask.shape
        left = mask[:, : W // 2]
        right = mask[:, W // 2 :]

        def side_points(binary, offset_x):
            cnts, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts: return None
            cnt = max(cnts, key=cv2.contourArea).reshape(-1, 2)
            byy = {}
            for x, y in cnt:
                byy.setdefault(int(y), []).append(int(x))
            pts = []
            for y, xs in byy.items():
                xi = max(xs) if offset_x == 0 else min(xs)
                pts.append([xi + offset_x, y])
            pts = np.array(sorted(pts, key=lambda p: p[1]))[::2]
            return None if len(pts) < 2 else pts.tolist()

        pl = side_points(left, 0); pr = side_points(right, W // 2)
        return (pl, pr) if (pl is not None and pr is not None) else (None, None)

    def fail_and_publish(self, mask, reason=""):
        self.get_logger().warn(f"ROI not found ({reason})")
        zeros = [0.0, 0.0]
        self.pub_green_lane.publish(Float32(data=0.0))
        self.pub_ground_lane.publish(Float32(data=0.0))
        self.pub_green_full.publish(Float32(data=0.0))
        self.pub_lane_area.publish(Float32(data=0.0))
        self.pub_lane_px.publish(Float32(data=0.0))
        self.pub_lat_px.publish(Float32(data=0.0))
        self.pub_lat_m.publish(Float32(data=0.0))
        self.pub_sym.publish(Float32(data=0.0))
        self.pub_edges.publish(Float32MultiArray(data=zeros))
        self.pub_conf_lane.publish(Float32(data=0.0))
        self.pub_conf_full.publish(Float32(data=0.0))
        self.pub_conf_area.publish(Float32(data=0.0))
        self.pub_conf.publish(Float32(data=0.0))
        self.pub_conf_level.publish(Int32(data=0))
        self.pub_row_errors.publish(Float32MultiArray(data=zeros))
        self.pub_state.publish(String(data=f"FAIL {reason}"))
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, encoding="mono8"))

def main(args=None):
    rclpy.init(args=args)
    node = Perception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
