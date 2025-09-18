import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, 10)

        # Row-following outputs
        self.pub_row_errors = self.create_publisher(Float32MultiArray, '/row_errors', 10)

        # Debug image (binary mask)
        self.pub_dbg_mask = self.create_publisher(Image, '/perception/mask', 10)

        # Lane-band metrics (publish as simple Float32 for easy echo/plot)
        self.pub_green_ratio_lane = self.create_publisher(Float32, '/perception/green_ratio_lane', 10)
        self.pub_ground_ratio_lane = self.create_publisher(Float32, '/perception/ground_ratio_lane', 10)
        self.pub_crop_symmetry     = self.create_publisher(Float32, '/perception/crop_symmetry', 10)
        self.pub_lane_width_px     = self.create_publisher(Float32, '/perception/lane_width_px', 10)
        self.pub_lateral_px        = self.create_publisher(Float32, '/perception/lateral_px', 10)
        self.pub_lateral_m         = self.create_publisher(Float32, '/perception/lateral_m', 10)
        self.pub_lane_edges        = self.create_publisher(Float32MultiArray, '/perception/lane_edges', 10)  # [x_L*, x_R*]

        # Hyperparameters
        self.lane_width_m = 1.0          # real-world lane width in meters (for px->m conversion)
        self.lookahead_frac = 0.66       # y* = lookahead as a fraction of image height (0..1)
        self.num_bands = 6               # number of horizontal bands for sliding-window tracking
        self.band_height_frac = 0.10     # each band height as a fraction of image height
        self.y_min_frac, self.y_max_frac = 0.30, 0.95  # vertical range for bands (near field)

        self.last_log = 0.0

    # ---------- Utility helpers ----------
    @staticmethod
    def _contiguous_segments(bool_arr: np.ndarray):
        """
        Return list of [start_idx, end_idx] (inclusive) for contiguous True regions in a 1D boolean array.
        """
        idx = np.where(bool_arr)[0]
        if idx.size == 0:
            return []
        splits = np.where(np.diff(idx) > 1)[0] + 1
        groups = np.split(idx, splits)
        return [(int(g[0]), int(g[-1])) for g in groups]

    def _find_edges_in_band(self, band_mask: np.ndarray, center_x: int):
        """
        Given a small horizontal band mask (binary: 0/255), find inner edges of the nearest-left and nearest-right crops.
        Returns (x_L, x_R) as integers, or (None, None) if not found.
        Strategy:
          - Column-wise sum -> boolean columns (green present)
          - Find contiguous segments -> pick left segment whose right edge is closest to center & < center
                                   -> pick right segment whose left edge is closest to center & > center
          - Inner edges: x_L = right edge of left segment; x_R = left edge of right segment
        """
        h_band, w_band = band_mask.shape[:2]
        # binarize: True if column has enough green pixels (>= 15% of rows)
        colsum = (band_mask > 0).sum(axis=0)
        thresh_cols = int(0.15 * h_band)
        col_bool = colsum >= thresh_cols

        segs = self._contiguous_segments(col_bool)
        if not segs:
            return None, None

        left_cands  = [seg for seg in segs if seg[1] < center_x]   # seg = (start, end)
        right_cands = [seg for seg in segs if seg[0] > center_x]

        if not left_cands or not right_cands:
            return None, None

        # nearest to center from left: max end
        left_seg  = max(left_cands, key=lambda s: s[1])
        # nearest to center from right: min start
        right_seg = min(right_cands, key=lambda s: s[0])

        x_L = int(left_seg[1])   # inner-right edge of left crop
        x_R = int(right_seg[0])  # inner-left edge of right crop
        if x_R <= x_L:
            return None, None
        return x_L, x_R

    def _polyfit_heading(self, ys: np.ndarray, x_mids: np.ndarray, y_eval: float):
        """
        Fit a quadratic x(y) = a*y^2 + b*y + c to (y, x_mid) pairs and return heading as atan(dx/dy)|_{y_eval}.
        If not enough points, return 0.0.
        """
        if len(ys) < 3:
            return 0.0
        try:
            coeffs = np.polyfit(ys, x_mids, 2)  # [a, b, c]
            a, b = coeffs[0], coeffs[1]
            dx_dy = 2.0 * a * y_eval + b
            heading = float(np.arctan(dx_dy))
            return heading
        except Exception:
            return 0.0

    # ---------- Main callback ----------
    def cb(self, msg):
        # Convert incoming (rgb8) to BGR for OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        x_center = int(w / 2)

        # 1) Build a robust green mask (HSV + green dominance union)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv_mask = cv2.inRange(
            hsv,
            np.array([25, 20, 20], np.uint8),
            np.array([95, 255, 255], np.uint8)
        )
        b, g, r = cv2.split(frame)
        dom = (g.astype(np.int16) - np.maximum(r, b).astype(np.int16))
        dom_mask = (dom > 25).astype(np.uint8) * 255

        mask = cv2.bitwise_or(hsv_mask, dom_mask)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

        # --- Sliding horizontal bands to find nearest left/right inner edges per band ---
        y0 = int(self.y_min_frac * h)
        y1 = int(self.y_max_frac * h)
        band_h = max(4, int(self.band_height_frac * h))
        centers_y = np.linspace(y0 + band_h//2, y1 - band_h//2, num=self.num_bands).astype(int)

        band_edges = []  # list of (y_c, x_L, x_R)
        x_mids = []      # midpoints per band for heading fit
        ys_valid = []

        for y_c in centers_y:
            y_top = max(0, y_c - band_h // 2)
            y_bot = min(h, y_c + band_h // 2)
            band = mask[y_top:y_bot, :]

            x_L, x_R = self._find_edges_in_band(band, x_center)
            if x_L is not None and x_R is not None:
                band_edges.append((int(y_c), int(x_L), int(x_R)))
                x_mids.append(0.5 * (x_L + x_R))
                ys_valid.append(y_c)

        # If nothing detected, publish zeros & debug image then return
        if not band_edges:
            self._publish_debug(mask, msg)
            self._publish_metrics_none()
            self._publish_row_errors(0.0, 0.0)  # [lateral_norm, heading]
            return

        # Choose a lookahead band closest to lookahead_frac
        y_star = int(self.lookahead_frac * h)
        best_idx = int(np.argmin([abs(y_c - y_star) for (y_c, _, _) in band_edges]))
        y_c_star, x_L_star, x_R_star = band_edges[best_idx]
        lane_width_px = float(max(1, x_R_star - x_L_star))  # avoid div by zero

        # Midpoint of lane at y*
        x_mid_star = 0.5 * (x_L_star + x_R_star)

        # Target inside lane:
        #  - Center mode: target = lane midpoint
        #  - Left-offset mode (e.g., 0.2 m from left): uncomment below to use 0.2 m offset
        x_target_star = x_mid_star
        # s = self.lane_width_m / lane_width_px
        # x_target_star = x_L_star + (0.2 / s)  # example: 0.2 m from left inner edge

        # Robot image x (camera center)
        x_robot = float(x_center)

        # Lateral error (pixels & meters) relative to target inside the lane (robot - target)
        lateral_px = float(x_robot - x_target_star)
        s_m_per_px = self.lane_width_m / lane_width_px
        lateral_m  = float(lateral_px * s_m_per_px)

        # Normalized lateral (crop_symmetry): ~ [-0.5, 0.5] when robot is inside lane band
        crop_symmetry = float((x_robot - x_mid_star) / lane_width_px)

        # Heading from quadratic fit of x_mid(y) across valid bands
        heading = self._polyfit_heading(np.array(ys_valid, dtype=np.float32),
                                        np.array(x_mids, dtype=np.float32),
                                        float(y_c_star))

        # Lane-band green/ground ratios:
        # Use all valid bands, summing green pixels ONLY within [x_L_i, x_R_i] per band
        green_count, band_area = 0, 0
        for (y_c, xL, xR) in band_edges:
            y_top = max(0, y_c - band_h // 2)
            y_bot = min(h, y_c + band_h // 2)
            sub = mask[y_top:y_bot, xL:xR]
            green_count += int((sub > 0).sum())
            band_area   += int(sub.size)

        green_ratio_lane = float(green_count) / float(band_area) if band_area > 0 else 0.0
        ground_ratio_lane = float(max(0.0, 1.0 - green_ratio_lane))

        # --------- Publish all metrics ---------
        # Debug image
        self._publish_debug(mask, msg)

        # Lane edges [x_L*, x_R*] at lookahead band
        lane_edges_msg = Float32MultiArray()
        lane_edges_msg.data = [float(x_L_star), float(x_R_star)]
        self.pub_lane_edges.publish(lane_edges_msg)

        # Scalars
        self.pub_lane_width_px.publish(Float32(data=float(lane_width_px)))
        self.pub_lateral_px.publish(Float32(data=float(lateral_px)))
        self.pub_lateral_m.publish(Float32(data=float(lateral_m)))
        self.pub_crop_symmetry.publish(Float32(data=float(crop_symmetry)))
        self.pub_green_ratio_lane.publish(Float32(data=float(green_ratio_lane)))
        self.pub_ground_ratio_lane.publish(Float32(data=float(ground_ratio_lane)))

        # Row errors: standardized to [lateral_norm, heading]
        # lateral_norm := crop_symmetry (unitless, comparable across lane widths)
        self._publish_row_errors(float(crop_symmetry), float(heading))

        # Throttled console log
        now = time.time()
        if now - self.last_log > 0.5:
            self.get_logger().info(
                f"y*={y_c_star} "
                f"lane_width_px={lane_width_px:.1f} "
                f"lat_px={lateral_px:.1f} lat_m={lateral_m:.3f} "
                f"sym={crop_symmetry:.3f} "
                f"green_lane={green_ratio_lane:.3f} ground_lane={ground_ratio_lane:.3f} "
                f"heading={heading:.3f}"
            )
            self.last_log = now

    # ---------- Helpers to publish ----------
    def _publish_debug(self, mask, msg):
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mask_msg.header = msg.header
            self.pub_dbg_mask.publish(mask_msg)
        except Exception:
            pass

    def _publish_metrics_none(self):
        # publish zeros when lane not found
        self.pub_lane_width_px.publish(Float32(data=0.0))
        self.pub_lateral_px.publish(Float32(data=0.0))
        self.pub_lateral_m.publish(Float32(data=0.0))
        self.pub_crop_symmetry.publish(Float32(data=0.0))
        self.pub_green_ratio_lane.publish(Float32(data=0.0))
        self.pub_ground_ratio_lane.publish(Float32(data=0.0))
        lane_edges_msg = Float32MultiArray()
        lane_edges_msg.data = [0.0, 0.0]
        self.pub_lane_edges.publish(lane_edges_msg)

    def _publish_row_errors(self, lateral_norm: float, heading: float):
        out = Float32MultiArray()
        out.data = [float(lateral_norm), float(heading)]
        self.pub_row_errors.publish(out)


def main():
    rclpy.init()
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
