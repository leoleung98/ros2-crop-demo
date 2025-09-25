#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String, Float32, Int32


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class RowController(Node):
    """
    Heading-error control + recovery:
      omega = k_yaw*(h_des - head_f) + k_lat*h_des - k_damp*omega_prev
      h_des = atan2(lat_norm, Ld) + I_lat
      - Anti-windup on integral
      - Confidence scales speed only (steering always active)
      - RECOVER when |lat| large or conf low; shorter lookahead + min spin
      - Online bias estimation near center to kill constant-curvature drift
    """

    def __init__(self):
        super().__init__("row_controller")

        # ------- Gains / geometry -------
        self.declare_parameter("k_yaw", 2.0)
        self.declare_parameter("k_lat", 1.0)
        self.declare_parameter("lookahead_norm", 1.0)
        self.declare_parameter("k_damp", 0.35)
        self.declare_parameter("sym_deadband", 0.01)

        # ------- Speed & limits -------
        self.declare_parameter("v_forward", 0.20)
        self.declare_parameter("v_min", 0.05)
        self.declare_parameter("confidence_gain", 1.0)
        self.declare_parameter("omega_limit", 1.50)
        self.declare_parameter("omega_acc_limit", 2.50)

        # ------- Filters & bias -------
        self.declare_parameter("lat_alpha", 0.35)
        self.declare_parameter("head_alpha", 0.45)
        self.declare_parameter("bias_tau_s", 1.2)
        self.declare_parameter("bias_enable", True)

        # ------- Recovery & integral -------
        self.declare_parameter("recover_sym_thresh", 0.28)
        self.declare_parameter("recover_conf_thresh", 0.25)
        self.declare_parameter("recover_spin_omega", 0.55)
        self.declare_parameter("recover_exit_sym", 0.05)

        self.declare_parameter("i_gain", 0.7)
        self.declare_parameter("i_limit", 0.5)
        self.declare_parameter("i_leak", 0.02)

        gp = self.get_parameter
        self.k_yaw = float(gp("k_yaw").value)
        self.k_lat = float(gp("k_lat").value)
        self.lookahead_norm = max(0.3, float(gp("lookahead_norm").value))
        self.k_damp = float(gp("k_damp").value)
        self.sym_deadband = float(gp("sym_deadband").value)

        self.v_forward = float(gp("v_forward").value)
        self.v_min = float(gp("v_min").value)
        self.conf_gain = float(gp("confidence_gain").value)
        self.omega_limit = float(gp("omega_limit").value)
        self.omega_acc_limit = float(gp("omega_acc_limit").value)

        self.lat_alpha = float(gp("lat_alpha").value)
        self.head_alpha = float(gp("head_alpha").value)
        self.bias_tau_s = max(0.2, float(gp("bias_tau_s").value))
        self.bias_enable = bool(gp("bias_enable").value)

        self.rec_sym_th = float(gp("recover_sym_thresh").value)
        self.rec_conf_th = float(gp("recover_conf_thresh").value)
        self.rec_spin = float(gp("recover_spin_omega").value)
        self.rec_exit = float(gp("recover_exit_sym").value)

        self.i_gain = float(gp("i_gain").value)
        self.i_limit = float(gp("i_limit").value)
        self.i_leak = float(gp("i_leak").value)

        # I/O
        self.sub_err = self.create_subscription(Float32MultiArray, "/row_errors", self.cb_err, 10)
        self.sub_conf = self.create_subscription(Float32, "/perception/confidence", self.cb_conf, 10)
        self.sub_level = self.create_subscription(Int32, "/perception/conf_level", self.cb_level, 10)

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_state = self.create_publisher(String, "/controller/state_str", 10)

        # States
        self.omega_prev = 0.0
        self.last_time = self.get_clock().now()
        self.last_conf = 1.0
        self.last_conf_time = self.get_clock().now()
        self.conf_level = 2

        self.lat_f = 0.0
        self.head_f = 0.0
        self.bias = 0.0
        self.i_lat = 0.0
        self.mode = "TRACK"

    # ---- confidence feeds ----
    def cb_conf(self, msg: Float32):
        self.last_conf = float(msg.data)
        self.last_conf_time = self.get_clock().now()

    def cb_level(self, msg: Int32):
        self.conf_level = int(msg.data)

    def _get_conf(self):
        dt = (self.get_clock().now() - self.last_conf_time).nanoseconds * 1e-9
        return 0.5 if dt > 0.5 else float(self.last_conf)

    # ---- main control ----
    def cb_err(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        lat_n = float(msg.data[0])   # [-0.5, 0.5]
        head  = float(msg.data[1])   # rad

        if abs(lat_n) < self.sym_deadband:
            lat_n = 0.0
        lat_n = clamp(lat_n, -0.5, 0.5)

        a_lat  = clamp(self.lat_alpha, 0.0, 1.0)
        a_head = clamp(self.head_alpha, 0.0, 1.0)
        self.lat_f  = (1.0 - a_lat)  * self.lat_f  + a_lat  * lat_n
        self.head_f = (1.0 - a_head) * self.head_f + a_head * head

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05

        conf = self._get_conf()

        # ---- TRACK <-> RECOVER ----
        if self.mode == "TRACK":
            if (abs(self.lat_f) > self.rec_sym_th) or (conf < self.rec_conf_th):
                self.mode = "RECOVER"
                self.i_lat = 0.0
        else:
            if (abs(self.lat_f) < self.rec_exit) and (conf >= self.rec_conf_th):
                self.mode = "TRACK"
                self.omega_prev *= 0.5

        # ---- desired heading from lateral ----
        Ld = max(0.5, self.lookahead_norm)
        if self.mode == "RECOVER":
            Ld = 0.6 * Ld
        h_lat = math.atan2(self.lat_f, Ld)

        # ---- integral with anti-windup ----
        i_before = self.i_lat
        h_des_tmp = h_lat + i_before
        head_err_tmp = h_des_tmp - self.head_f
        omega_tmp = self.k_yaw * head_err_tmp + self.k_lat * h_des_tmp - self.k_damp * self.omega_prev
        saturated = abs(omega_tmp) > self.omega_limit * 0.98

        if (not saturated) or (saturated and (omega_tmp * self.lat_f < 0.0)):
            self.i_lat += self.i_gain * self.lat_f * dt
            self.i_lat = clamp(self.i_lat, -self.i_limit, self.i_limit)
        self.i_lat = (1.0 - self.i_leak * dt) * self.i_lat

        h_des = h_lat + self.i_lat
        head_err = h_des - self.head_f

        omega_cmd = self.k_yaw * head_err + self.k_lat * h_des - self.k_damp * self.omega_prev

        if self.mode == "RECOVER":
            sgn = 1.0 if (self.lat_f > 0) else (-1.0 if self.lat_f < 0 else 0.0)
            omega_cmd = sgn * max(abs(omega_cmd), self.rec_spin)

        if self.bias_enable and abs(self.lat_f) < 0.03 and abs(self.head_f) < 0.03 and conf > 0.6:
            alpha = clamp(dt / self.bias_tau_s, 0.0, 1.0)
            self.bias = (1.0 - alpha) * self.bias + alpha * omega_cmd
        omega_cmd -= self.bias

        omega_cmd = clamp(omega_cmd, -self.omega_limit, self.omega_limit)
        max_step = self.omega_acc_limit * dt
        omega_cmd = clamp(omega_cmd, self.omega_prev - max_step, self.omega_prev + max_step)
        self.omega_prev = omega_cmd

        scale = (0.4 + 0.6 * conf) ** max(0.0, self.conf_gain)
        v = max(self.v_min, self.v_forward * scale)
        if self.mode == "RECOVER":
            v = max(self.v_min, min(v, 0.6 * self.v_forward))

        tw = Twist()
        tw.linear.x = v
        tw.angular.z = omega_cmd
        self.pub_cmd.publish(tw)
        self.pub_state.publish(String(
            data=f"mode={self.mode} v={v:.3f} omega={omega_cmd:.3f} "
                 f"lat={self.lat_f:.3f} head={self.head_f:.3f} conf={conf:.2f} "
                 f"h_des={h_des:.3f} herr={head_err:.3f} i_lat={self.i_lat:.3f} bias={self.bias:.3f}"
        ))


def main(args=None):
    rclpy.init(args=args)
    node = RowController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
