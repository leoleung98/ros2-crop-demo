#!/usr/bin/env python3
import csv
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, UInt8

@dataclass
class Buf:
    green: float = 0.0
    ground: float = 0.0
    sym: float = 0.0
    lat_m: float = 0.0
    heading: float = 0.0
    status: int = 0

class Recorder(Node):
    def __init__(self):
        super().__init__('metrics_recorder')
        self.declare_parameter('csv_path', 'run_metrics.csv')
        self.declare_parameter('hz', 20.0)

        self.buf = Buf()
        self.t0 = time.time()
        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.hz = float(self.get_parameter('hz').value)
        self._opened = False

        self.create_subscription(Float32, '/perception/green_ratio_lane', self.on_green, 10)
        self.create_subscription(Float32, '/perception/ground_ratio_lane', self.on_ground, 10)
        self.create_subscription(Float32, '/perception/crop_symmetry', self.on_sym, 10)
        self.create_subscription(Float32, '/perception/lateral_m', self.on_latm, 10)
        self.create_subscription(Float32MultiArray, '/row_errors', self.on_err, 10)
        self.create_subscription(UInt8, '/perception/status', self.on_status, 10)

        self.timer = self.create_timer(1.0 / self.hz, self.tick)

    def on_green(self, msg: Float32):  self.buf.green = float(msg.data)
    def on_ground(self, msg: Float32): self.buf.ground = float(msg.data)
    def on_sym(self, msg: Float32):    self.buf.sym = float(msg.data)
    def on_latm(self, msg: Float32):   self.buf.lat_m = float(msg.data)
    def on_err(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.buf.heading = float(msg.data[1])
    def on_status(self, msg: UInt8):   self.buf.status = int(msg.data)

    def tick(self):
        t = time.time() - self.t0
        row = [f'{t:.3f}', f'{self.buf.green:.6f}', f'{self.buf.ground:.6f}',
               f'{self.buf.sym:.6f}', f'{self.buf.lat_m:.6f}',
               f'{self.buf.heading:.6f}', str(self.buf.status)]
        if not self._opened:
            self._f = open(self.csv_path, 'w', newline='')
            self._w = csv.writer(self._f)
            self._w.writerow(['t_sec','green_lane','ground_lane','crop_symmetry',
                              'lateral_m','heading','status'])
            self._opened = True
        self._w.writerow(row)
        self._f.flush()

def main():
    rclpy.init()
    node = Recorder()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
