#!/usr/bin/env python3
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import Twist


class DriveToObject(Node):
    def __init__(self, target_class_override=None):
        super().__init__('drive_to_object')

        # --- Parameter mit sicheren Defaults ---
        self.declare_parameter('dry_run', True)            # True = faehrt NICHT, nur Log
        self.declare_parameter('target_class', '')         # '' = beliebiges Objekt, sonst z.B. 'person'
        self.declare_parameter('min_confidence', 0.4)
        self.declare_parameter('stop_distance', 0.5)       # m: Zielabstand vor dem Objekt
        self.declare_parameter('distance_tolerance', 0.05) # m
        self.declare_parameter('max_linear', 0.15)         # m/s
        self.declare_parameter('max_angular', 0.6)         # rad/s
        self.declare_parameter('k_lin', 0.4)               # P-Verstaerkung vorwaerts
        self.declare_parameter('k_ang', 1.2)               # P-Verstaerkung Drehung
        self.declare_parameter('align_threshold', 0.35)    # rad (~20 Grad): erst ausrichten, dann fahren
        self.declare_parameter('angular_deadband', 0.05)   # rad: darunter keine Drehung
        self.declare_parameter('detection_timeout', 0.5)   # s: ohne Detektion -> Stopp
        self.declare_parameter('control_rate', 15.0)       # Hz

        gp = self.get_parameter
        self.dry_run = gp('dry_run').value
        self.tgt_class = gp('target_class').value
        if target_class_override is not None:
            self.tgt_class = target_class_override
        self.min_conf = gp('min_confidence').value
        self.stop_d = gp('stop_distance').value
        self.dist_tol = gp('distance_tolerance').value
        self.max_lin = gp('max_linear').value
        self.max_ang = gp('max_angular').value
        self.k_lin = gp('k_lin').value
        self.k_ang = gp('k_ang').value
        self.align_th = gp('align_threshold').value
        self.ang_db = gp('angular_deadband').value
        self.timeout = gp('detection_timeout').value
        rate = gp('control_rate').value

        self.last_target = None                 # (x, z, label, score)
        self.last_time = self.get_clock().now()

        self.sub = self.create_subscription(
            Detection3DArray, '/oak/nn/spatial_detections', self.on_detections, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        mode = 'DRY-RUN (faehrt NICHT)' if self.dry_run else 'AKTIV (faehrt!)'
        self.get_logger().info(f'drive_to_object gestartet - Modus: {mode}, '
                               f'Ziel-Klasse: {self.tgt_class or "beliebig"}')

    def on_detections(self, msg):
        best = None
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            if hyp.score < self.min_conf:
                continue
            if self.tgt_class and hyp.class_id != self.tgt_class:
                continue
            p = det.results[0].pose.pose.position
            if p.z <= 0.0:
                continue
            if best is None or p.z < best[1]:
                best = (p.x, p.z, hyp.class_id, hyp.score)
        self.last_target = best
        self.last_time = self.get_clock().now()

    def control_loop(self):
        twist = Twist()
        age = (self.get_clock().now() - self.last_time).nanoseconds * 1e-9
        target = self.last_target

        if target is None or age > self.timeout:
            self.publish(twist, 'kein aktuelles Ziel -> Stopp')
            return

        x, z, label, score = target
        heading = math.atan2(x, z)      # >0 = Ziel rechts
        err_z = z - self.stop_d

        # Drehung (Ziel rechts -> rechts drehen -> angular.z negativ)
        ang = -self.k_ang * heading
        if abs(heading) < self.ang_db:
            ang = 0.0
        ang = max(-self.max_ang, min(self.max_ang, ang))

        # Vorwaerts nur wenn grob ausgerichtet und noch zu weit weg
        lin = 0.0
        if abs(heading) < self.align_th and err_z > self.dist_tol:
            lin = max(0.0, min(self.max_lin, self.k_lin * err_z))

        twist.linear.x = float(lin)
        twist.angular.z = float(ang)
        self.publish(twist,
                     f'{label} {score*100:.0f}%  x={x:+.2f} z={z:.2f}  '
                     f'-> v={lin:.2f} w={ang:+.2f}')

    def publish(self, twist, reason):
        if self.dry_run:
            self.get_logger().info(f'[DRY] {reason}')
        else:
            self.pub.publish(twist)
            self.get_logger().info(reason)

    def stop(self):
        # Im Dry-Run garantiert nichts auf cmd_vel publizieren.
        if self.dry_run:
            return
        try:
            self.pub.publish(Twist())
        except Exception:
            pass


def parse_cli_args(argv):
    target_class = None
    ros_args = []
    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg == '--ros-args':
            ros_args.extend(argv[i:])
            break
        if arg in ('--target', '--target-class'):
            if i + 1 >= len(argv):
                raise SystemExit(f'{arg} braucht einen Klassen-Namen, z.B. {arg} person')
            target_class = argv[i + 1]
            i += 2
            continue
        if arg.startswith('--target='):
            target_class = arg.split('=', 1)[1]
            i += 1
            continue
        if arg.startswith('--target-class='):
            target_class = arg.split('=', 1)[1]
            i += 1
            continue
        if not arg.startswith('-') and target_class is None:
            target_class = arg
            i += 1
            continue
        ros_args.append(arg)
        i += 1

    if target_class == '':
        target_class = None
    return target_class, ros_args


def main():
    target_class, ros_args = parse_cli_args(sys.argv[1:])
    rclpy.init(args=ros_args)
    node = DriveToObject(target_class_override=target_class)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
