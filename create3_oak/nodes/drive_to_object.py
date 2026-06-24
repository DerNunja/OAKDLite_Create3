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

        # --- Parameter (mit sicheren Defaults) -------------------------------
        # Alle Werte sind beim Start über ROS-Parameter überschreibbar,
        # z. B.:  --ros-args -p stop_distance:=0.7 -p k_ang:=0.9
        self.declare_parameter('dry_run', True)            # True = nicht fahren, nur loggen
        self.declare_parameter('target_class', '')         # '' = beliebiges Objekt, sonst z. B. 'person'
        self.declare_parameter('min_confidence', 0.4)      # Detektionen darunter ignorieren
        self.declare_parameter('stop_distance', 0.5)       # m  - gewuenschter Abstand vor dem Objekt
        self.declare_parameter('distance_tolerance', 0.05) # m  - Totbereich um den Zielabstand
        self.declare_parameter('max_linear', 0.15)         # m/s - maximale Vorwaertsgeschwindigkeit
        self.declare_parameter('max_angular', 0.6)         # rad/s - maximale Drehgeschwindigkeit
        self.declare_parameter('k_lin', 0.4)               # Verstaerkung Vorwaerts (P-Regler)
        self.declare_parameter('k_ang', 1.2)               # Verstaerkung Drehung  (P-Regler)
        self.declare_parameter('align_threshold', 0.35)    # rad (~20 deg) - erst ausrichten, dann fahren
        self.declare_parameter('angular_deadband', 0.05)   # rad - darunter keine Drehung (gegen Zittern)
        self.declare_parameter('detection_timeout', 0.5)   # s  - ohne Detektion -> Stopp
        self.declare_parameter('control_rate', 15.0)       # Hz - Takt der Regelschleife

        # Parameterwerte einlesen
        gp = self.get_parameter
        self.dry_run = gp('dry_run').value
        self.tgt_class = gp('target_class').value
        # Eine ueber die Kommandozeile angegebene Zielklasse hat Vorrang.
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

        # zuletzt gesehenes objekt: (x, z, label, score) und empfangszeitpunkt.
        self.last_target = None
        self.last_time = self.get_clock().now()

        # ROS-Schnittstellen: detektionen abonnieren, fahrbefehle publizieren.
        self.sub = self.create_subscription(
            Detection3DArray, '/oak/nn/spatial_detections', self.on_detections, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # regelschleife läuft zeitgesteuert (nicht nur bei Empfang), damit
        # kontinuierlich /cmd_vel kommt und der Stopp bei Zielverlust sicher greift
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        mode = 'DRY-RUN (fährt NICHT)' if self.dry_run else 'AKTIV (fährt!)'
        self.get_logger().info(f'drive_to_object gestartet - Modus: {mode}, '
                               f'Zielklasse: {self.tgt_class or "beliebig"}')

    def on_detections(self, msg):
        """Callback für /oak/nn/spatial_detections.

        Wählt aus allen Detektionen das nächstgelegene gültige Zielobjekt aus:
        ausreichende Konfidenz, passende Zielklasse (falls gesetzt) und gültige
        Tiefe (z > 0). Gespeichert wird der beste Treffer mit Zeitstempel.
        """
        best = None
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            if hyp.score < self.min_conf:                       # zu unsicher
                continue
            if self.tgt_class and hyp.class_id != self.tgt_class:  # falsche Klasse
                continue
            p = det.results[0].pose.pose.position
            if p.z <= 0.0:                                      # keine gueltige Tiefe
                continue
            # "Naechstes" Objekt = kleinster Z-Wert.
            if best is None or p.z < best[1]:
                best = (p.x, p.z, hyp.class_id, hyp.score)
        self.last_target = best
        self.last_time = self.get_clock().now()

    def _compute_velocity(self, x, z):
        """Kern der Regelung: berechnet aus der Objektposition (x, z) die
        Fahrbefehle (lineare Geschwindigkeit lin, Drehgeschwindigkeit ang).

          heading = atan2(x, z)   Winkel zum Objekt; > 0 bedeutet "Objekt rechts".

          Drehung:
            ang = -k_ang * heading
            Vorzeichen: Objekt rechts (heading > 0) -> ang < 0 -> Rechtsdrehung
            (ROS-Konvention: positives angular.z = Drehung nach links).
            Innerhalb eines kleinen Totbereichs (angular_deadband) wird nicht
            gedreht, danach auf max_angular begrenzt.

          Vorwaerts:
            lin = k_lin * (z - stop_distance)
            Nur, wenn der Roboter grob zum Objekt ausgerichtet ist
            (|heading| < align_threshold) und noch weiter weg als der Zielabstand.
            Auf [0, max_linear] begrenzt (kein Rueckwaerts).
        """
        heading = math.atan2(x, z)
        err_z = z - self.stop_d

        # Drehgeschwindigkeit
        ang = -self.k_ang * heading
        if abs(heading) < self.ang_db:
            ang = 0.0
        ang = max(-self.max_ang, min(self.max_ang, ang))

        # Vorwärtsgeschwindigkeit (erst ausrichten, dann fahren)
        lin = 0.0
        if abs(heading) < self.align_th and err_z > self.dist_tol:
            lin = max(0.0, min(self.max_lin, self.k_lin * err_z))

        return lin, ang

    def control_loop(self):
        """Regelschleife (läuft mit control_rate).

        Watchdog: Ist kein Ziel vorhanden oder die letzte Detektion älter als
        detection_timeout, wird ein Stopp gesendet. So hält der Roboter an,
        wenn das Objekt verschwindet oder der Kamera-Node ausfällt.
        """
        twist = Twist()
        age = (self.get_clock().now() - self.last_time).nanoseconds * 1e-9
        target = self.last_target

        if target is None or age > self.timeout:
            self.publish(twist, 'kein aktuelles Ziel -> Stopp')
            return

        x, z, label, score = target
        lin, ang = self._compute_velocity(x, z)
        twist.linear.x = float(lin)
        twist.angular.z = float(ang)
        self.publish(twist,
                     f'{label} {score * 100:.0f}%  x={x:+.2f} z={z:.2f}  '
                     f'-> v={lin:.2f} w={ang:+.2f}')

    def publish(self, twist, reason):
        """Sendet den Fahrbefehl - oder loggt ihn im Dry-Run nur, ohne zu senden."""
        if self.dry_run:
            self.get_logger().info(f'[DRY] {reason}')
        else:
            self.pub.publish(twist)
            self.get_logger().info(reason)

    def stop(self):
        """Beim Beenden den Roboter sicher anhalten (Null-Twist).

        Im Dry-Run wird bewusst gar nichts gesendet, damit /cmd_vel dort wirklich
        unberührt bleibt.
        """
        if self.dry_run:
            return
        try:
            self.pub.publish(Twist())
        except Exception:
            pass


def parse_cli_args(argv):
    """Trennt eine optionale Zielklasse von den ROS-Argumenten.

    Erlaubt mehrere bequeme Schreibweisen fuer die Zielklasse:
      drive_to_object.py person
      drive_to_object.py --target person
      drive_to_object.py --target-class person
      drive_to_object.py --target=person
    Alles ab '--ros-args' wird unveraendert an rclpy weitergereicht (z. B. -p dry_run:=false).
    Rueckgabe: (zielklasse_oder_None, ros_argumente_liste).
    """
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
                raise SystemExit(f'{arg} benoetigt einen Klassennamen, z. B. {arg} person')
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
        # Erstes "freies" Argument (ohne '-') wird als Zielklasse interpretiert.
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
        # Sauberes Beenden per Strg+C bzw. externem Shutdown.
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()