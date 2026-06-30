import math
import sys
from collections import namedtuple

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import Twist

# Verwendung eines namedTupels zum einfacheren arbeiten im Code
Target = namedtuple("Target", ["x", "z", "label", "score"])
# x, z  : Position relativ zur Kamera in Metern (x = seitlich, z = nach vorne)
# label : Klassenname, z. B. "person"
# score : Konfidenz 0..1


class DriveToObject(Node):
    def __init__(self, target_class_override=None):
        super().__init__("drive_to_object")

        # Parameter mit getesteten Defaults 
        # Alle Werte sind beim Start über ROS-Parameter überschreibbar,
        # z. B.:  --ros-args -p stop_distance:=0.7 -p k_ang:=0.9
        self.declare_parameter("dry_run", True) # True = nicht fahren, nur loggen; wurde zum testen verwendet damit der Roboter nicht unkontrolliert los fährt
        self.declare_parameter("target_class", "")  # "" = beliebiges Objekt, sonst z. B. "person"
        self.declare_parameter("min_confidence", 0.3)   # Detektionen darunter ignorieren
        self.declare_parameter("stop_distance", 0.5)    # m - Zielabstand vor dem Objekt
        self.declare_parameter("distance_tolerance", 0.05)  # m - Totbereich um den Zielabstand
        self.declare_parameter("max_linear", 0.15)  # m/s - maximale Vorwärtsgeschwindigkeit
        self.declare_parameter("max_angular", 0.6)  # rad/s - maximale Drehgeschwindigkeit
        self.declare_parameter("k_lin", 0.4)    # Reglerverstärkung vorwärts (k_lin in der Formel)
        self.declare_parameter("k_ang", 1.2)    # Reglerverstärkung Drehung   (k_ang in der Formel)
        self.declare_parameter("align_threshold", 0.35) # rad (~20 Grad) - erst ausrichten, dann fahren
        self.declare_parameter("angular_deadband", 0.05)    # rad - darunter keine Drehung (gegen Zittern)
        self.declare_parameter("detection_timeout", 0.5)    # s - ohne Detektion -> Stopp
        self.declare_parameter("control_rate", 15.0)    # Hz - Takt der Regelschleife

        # Kleine Hilfsfunktion, damit das Einlesen der Parameter gut lesbar bleibt.
        def param(name):
            return self.get_parameter(name).value

        self.dry_run = param("dry_run")
        self.target_class = param("target_class")
        # Eine über die Kommandozeile angegebene Zielklasse hat Vorrang.
        if target_class_override is not None:
            self.target_class = target_class_override
        self.min_confidence = param("min_confidence")
        self.stop_distance = param("stop_distance")
        self.distance_tolerance = param("distance_tolerance")
        self.max_linear = param("max_linear")
        self.max_angular = param("max_angular")
        self.k_lin = param("k_lin")
        self.k_ang = param("k_ang")
        self.align_threshold = param("align_threshold")
        self.angular_deadband = param("angular_deadband")
        self.detection_timeout = param("detection_timeout")
        control_rate = param("control_rate")

        # Zuletzt erkanntes Zielobjekt und Zeitpunkt der letzten Detektion
        # (für die Stopp-bei-Zielverlust-Logik).
        self.last_target = None
        self.last_detection_time = self.get_clock().now()

        # ROS-Schnittstellen: Detektionen abonnieren, Fahrbefehle publizieren.
        self.subscription = self.create_subscription(
            Detection3DArray, "/oak/nn/spatial_detections", self.on_detections, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Die Regelschleife läuft zeitgesteuert (nicht nur bei Empfang), damit
        # kontinuierlich /cmd_vel kommt und der Stopp bei Zielverlust sicher greift.
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)

        mode = "DRY-RUN (fährt NICHT)" if self.dry_run else "AKTIV (fährt!)"
        self.get_logger().info(f"drive_to_object gestartet - Modus: {mode}, "
                               f"Zielklasse: {self.target_class or 'beliebig'}")

    def on_detections(self, message):
        """Callback für /oak/nn/spatial_detections.

        Wählt aus allen Detektionen das nächstgelegene gültige Zielobjekt:
        ausreichende Konfidenz, passende Zielklasse (falls gesetzt) und gültige
        Tiefe (z > 0). Gespeichert wird der beste Treffer mit Zeitstempel.
        """
        nearest = None
        for detection in message.detections:
            if not detection.results:
                continue
            hypothesis = detection.results[0].hypothesis
            if hypothesis.score < self.min_confidence:                 # zu unsicher
                continue
            if self.target_class and hypothesis.class_id != self.target_class:  # falsche Klasse
                continue
            position = detection.results[0].pose.pose.position
            if position.z <= 0.0:                                      # keine gültige Tiefe
                continue
            # "Nächstes" Objekt = kleinster Z-Wert (Entfernung nach vorne).
            if nearest is None or position.z < nearest.z:
                nearest = Target(x=position.x, z=position.z, label=hypothesis.class_id, score=hypothesis.score)

        self.last_target = nearest
        self.last_detection_time = self.get_clock().now()

    def _compute_velocity(self, x, z):
        """Kern der Regelung: berechnet aus der Objektposition (x, z) die
        Fahrbefehle (lineare Geschwindigkeit, Drehgeschwindigkeit).

          heading = atan2(x, z)   Winkel zum Objekt; > 0 bedeutet "Objekt rechts".

          Drehung:
            angular = -k_ang * heading
            Vorzeichen: Objekt rechts (heading > 0) -> angular < 0 -> Rechtsdrehung
            Innerhalb eines kleinen Totbereichs (angular_deadband) wird nicht
            gedreht; danach auf max_angular begrenzt.

          Vorwärts:
            linear = k_lin * (z - stop_distance)
            Nur, wenn der Roboter grob zum Objekt ausgerichtet ist
            (|heading| < align_threshold) und noch weiter weg als der Zielabstand.
            Auf [0, max_linear] begrenzt (kein Rückwärts).
        """
        heading = math.atan2(x, z)
        distance_error = z - self.stop_distance

        # Drehgeschwindigkeit
        angular_velocity = -self.k_ang * heading
        if abs(heading) < self.angular_deadband:
            angular_velocity = 0.0
        angular_velocity = max(-self.max_angular, min(self.max_angular, angular_velocity))

        # Vorwärtsgeschwindigkeit (erst ausrichten, dann fahren)
        linear_velocity = 0.0
        if abs(heading) < self.align_threshold and distance_error > self.distance_tolerance:
            linear_velocity = max(0.0, min(self.max_linear, self.k_lin * distance_error))

        return linear_velocity, angular_velocity

    def control_loop(self):
        """Regelschleife (läuft mit control_rate).

        Ist kein Ziel vorhanden oder die letzte Detektion älter als
        detection_timeout, wird ein Stopp gesendet. So hält der Roboter an,
        wenn das Objekt verschwindet oder der Kamera-Node ausfällt.
        """
        twist = Twist()
        seconds_since_detection = (
            self.get_clock().now() - self.last_detection_time).nanoseconds * 1e-9
        target = self.last_target

        if target is None or seconds_since_detection > self.detection_timeout:
            self.publish(twist, "kein aktuelles Ziel -> Stopp")
            return

        linear_velocity, angular_velocity = self._compute_velocity(target.x, target.z)
        twist.linear.x = float(linear_velocity)
        twist.angular.z = float(angular_velocity)
        self.publish(twist,
                     f"{target.label} {target.score * 100:.0f}%  "
                     f"x={target.x:+.2f} z={target.z:.2f}  "
                     f"-> v={linear_velocity:.2f} w={angular_velocity:+.2f}")

    def publish(self, twist, reason):
        """Sendet den Fahrbefehl - oder loggt ihn im Dry-Run nur, ohne zu senden."""
        if self.dry_run:
            self.get_logger().info(f"[DRY] {reason}")
        else:
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info(reason)

    def stop(self):
        """Beim Beenden den Roboter sicher anhalten (Null-Twist).

        Im Dry-Run wird bewusst gar nichts gesendet, damit /cmd_vel dort wirklich
        unberührt bleibt.
        """
        if self.dry_run:
            return
        try:
            self.cmd_vel_publisher.publish(Twist())
        except Exception:
            pass


def parse_cli_args(argv):
    """Trennt eine optionale Zielklasse von den ROS-Argumenten.

    Erlaubt mehrere bequeme Schreibweisen für die Zielklasse:
      drive_to_object.py person
      drive_to_object.py --target person
      drive_to_object.py --target-class person
      drive_to_object.py --target=person
    Alles ab "--ros-args" wird unverändert an rclpy weitergereicht
    (z. B. -p dry_run:=false). Rückgabe: (zielklasse_oder_None, ros_argumente).
    """
    target_class = None
    ros_args = []
    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg == "--ros-args":
            # Ab hier gehört alles ROS; unverändert weiterreichen.
            ros_args.extend(argv[i:])
            break
        if arg in ("--target", "--target-class"):
            if i + 1 >= len(argv):
                raise SystemExit(f"{arg} benoetigt einen Klassennamen, z. B. {arg} person")
            target_class = argv[i + 1]
            i += 2
            continue
        if arg.startswith("--target="):
            target_class = arg.split("=", 1)[1]
            i += 1
            continue
        if arg.startswith("--target-class="):
            target_class = arg.split("=", 1)[1]
            i += 1
            continue
        # Erstes "freies" Argument (ohne fuehrendes '-') ist die Zielklasse.
        if not arg.startswith("-") and target_class is None:
            target_class = arg
            i += 1
            continue
        ros_args.append(arg)
        i += 1

    if target_class == "":
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


if __name__ == "__main__":
    main()