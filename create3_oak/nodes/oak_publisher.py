import argparse
import sys

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
import depthai as dai


FPS = 15                       # Bildrate von Kamera und neuronalem Netz
MONO_RESOLUTION = (640, 400)   # Auflösung der Mono-/Tiefenbilder (Breite, Höhe)
FRAME_ID = "oak_camera"        # ROS-Frame, in dem die Positionen angegeben werden
DEFAULT_MODEL = "yolov6-nano"  # Objekterkennungs-Modell aus dem DepthAI-Model-Zoo
DEPTH_MIN_MM = 100             # naeher als 0,1 m gemessene Objekte ignorieren
DEPTH_MAX_MM = 5000            # weiter als 5 m entfernte Objekte ignorieren


def camera_is_available(logger):
    """Prüft vor dem Pipeline-Start, ob eine OAK-/DepthAI-Kamera erreichbar ist.

    Gibt True zurück, wenn mindestens ein Gerät gefunden wird. Dadurch bricht
    der Node mit einer klaren Meldung ab, statt in eine Reconnect-Schleife zu
    laufen, falls die Kamera (z. B. nach einem vorherigen Lauf) nicht sauber
    bereitsteht.
    """
    try:
        devices = dai.Device.getAllAvailableDevices()
    except Exception as exc:
        logger.error(f"Kamera-Check fehlgeschlagen: {exc}")
        return False

    if not devices:
        logger.error("Keine OAK-D/DepthAI-Kamera erreichbar. USB prüfen oder Kamera kurz ab-/anstecken.")
        return False

    logger.info("OAK-D/DepthAI-Kamera erreichbar: " + ", ".join(str(device) for device in devices))
    return True


def parse_cli_args(argv):
    """Liest die eigenen Argumente (--model) aus und reicht ROS-Argumente weiter.

    Rückgabe: (eigene_argumente, ros_argumente). Alles, was argparse nicht kennt
    (z. B. '--ros-args -p model:=...'), landet unverändert in ros_argumente.
    """
    parser = argparse.ArgumentParser(description="Publiziert OAK-D-Lite-Detektionen als ROS-2-Topic.")
    parser.add_argument("--model", default=DEFAULT_MODEL, help=f"DepthAI-Model-Zoo-Name, z. B. yolov6-nano. Default: {DEFAULT_MODEL}",)
    own_args, ros_args = parser.parse_known_args(argv)

    return own_args, ros_args


def build_spatial_network(pipeline, cam_color, stereo, model_name, logger):
    """Lädt das gewählte Detektions-Modell und baut das Spatial-Detection-Netz.

    Das SpatialDetectionNetwork verbindet die Objekterkennung (auf dem Farbbild)
    mit der Stereo-Tiefe und liefert pro Objekt Klasse, Konfidenz und 3D-Position.
    Gibt None zurück, wenn der Modellname kein gueltiger Model-Zoo-Slug ist.
    """
    try:
        model = dai.NNModelDescription(model_name)
        return pipeline.create(dai.node.SpatialDetectionNetwork).build(cam_color, stereo, model)
    
    except RuntimeError as exc:
        logger.error(f"DepthAI-Modell konnte nicht geladen werden: {model_name}")
        logger.error("Der Name muss ein gültiger oeffentlicher DepthAI-Model-Zoo-Slug sein.")
        logger.error(f"Getesteter Standardwert: {DEFAULT_MODEL}")
        logger.error(f"Originalfehler: {exc}")
        return None


def detections_to_ros_message(spatial_detections, stamp):
    """Wandelt die DepthAI-Detektionen in eine ROS-Nachricht (Detection3DArray) um.

    Pro erkanntem Objekt werden Klasse, Konfidenz und 3D-Position übernommen.
    DepthAI liefert die Position in Millimetern, ROS erwartet Meter (-> / 1000).
    """
    message = Detection3DArray()    # Spezielles ros Array/Nachrichtentyp für 3D Detections
    message.header.stamp = stamp
    message.header.frame_id = FRAME_ID

    for detection in spatial_detections:
        ros_detection = Detection3D()
        ros_detection.header = message.header

        # Klasse + Konfidenz
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(detection.labelName)   # z. B. "person"
        hypothesis.hypothesis.score = float(detection.confidence)

        # 3D-Position: Millimeter (DepthAI) -> Meter (ROS)
        hypothesis.pose.pose.position.x = detection.spatialCoordinates.x / 1000.0
        hypothesis.pose.pose.position.y = detection.spatialCoordinates.y / 1000.0
        hypothesis.pose.pose.position.z = detection.spatialCoordinates.z / 1000.0

        ros_detection.results.append(hypothesis)
        message.detections.append(ros_detection)

    return message


def main():
    # Eigene Argumente von ROS-Argumenten trennen und ROS initialisieren
    own_args, ros_args = parse_cli_args(sys.argv[1:])
    rclpy.init(args=ros_args)

    node = Node("oak_spatial_publisher")
    publisher = node.create_publisher(Detection3DArray, "/oak/nn/spatial_detections", 10)

    # Modellname zusätzlich als ROS-Parameter verfügbar machen (per --ros-args
    # überschreibbar); Default ist yolov6-nano
    node.declare_parameter("model", own_args.model)
    model_name = node.get_parameter("model").value

    try:
        # 1) Ist die Kamera überhaupt erreichbar? Sonst abbrechen
        if not camera_is_available(node.get_logger()):
            return 1
        node.get_logger().info(f"Verwende DepthAI-Modell: {model_name}")

        # 2) DepthAI-Pipeline aufbauen
        with dai.Pipeline() as pipeline:
            # Farbkamera (CAM_A): liefert das Bild für das neuronale Netz
            cam_color = pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_A, sensorFps=FPS)
            # Mono-Kameras (CAM_B/CAM_C): liefern das Stereo-Paar für die Tiefe
            mono_left = pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_B, sensorFps=FPS)
            mono_right = pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_C, sensorFps=FPS)

            # Stereo-Tiefe aus dem Mono-Paar berechnen.
            stereo = pipeline.create(dai.node.StereoDepth)
            stereo.setExtendedDisparity(True)  # bessere Tiefe im Nahbereich
            mono_left.requestOutput(MONO_RESOLUTION).link(stereo.left)
            mono_right.requestOutput(MONO_RESOLUTION).link(stereo.right)

            # Objekterkennung + Tiefe kombinieren
            spatial_network = build_spatial_network(
                pipeline, cam_color, stereo, model_name, node.get_logger())
            if spatial_network is None:
                return 2
            spatial_network.input.setBlocking(False)
            spatial_network.setDepthLowerThreshold(DEPTH_MIN_MM)
            spatial_network.setDepthUpperThreshold(DEPTH_MAX_MM)

            # Ausgabe-Warteschlange der Detektionen (Host-Seite)
            detection_queue = spatial_network.out.createOutputQueue()
            pipeline.start()
            node.get_logger().info("Pipeline läuft, publiziere /oak/nn/spatial_detections")

            # 3) Hauptschleife: Detektionen abholen, umwandeln, veröffentlichen
            while pipeline.isRunning() and rclpy.ok():
                spatial_detections = detection_queue.get() # Detection wird geholt
                stamp = node.get_clock().now().to_msg()
                message = detections_to_ros_message(spatial_detections.detections, stamp)   # In ROS Format umwandeln
                publisher.publish(message)  # und auf /oak/nn/spatial_detections veröffentlichen
                if message.detections:
                    node.get_logger().info(f"{len(message.detections)} Objekt(e) publiziert")

    except KeyboardInterrupt:
        # Strg+C: Schleife verlassen und aufräumen
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())