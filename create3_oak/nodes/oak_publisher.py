import sys

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
import depthai as dai

FPS = 15                 # Bildrate von Kamera und Netz
SIZE = (640, 400)        # Auflösung der Mono-/Tiefenbilder (Breite, Hoehe)
FRAME_ID = "oak_camera"  # ROS-Frame, in dem die Positionen angegeben werden


def camera_is_available(logger):
    """Prueft vor dem Pipeline-Start, ob eine OAK-/DepthAI-Kamera erreichbar ist.

    Gibt True zurueck, wenn mindestens ein Geraet gefunden wird. So bricht der
    Node mit klarer Meldung ab, statt in eine Reconnect-Schleife zu laufen, falls
    die Kamera (z. B. nach einem vorherigen Lauf) nicht sauber bereitsteht.
    """
    try:
        devices = dai.Device.getAllAvailableDevices()
    except Exception as exc:
        logger.error(f"Kamera-Check fehlgeschlagen: {exc}")
        return False

    if not devices:
        logger.error("Keine OAK-D/DepthAI-Kamera erreichbar. USB prüfen oder Kamera kurz ab-/anstecken.")
        return False

    logger.info("OAK-D/DepthAI-Kamera erreichbar: " + ", ".join(str(d) for d in devices))
    return True


def main():
    rclpy.init()
    node = Node("oak_spatial_publisher")
    pub = node.create_publisher(Detection3DArray, "/oak/nn/spatial_detections", 10)

    try:
        if not camera_is_available(node.get_logger()): #als erstes wird geprüft ob die kamera vorhanden ist
            return 1

        model = dai.NNModelDescription("yolov6-nano") #objekt erkennungsmodell laden aus depthai

        with dai.Pipeline() as p: # pipeline
            camRgb = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=FPS) #farbbild für das objekterkennungsmodell
            # Mono-Kameras (CAM_B/CAM_C): liefern werden für die tiefen berechnung verwendet
            monoLeft = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=FPS)
            monoRight = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=FPS)

            stereo = p.create(dai.node.StereoDepth) #berechnet stereo-tiefe aus dem mono paar
            stereo.setExtendedDisparity(True)  # bessere Tiefe im Nahbereich
            monoLeft.requestOutput(SIZE).link(stereo.left)
            monoRight.requestOutput(SIZE).link(stereo.right)

            spatial = p.create(dai.node.SpatialDetectionNetwork).build(camRgb, stereo, model) #objekterkennung und tiefe werden kombiniert
            spatial.input.setBlocking(False)
            spatial.setDepthLowerThreshold(100)   # in mm - näher wird ignoriert
            spatial.setDepthUpperThreshold(5000)  # in mm - weiter wird ignoriert
            # liefert pro objekt klasse, konfidenz und 3D-position.

            # Ausgabewarteschlange fuer die Detektionen (Host-Seite).
            qDet = spatial.out.createOutputQueue()
            p.start()
            node.get_logger().info("Pipeline laeuft, publiziere /oak/nn/spatial_detections")

            # schleife wandelt detektion in ros topic um
            while p.isRunning() and rclpy.ok():
                inDet = qDet.get()
                msg = Detection3DArray()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = FRAME_ID

                for d in inDet.detections:
                    det = Detection3D()
                    det.header = msg.header
                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = str(d.labelName)   # z. B. "person"
                    hyp.hypothesis.score = float(d.confidence)
                    # detpai liefert mm -> ros braucht m.
                    hyp.pose.pose.position.x = d.spatialCoordinates.x / 1000.0
                    hyp.pose.pose.position.y = d.spatialCoordinates.y / 1000.0
                    hyp.pose.pose.position.z = d.spatialCoordinates.z / 1000.0
                    det.results.append(hyp)
                    msg.detections.append(det)

                pub.publish(msg)
                if msg.detections:
                    node.get_logger().info(f"{len(msg.detections)} Objekt(e) publiziert")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())