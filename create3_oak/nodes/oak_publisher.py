#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
import depthai as dai

FPS = 15
SIZE = (640, 400)
FRAME_ID = "oak_camera"


def camera_is_available(logger):
    try:
        devices = dai.Device.getAllAvailableDevices()
    except Exception as exc:
        logger.error(f"Camera check failed: {exc}")
        return False

    if not devices:
        logger.error("No OAK-D/DepthAI camera reachable. Check USB or unplug/replug the camera.")
        return False

    logger.info("OAK-D/DepthAI camera reachable: " + ", ".join(str(d) for d in devices))
    return True


def main():
    rclpy.init()
    node = Node("oak_spatial_publisher")
    pub = node.create_publisher(Detection3DArray, "/oak/nn/spatial_detections", 10)

    try:
        if not camera_is_available(node.get_logger()):
            return 1

        model = dai.NNModelDescription("yolov6-nano")
        with dai.Pipeline() as p:
            camRgb = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=FPS)
            monoLeft = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=FPS)
            monoRight = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=FPS)

            stereo = p.create(dai.node.StereoDepth)
            stereo.setExtendedDisparity(True)
            monoLeft.requestOutput(SIZE).link(stereo.left)
            monoRight.requestOutput(SIZE).link(stereo.right)

            spatial = p.create(dai.node.SpatialDetectionNetwork).build(camRgb, stereo, model)
            spatial.input.setBlocking(False)
            spatial.setDepthLowerThreshold(100)
            spatial.setDepthUpperThreshold(5000)

            qDet = spatial.out.createOutputQueue()
            p.start()
            node.get_logger().info("Pipeline running, publishing /oak/nn/spatial_detections")

            while p.isRunning() and rclpy.ok():
                inDet = qDet.get()
                msg = Detection3DArray()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = FRAME_ID
                for d in inDet.detections:
                    det = Detection3D()
                    det.header = msg.header
                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = str(d.labelName)
                    hyp.hypothesis.score = float(d.confidence)
                    # DepthAI provides millimeters; ROS convention here is meters.
                    hyp.pose.pose.position.x = d.spatialCoordinates.x / 1000.0
                    hyp.pose.pose.position.y = d.spatialCoordinates.y / 1000.0
                    hyp.pose.pose.position.z = d.spatialCoordinates.z / 1000.0
                    det.results.append(hyp)
                    msg.detections.append(det)
                pub.publish(msg)
                if msg.detections:
                    node.get_logger().info(f"published {len(msg.detections)} object(s)")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
