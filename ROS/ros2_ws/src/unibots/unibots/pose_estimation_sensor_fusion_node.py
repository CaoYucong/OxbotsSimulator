import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class PoseEstimationSensorFusionNode(Node):
    """Forwards /current_position_camera to /current_position unchanged."""

    def __init__(self) -> None:
        super().__init__('pose_estimation_sensor_fusion')

        self._pub = self.create_publisher(PoseStamped, '/current_position', 10)
        self.create_subscription(
            PoseStamped,
            '/current_position_camera',
            self._on_camera_pose,
            10,
        )

        self.get_logger().info(
            'pose_estimation_sensor_fusion started; forwarding '
            '/current_position_camera -> /current_position'
        )

    def _on_camera_pose(self, msg: PoseStamped) -> None:
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseEstimationSensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
