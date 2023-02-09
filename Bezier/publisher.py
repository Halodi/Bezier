import rclpy
import rclpy.qos as qos
from rclpy.node import Node

from ocs2_ros2_msgs.msg import BezierTrajectory

rclpy.init()


class BezierPublisher(Node):

    def __init__(self):
        super().__init__('bezier_publisher')
        qos_reliable = qos.QoSProfile(
            history=qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=qos.QoSDurabilityPolicy.VOLATILE,
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
        )

        self.publisher_ = self.create_publisher(BezierTrajectory, '/eve/bezier_trajectory',
                                                qos_reliable)

    def publish(self, msg: BezierTrajectory):
        self.publisher_.publish(msg)
