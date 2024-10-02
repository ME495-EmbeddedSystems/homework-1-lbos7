

import rclpy
from rclpy.node import Node

class Waypoint(Node):

    def __init__(self):
        super().__init__('waypoint')
        self.get_logger().info('Waypoint')
        self.declare_parameter('frequency', 90)
        self._freq = self.get_parameter('frequency').value
        self._tmr = self.create_timer(1/self._freq, self.timer_callback)

    def timer_callback(self):
        self.get_logger().debug('Issuing Command!')

def main(args=None):
    """Entrypoint for the mynode ROS node."""
    rclpy.init(args=args)
    node = Waypoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)