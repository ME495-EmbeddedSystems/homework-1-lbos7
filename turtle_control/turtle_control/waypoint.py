

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class Waypoint(Node):

    def __init__(self):
        super().__init__('waypoint')
        self.get_logger().info('Waypoint')
        self.declare_parameter('frequency', 90)
        self._freq = self.get_parameter('frequency').value
        self._tmr = self.create_timer(1/self._freq, self.timer_callback)
        self._toggle = self.create_service(Empty, 'toggle', self.toggle_callback)
        self._state = 'STOPPED'
        self._first_log_for_stop = True

    def timer_callback(self):
        if self._state == 'STOPPED':
            if self._first_log_for_stop:
                self.get_logger().info('Stopping.')
                self._first_log_for_stop = False
        elif self._state == 'MOVING':
            self.get_logger().error('Issuing Command!')
        
    def toggle_callback(self, request, response):
        if self._state == 'STOPPED':
            self._state = 'MOVING'
        elif self._state == 'MOVING':
            self._state = 'STOPPED'
            self._first_log_for_stop = True
        return response

        

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