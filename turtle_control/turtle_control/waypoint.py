

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.srv import SetPen
from enum import Enum, auto

class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration
    """
    STOPPED = auto(),
    MOVING = auto()

class Waypoint(Node):

    def __init__(self):
        super().__init__('waypoint')
        self.get_logger().info('Waypoint')
        self.declare_parameter('frequency', 90)
        self.freq = self.get_parameter('frequency').value
        self.tmr = self.create_timer(1/self.freq, self.timer_callback)
        self.toggle = self.create_service(Empty, 'toggle', self.toggle_callback)
        self.path_client = self.create_client(SetPen, 'set_pen')
        self.state = State.STOPPED
        self.first_log_for_stop = True

    def timer_callback(self):
        if self.state == State.STOPPED:
            if self.first_log_for_stop:
                self.get_logger().info('Stopping.')
                self.first_log_for_stop = False
        elif self.state == State.MOVING:
            self.get_logger().error('Issuing Command!')
        
    def toggle_callback(self, request, response):
        if self.state == State.STOPPED:
            self.state = State.MOVING
            self.path_client.call_async(SetPen.Request(r=255, g=255, b=255,
                                                       width=3, off=0))
        elif self.state == State.MOVING:
            self.state = State.STOPPED
            self.path_client.call_async(SetPen.Request(r=255, g=255, b=255,
                                                       width=3, off=1))
            self.first_log_for_stop = True
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