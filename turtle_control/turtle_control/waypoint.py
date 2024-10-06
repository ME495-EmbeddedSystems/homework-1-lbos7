

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Empty
from turtlesim.srv import SetPen, TeleportAbsolute, TeleportRelative
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Vector3
from turtle_interfaces.srv import Waypoints
from enum import Enum, auto
import math

class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration
    """
    STOPPED = auto(),
    MOVING = auto()

class Waypoint(Node):

    def __init__(self):
        super().__init__('waypoint')
        cb_group = MutuallyExclusiveCallbackGroup()
        self.get_logger().info('Waypoint')
        self.declare_parameter('frequency', 90)
        self.freq = self.get_parameter('frequency').value
        self.tmr = self.create_timer(1/self.freq, self.timer_callback)
        self.toggle = self.create_service(Empty, 'toggle', self.toggle_callback)
        self.load = self.create_service(Waypoints, 'load', self.load_callback)
        self.path_client = self.create_client(SetPen, 'set_pen', callback_group=cb_group)
        self.reset_client = self.create_client(Empty, 'reset', callback_group=cb_group)
        self.teleport_abs_client = self.create_client(TeleportAbsolute, 'teleport_absolute', callback_group=cb_group)
        self.teleport_rel_client = self.create_client(TeleportRelative, 'teleport_relative', callback_group=cb_group)
        # self.pose = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.state = State.STOPPED
        self.first_log_for_stop = True
        self.pen_state = 1
        self.pose = 0


    def timer_callback(self):
        if self.state == State.STOPPED:
            if self.first_log_for_stop:
                self.get_logger().info('Stopping.')
                self.first_log_for_stop = False
        elif self.state == State.MOVING:
            self.get_logger().debug('Issuing Command!')
        
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


    async def load_callback(self, request, response):
        await self.reset_client.call_async(Empty.Request())
        is_first_point = True
        first_point = {}
        prev_point = {}
        if len(request.waypoints) > 0:
            for i in range(len(request.waypoints)):
                if not is_first_point:
                    response.distance += math.dist([request.waypoints[i].x,
                                           request.waypoints[i].y],
                                          [prev_point.x, prev_point.y])
                    prev_point = request.waypoints[i]
                else:
                    prev_point = request.waypoints[i]
                    first_point = request.waypoints[i]
                    is_first_point = False
            await self.draw_x(request.waypoints)
            self.path_client.call_async(SetPen.Request(r=255, g=255, b=255,
                                                        width=3, off=1))
            await self.teleport_abs_client.call_async(TeleportAbsolute.Request(
                x=first_point.x, y=first_point.y, theta=0.0))
        else:
            self.state = State.STOPPED
        return response
        
    
    async def draw_x(self, points):
        for point in points:
            self.path_client.call_async(SetPen.Request(r=255, g=0, b=0,
                                                        width=3, off=1))
            await self.teleport_abs_client.call_async(TeleportAbsolute.Request(
                x=point.x, y=point.y, theta=0.0))
            self.path_client.call_async(SetPen.Request(r=255, g=0, b=0,
                                                        width=3, off=0))
            await self.teleport_rel_client.call_async(TeleportRelative.Request(
                linear=.2, angular=math.pi/4))
            await self.teleport_rel_client.call_async(TeleportRelative.Request(
                linear=.4, angular=math.pi))
            await self.teleport_rel_client.call_async(TeleportRelative.Request(
                linear=.2, angular=math.pi))
            await self.teleport_rel_client.call_async(TeleportRelative.Request(
                linear=.2, angular=math.pi/2))
            await self.teleport_rel_client.call_async(TeleportRelative.Request(
                linear=.4, angular=math.pi))
            

    # def draw_x(self, point):
    #     self.path_client.call_async(SetPen.Request(r=255, g=0, b=0,
    #                                                    width=3, off=0))
    #     self.teleport_abs_client.call_async(TeleportAbsolute.Request(
    #         x=point.x, y=point.y, theta=0.0))
    #     self.get_logger().error('center of point')
    #     self.teleport_rel_client.call_async(TeleportRelative.Request(
    #         linear=.2, angular=math.pi/4))
    #     self.teleport_rel_client.call_async(TeleportRelative.Request(
    #         linear=.4, angular=math.pi))
    #     self.teleport_abs_client.call_async(TeleportAbsolute.Request(
    #         x=point.x, y=point.y, theta=math.pi))
    #     self.teleport_rel_client.call_async(TeleportRelative.Request(
    #         linear=.2, angular=-math.pi/4))
    #     self.teleport_rel_client.call_async(TeleportRelative.Request(
    #         linear=.4, angular=math.pi))
    #     self.path_client.call_async(SetPen.Request(r=255, g=255, b=255,
    #                                                    width=3, off=1))


    def pose_callback(self, pose):
        self.pose = pose

        

def main(args=None):
    """Entrypoint for the waypoint ROS node."""
    rclpy.init(args=args)
    node = Waypoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)