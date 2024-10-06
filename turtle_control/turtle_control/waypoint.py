

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
        self.pose_sub = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state = State.STOPPED
        self.first_log_for_stop = True
        self.pose = []
        self.waypoints = []
        self.current_waypoint_ind = 1
        self.waypoints_forward = True
        self.distance_error = 0.0
        self.angle_error = 0.0
        self.dist_thresh = .05
        self.lin_gain = 2.5
        self.angle_gain = 7.5


    def timer_callback(self):
        if self.state == State.STOPPED:
            if self.first_log_for_stop:
                self.get_logger().info('Stopping.')
                self.first_log_for_stop = False
            self.vel_pub.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))
        elif self.state == State.MOVING:
            self.get_logger().debug('Issuing Command!')
            if len(self.waypoints) > 0:
                target = self.waypoints[self.current_waypoint_ind]
                self.distance_error = math.dist([target.x, target.y], [self.pose.x, self.pose.y])
                self.angle_error = math.atan2(target.y - self.pose.y, target.x - self.pose.x) - self.pose.theta
                self.vel_pub.publish(Twist(linear=Vector3(x=self.lin_gain*self.distance_error), angular=Vector3(z=self.angle_gain*self.angle_error)))
                if self.distance_error < self.dist_thresh:
                    if self.current_waypoint_ind == (len(self.waypoints) - 1):
                        self.current_waypoint_ind -= 1
                        self.waypoints_forward = False
                    elif self.current_waypoint_ind == 0:
                        self.current_waypoint_ind += 1
                        self.waypoints_forward = True
                    elif self.waypoints_forward:
                        self.current_waypoint_ind += 1
                    else:
                        self.current_waypoint_ind -= 1

                
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
        self.waypoints = request.waypoints
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