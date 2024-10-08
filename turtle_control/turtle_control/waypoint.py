""" Class for a waypoint node in the turtle_control package """
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Empty
from turtlesim.srv import SetPen, TeleportAbsolute, TeleportRelative
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Vector3
from turtle_interfaces.srv import Waypoints
from turtle_interfaces.msg import ErrorMetric
from enum import Enum, auto
import math

class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration
    """
    STOPPED = auto(),
    MOVING = auto()

class Waypoint(Node):
    """
    Node for controlling a turtle's movement towards waypoints in turtlesim.

    Publishes
    ---------
    cmd_vel : geometry_msgs/msg/Twist - velocity to control the turtle
    loop_metrics : turtle_interfaces/msg/ErrorMetric - information about turtle's motion

    Subscribes
    ----------
    pose : turtlesim/msg/Pose - the pose of the turtle

    Services
    ----------
    toggle : std_srvs/srv/Empty - changes turtle's current state
    load : turtle_interfaces/srv/Waypoints - loads a set of waypoints for the turtle to move towards

    Service Clients
    ----------
    set_pen : turtlesim/srv/SetPen - changes pen settings for turtle
    reset : std_srvs/srv/Empty - loads a set of waypoints for the turtle to move towards
    teleport_absolute : turtlesim/srv/TeleportAbsolute - teleports the turtle to a given x, y, and theta position
    teleport_relative : turtlesim/srv/TeleportRelative - teleports the a linear distance and angular distance away from current position

    Parameters
    ----------
    frequency : float64 - the frequency at which the timer callback is executed
    tolerance : float64 - the tolerance for if a turtle has reached a waypoint
    """

    def __init__(self):
        super().__init__('waypoint')
        cb_group = MutuallyExclusiveCallbackGroup()

        self.get_logger().info('Waypoint')

        self.declare_parameter('frequency', 90.0)
        self.declare_parameter('tolerance', .05)
        self.freq = self.get_parameter('frequency').value
        self.tolerance = self.get_parameter('tolerance').value

        self.tmr = self.create_timer(1/self.freq, self.timer_callback)

        self.toggle = self.create_service(Empty, 'toggle', self.toggle_callback)
        self.load = self.create_service(Waypoints, 'load', self.load_callback)

        self.path_client = self.create_client(SetPen, 'set_pen', callback_group=cb_group)
        self.reset_client = self.create_client(Empty, 'reset', callback_group=cb_group)
        self.teleport_abs_client = self.create_client(TeleportAbsolute, 'teleport_absolute', callback_group=cb_group)
        self.teleport_rel_client = self.create_client(TeleportRelative, 'teleport_relative', callback_group=cb_group)
        
        self.pose_sub = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.error_pub = self.create_publisher(ErrorMetric, 'loop_metrics', 10)
        
        self.state = State.STOPPED
        self.first_log_for_stop = True
        self.pose = []
        self.prev_pose = []
        self.waypoints = []
        self.current_waypoint_ind = 1
        self.waypoints_forward = True
        self.distance_error = 0.0
        self.angle_error = 0.0
        self.lin_gain = 1
        self.angle_gain = 8
        self.minimum_vel = 3.0
        self.complete_loops = 0
        self.actual_distance = 0.0
        self.sl_dist = 0.0
        self.error = 0.0


    def timer_callback(self):
        """ Callback for timer
        
        Publishes commands at a set frequency paramter (commands vary based on turtle state)
        """
        if self.state == State.STOPPED:
            if self.first_log_for_stop:
                self.get_logger().info('Stopping.')
                self.first_log_for_stop = False
            self.vel_pub.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))

        elif self.state == State.MOVING:
            self.get_logger().debug('Issuing Command!')

            # Only moves the turtle if there are waypoints for the turtle to follow
            if len(self.waypoints) > 0:

                # Finding Errors used for P-Control
                target = self.waypoints[self.current_waypoint_ind]
                self.distance_error = math.dist([target.x, target.y], [self.pose.x, self.pose.y])
                self.angle_error = math.atan2(target.y - self.pose.y, target.x - self.pose.x) - self.pose.theta

                # Adjusting for if the magnitude of the error is greater than pi
                if self.angle_error > math.pi:
                    self.angle_error = self.angle_error - 2*math.pi
                elif self.angle_error < -math.pi:
                    self.angle_error = 2*math.pi + self.angle_error

                # Publishing to cmd_vel (linear speed has a minimum value to avoid slowing when approaching a waypoint)
                self.vel_pub.publish(
                    Twist(linear=Vector3(x=max(self.lin_gain*self.distance_error, self.minimum_vel)),
                          angular=Vector3(z=self.angle_gain*self.angle_error)))
                
                # Checking to see if the turtle is close enough to waypoint
                if self.distance_error < self.tolerance:

                    # Setting next waypoint for turtle to move towards and publishing to /loop_metrics if applicable
                    if self.current_waypoint_ind == (len(self.waypoints) - 1):
                        self.current_waypoint_ind -= 1
                        self.waypoints_forward = False
                    elif self.current_waypoint_ind == 0:
                        self.current_waypoint_ind += 1
                        self.complete_loops += 1
                        self.error = self.actual_distance - 2*self.complete_loops*self.sl_dist
                        self.error_pub.publish(ErrorMetric(complete_loops=self.complete_loops, actual_distance=self.actual_distance, error=self.error))
                        self.waypoints_forward = True
                    elif self.waypoints_forward:
                        self.current_waypoint_ind += 1
                    else:
                        self.current_waypoint_ind -= 1

                
    def toggle_callback(self, request, response):
        """ Callback for the toggle service

        Changes the turtle's current state & adjusts pen

            Args:
                request (Empty)

            Returns:
                response (Empty)
        """
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
        """ Callback function for the load service

        Resets turtlesim window and variables related to traversing waypoints,
        loads waypoints, and draws an X at each waypoint

            Args:
                request (Waypoints_Request) : the waypoint field contains a list
                of point dictionaries with x, y, and z components

            Returns:
                response (Waypoints_Response) : the straight-line distance
                between all of the waypoints
        """

        # Resetting variables and turtlesim window for new waypoints
        self.waypoints = request.waypoints
        self.current_waypoint_ind = 1
        self.waypoints_forward = True
        self.complete_loops = 0
        self.actual_distance = 0.0
        self.error = 0.0
        self.error_pub.publish(ErrorMetric(complete_loops=self.complete_loops, actual_distance=self.actual_distance, error=self.error))
        await self.reset_client.call_async(Empty.Request())

        
        is_first_point = True
        first_point = {}
        prev_point = {}

        # Only runs if waypoints are given
        if len(request.waypoints) > 0:

            # Calculating straight-line distance between waypoints
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

            # Drawing markers and then moving turtle to first waypoint
            await self.draw_x(request.waypoints)
            self.path_client.call_async(SetPen.Request(r=255, g=255, b=255,
                                                        width=3, off=1))
            await self.teleport_abs_client.call_async(TeleportAbsolute.Request(
                x=first_point.x, y=first_point.y, theta=0.0))
            
        # Stopping turtle if it was moving when the load service was called
        if self.state == State.MOVING:
            self.state = State.STOPPED
            self.first_log_for_stop = True

        # Saving straight-line distance and returning response
        self.sl_dist = response.distance
        return response
        
    
    async def draw_x(self, points):
        """ Draws an X at each of the given points

            Args:
                points (list of point dictionaries) : a list of point dictionaries
                where the markers should be drawn
        """
        for point in points:
            self.path_client.call_async(SetPen.Request(r=0, g=0, b=0,
                                                        width=3, off=1))
            await self.teleport_abs_client.call_async(TeleportAbsolute.Request(
                x=point.x, y=point.y, theta=0.0))
            self.path_client.call_async(SetPen.Request(r=0, g=0, b=0,
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
        """ Updates the self.pose, self.prev_pose, and self.actual_distance variables 
        
            Args:
                pose (dictionary) : the current pose of the turtle
        """
        self.prev_pose = self.pose
        self.pose = pose

        # if the current and previous poses are both the same type, update the actual distance travelled by the turtle
        if type(self.pose) == type(self.prev_pose):
            self.actual_distance += math.dist([self.pose.x, self.pose.y],
                                            [self.prev_pose.x, self.prev_pose.y])


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