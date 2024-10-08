# ME495 Embedded Systems Homework 1
Author: Logan Boswell

The turtle_control package provides a way to drive a turtle through a set of waypoints in turtlesim

1. Use `ros2 launch turtle_control waypoints.launch.xml` to run the code
2. The `ros2 service call /load turtle_interfaces/srv/Waypoints "{waypoints: [{x: 1.4, y: 1.6}, {x: 2.2, y: 9.4}, {x: 7.2, y: 6.1}, {x: 4.0, y: 2.6}, {x: 8.2, y: 1.5}, {x: 4.1, y: 5.3}]}"` service loads waypoints for the turtle to follow
3. The `ros2 service call /toggle std_srvs/srv/Empty` starts and stops the turtle.
4. Here is a video of the turtle in action with the commands above.
    <video src="https://github-production-user-asset-6210df.s3.amazonaws.com/181179449/374396453-6bfe70e7-9a13-4a59-851a-26d4da715280.webm?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=AKIAVCODYLSA53PQK4ZA%2F20241008%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Date=20241008T033130Z&X-Amz-Expires=300&X-Amz-Signature=bbe5dfd01f19661fd6ccc5b7841088719be2ba4a3f81cf24611102467e812d2e&X-Amz-SignedHeaders=host" width="500" />
5. When using the turtle.bag data in a new turtlesim window, the turtle tries to complete the same movements as in the video, but runs into the walls of the window since the turtle is not starting from the same location as the first waypoint in the video