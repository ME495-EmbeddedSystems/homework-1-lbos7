# ME495 Embedded Systems Homework 1
Author: Logan Boswell
1. Use `ros2 launch turtle_control waypoints.launch.xml` to run the code
2. The `ros2 service call /load turtle_interfaces/srv/Waypoints "{waypoints: [{x: 1.4, y: 1.6}, {x: 2.2, y: 9.4}, {x: 7.2, y: 6.1}, {x: 4.0, y: 2.6}, {x: 8.2, y: 1.5}, {x: 4.1, y: 5.3}]}"` service loads waypoints for the turtle to follow
3. The `ros2 service call /toggle std_srvs/srv/Empty` starts and stops the turtle.
4. Here is a video of the turtle in action.
    <video src="https://github-production-user-asset-6210df.s3.amazonaws.com/181179449/374395046-9744766b-12d1-4a7a-b023-6efc7c4d2f96.mp4?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=AKIAVCODYLSA53PQK4ZA%2F20241008%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Date=20241008T032714Z&X-Amz-Expires=300&X-Amz-Signature=46efe179350171e299ecb4e002d6c57eeb781a26f35ca2e889a1876e0efad78e&X-Amz-SignedHeaders=host" width="500" />
