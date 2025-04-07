# Turtle Ball Tracker Tutorial (ROS 2)

This ROS 2 node Tutorial genarates an image of a red ball that moves randomly across the screen and a TurtleSim turtle follows the moving ball in real time.

## Setup Instructions

1. **Open rqt** to visualize the ball movement.
2. **Monitor the following ROS 2 topic:**
   - `/camera/image_raw/`
3. **Execute the following in a terminal under the path:~/ros_tutorial_ws_2:**
    ros2 launch turtle_ball_tracking ball_tracker.launch.py