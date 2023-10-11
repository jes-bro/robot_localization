Writeup (Due 10-13)Permalink

In your ROS package create a README.md file to hold your project writeup. Your writeup should touch on the following topics. We expect this writeup to be done in such a way that you are proud to include it as part of your professional portfolio. As such, please make sure to write the report so that it is understandable to an external audience. Make sure to add pictures to your report, links to Youtube videos, embedded animated Gifs (these can be recorded with the tool peek).

    What was the goal of your project?
    How did you solve the problem? (Note: this doesn’t have to be super-detailed, you should try to explain what you did at a high-level so that others in the class could reasonably understand what you did).
    Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.
    What if any challenges did you face along the way?
    What would you do to improve your project if you had more time?
    Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.

ros2 launch neato2_gazebo neato_gauntlet_world.py
ros2 launch robot_localization launch_map_server.py map_yaml:=./gauntlet.yaml
RUN FROM robot_localization/maps

# Robot Localization
The goal of robot localization is to find a robot's position relative to a global map frame. For example, if you drop a robot randomly in a room and turn it on, how does the robot know where it is in the room? Or another example, if a robot knows where its starting position is and starts moving, how do we account for the error in the wheel/positional sensors to get a reliable position reading of the robot relative ot the room? Using a technique like a particle filter and laser scan data from a robot, we can find the robots location relative to a global frame without knowing where it started!