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

## Particle Filter

A particle filter works by using particles distributed around a global map to converge on the robots position. Here are the steps:

1. Distribute particles each with a random x position, y position, and rotation value around a global map.
2. Gather laser scan data from the robots laser scanner
3. Map each laser scan onto each particle and calculate an confidence for each particle depending on how the mapped laser scan matches with the occupancy grid of the global map. **\***
4.

Lets dive into each step.

At the beginning of each run, we must initialize particles to our map frame. A default approach is to evenly cover the entire global frame with particles with random rotations to ensure that we are somewhat close to the NEATOs starting position. However, in our scenario we are given the starting position of the NEATO, so a better approach is to concentrate more of the particles around this starting position. In order to get a spread on our particles, we can add noise

## Design Decisions

One interesting design decision we made was how we assigned confidence values to each of our particles. We decided first accumulate error on each particle, and then transform that error into a confidence value. The error of each particle is determined with how close the laser scan in its frame aligns with the occupancy grid of the map. In the code below, we loop through each particle in our particle cloud. For each particle, we loop through laser scan data. In each of these loops is where the error accumulates. First we calculute the error by transforming the laser scan data onto the particles position and then recording how close that laser scan is to a point covered in the occupancy grid. If its not close, that means that particle's laser scan doesn't match with the robot's laser scan, so we add that difference to an error that accumulates over the rest of the ranges.

```python
for particle in self.particle_cloud:
    accumulated_error = 0
    for range_index, range in enumerate(r):
        # Transforms laser scan to particle's frame
        x = range * cos(theta[range_index])
        y = range * sin(theta[range_index])
        range_pose = np.array([x, y, 1]).T
        particle_transform = particle.make_homogeneous_transform()
        range_in_map = particle_transform @ range_pose
        # ...

        # Error is how far this scan in the particle's frame is from
        # covered point in the occupancy grid
        error = self.occupancy_field.get_closest_obstacle_distance(range_in_map[0], range_in_map[1])

        # If nan, we add a specific nan penalty
        # If not nan, add this scans error to the accumulated error for
        # # the particle
        if np.isnan(error):
            accumulated_error += self.nan_penalty
        else:
            accumulated_error += error
    # ...

    # Particle confidence is the inverse of the error
    # If the error is large, the confidence will be small
    # If the error is small, the confidence will be large
    particle.w = 1 / accumulated_error
```

This process was relatively straight forward, but it wasn't so obvious how to convert this error to a confidence value for each particle. Originally, we tried normalizing all of the particle's errors relative to eachother and then subtracting their error from 1. This led to most of the particle's error being very close to eachother which didn't allow a most confident particle to stand out as much. After some trials in the simulator, we landed on a simple inversion of the error to get our particle's confidence. This has the benefit of being a much simpler approach and much more computationally efficient than having to do an extra normalization step for each pass.

## Challenges

One Python specific challenge that arose was dealing with NaN edgecases in our code. This was especially prevelant in our initialize_particle_cloud function where the laser scan data has the possibility to contain NaNs. The challenge was that these NaNs would not throw errors in our initialize_particle_cloud function and therefore could propagate throughout our code, causing cryptic type errors to appear far down the stack. To rectify this, we added some simple isNaN checking statements in our initialize_particle_cloud function. The solution may have been straightforward, but from this oversight we learned that checking the output of a function, whether that be with print statements, or the debugger, will help reduce the search area when encountering type errors.

Apart from python errors, we had trouble using the RVIZ simulator due to a lack of knowledge of how config files work. We could have avoided this issue by reading through the "end of the finish line" template that was given to us. We found that when exposed to new tools with long configuration processes, it was handy to keep a list of commands/actions in a text file for reference.

##
