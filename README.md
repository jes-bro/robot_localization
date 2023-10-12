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

## How we solved the problem

In order to figure out where the robot actually is, we can't just interpret sensor data directly, because sensors have error and the world has uncertainty; therefore, sensor readings may not be an accurate indicator of where the robot is in space. 

To solve this problem, we created a particle filter that localizes the robot in a 2D map of its environment. 

A particle filter consists of a particle cloud. We created a particle cloud that consists of particles, or 2D vectors that represents a potential location of the robot. You could think of it as a hypothesis that indicates a potential place the robot could be. Each particle has an associated weight that reflects a confidence. The confidence indicates how probable it is that the particle is at the robot's true location. We then resample the particles and create a new cloud, with higher-weighted particles being more likely to be chosen. This creates an accurate filter that localizes the robot in 2D space. 

### Particle Filter Steps

Our particle filter script adhered to this series of steps:

1. The particle cloud is initialized around a given pose. If no initial pose is provided, it uses odometry data to create a particle cloud around where the wheel encoders suggest the robot is in space.
2. As new encoder/odometry and laser scan data are received, the particles locations and weights are updated, making use of this new data. The odometry data is used to update the particle locations and the laser scan data is used to update the particle weights. 
3. The particles are resampled according to their weights. Particles with higher weights are more likely to be resampled / are sampled more often. This is good because those particles are the most likely to represent the robots true state. 
4. The robot's estimated pose is updated based on the best particle / the particle with the highest confidence. 
5. Steps 2-4 repeat each time new odometry and laser scan data is recieved. 


### To Go More In-Depth

### How the particle filter works

We use ROS 2 to handle sensor data retrieval and particle publishing. 

#### Initialization

We create 3 normal distributions, each centered around the x, y, and theta values of the robots initial pose, respectively. To generate the normal distributions we use numpy.random.normal: 

```python
def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """ Initialize the particle cloud.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is omitted, the odometry will be used """
        # Initialize xy_theta / robot's initial pose to odom pose if no initial pose is provided
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose) 
        # Initialize particle cloud
        self.particle_cloud = []
        # Create standard deviations for xy distributions
        xy_standard_deviation = 0.002 # 0.1
        # Create theta standard deviation for theta distribution
        theta_standard_deviation = 0.001
        # Set scale for theta distribution. We do this to make it more steep and less wide. 
        self.distribution_scale = 10
        # xy_theta is a tuple, so we need to extract each component of the robot's location
        x = xy_theta[0]
        y = xy_theta[1]
        theta = xy_theta[2]
        # Create distributions / sample n_particles from them
        self.xs = np.random.normal(x, x, self.n_particles)
        self.ys = np.random.normal(y, xy_standard_deviation, self.n_particles)
        self.thetas = self.distribution_scale * np.random.normal(theta, theta_standard_deviation, self.n_particles)
        # Create particle objects using these randomly generated xy_theta values and append them to the particle cloud
        for i in range(self.n_particles):
            self.particle_cloud.append(Particle(self.xs[i], self.ys[i], self.thetas[i], 1/self.n_particles))
        # Normalize the particle weights
        self.normalize_particles()
        # Update the robot's pose
        self.update_robot_pose()
```

In each iteration of the run loop, the following functions are called:


            self.update_particles_with_odom()            # update particle poses based on odometry
            self.update_particles_with_laser(r, theta)   # update particle weights based on laser scan
            self.publish_particles(self.last_scan_timestamp)
            self.update_robot_pose()                     # update robot's estimated pose based on particles
            self.resample_particles()                    # resample particles to focus on areas of high density

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
