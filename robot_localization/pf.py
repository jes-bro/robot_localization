#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

import rclpy
from typing import List
from threading import Thread
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import ParticleCloud, Particle
from nav2_msgs.msg import Particle as Nav2Particle
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from rclpy.duration import Duration
import math
import time
import numpy as np
from numpy import sin, cos
from occupancy_field import OccupancyField
from helper_functions import TFHelper, draw_random_sample, stamped_transform_to_pose
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler


class Particle(object):
    """Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
    Attributes:
        x: the x-coordinate of the hypothesis relative to the map frame
        y: the y-coordinate of the hypothesis relative ot the map frame
        theta: the yaw of the hypothesis relative to the map frame
        w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, w=1.0):
        """Construct a new Particle
        x: the x-coordinate of the hypothesis relative to the map frame
        y: the y-coordinate of the hypothesis relative ot the map frame
        theta: the yaw of KeyboardInterruptthe hypothesis relative to the map frame
        w: the particle weight (the class does not ensure that particle weights are normalized
        """
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """A helper function to convert a particle to a geometry_msgs/Pose message"""
        q = quaternion_from_euler(0, 0, self.theta)
        return Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
        )

    # TODO: define additional helper functions if needed

    def make_homogeneous_transform(self):
        transform = np.array(
            [
                [cos(self.theta), -sin(self.theta), self.x],
                [sin(self.theta), cos(self.theta), self.y],
                [0, 0, 1],
            ]
        )
        return transform

    def update_pose_from_transform(self, transform):
        self.theta = np.arctan2(transform[1, 0], transform[0, 0])
        self.x = transform[0, 2]
        self.y = transform[1, 2]


class ParticleFilter(Node):
    """The class that represents a Particle Filter ROS Node
    Attributes list:
        base_frame: the name of the robot base coordinate frame (should be "base_footprint" for most robots)
        map_frame: the name of the map coordinate frame (should be "map" in most cases)
        odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
        scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
        n_particles: the number of particles in the filter
        d_thresh: the amount of linear movement before triggering a filter update
        a_thresh: the amount of angular movement before triggering a filter update
        pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
        particle_pub: a publisher for the particle cloud
        last_scan_timestamp: this is used to keep track of the clock when using bags
        scan_to_process: the scan that our run_loop should process next
        occupancy_field: this helper class allows you to query the map for distance to closest obstacle
        transform_helper: this helps with various transform operations (abstracting away the tf2 module)
        particle_cloud: a list of particles representing a probability distribution over robot poses
        current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                               The pose is expressed as a list [x,y,theta] (where theta is the yaw)
        thread: this thread runs your main loop
    """

    def __init__(self):
        super().__init__("pf")
        self.base_frame = "base_footprint"  # the frame of the robot base
        self.map_frame = "map"  # the name of the map coordinate frame
        self.odom_frame = "odom"  # the name of the odometry coordinate frame
        self.scan_topic = "scan"  # the topic where we will get laser scans from

        self.n_particles = 300  # the number of particles to use

        self.d_thresh = 0.2  # the amount of linear movement before performing an update
        self.a_thresh = (
            math.pi / 6
        )  # the amount of angular movement before performing an update
        self.nan_penalty = 20

        # TODO: define additional constants if needed

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self.update_initial_pose, 10
        )

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = self.create_publisher(
            ParticleCloud, "particle_cloud", qos_profile_sensor_data
        )

        # laser_subscriber listens for data from the lidar
        self.create_subscription(LaserScan, self.scan_topic, self.scan_received, 10)

        # this is used to keep track of the timestamps coming from bag files
        # knowing this information helps us set the timestamp of our map -> odom
        # transform correctly
        self.last_scan_timestamp = None
        # this is the current scan that our run_loop should process
        self.scan_to_process = None
        # your particle cloud will go here
        self.particle_cloud: List[Particle] = []

        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField(self)
        self.transform_helper = TFHelper(self)

        # we are using a thread to work around single threaded execution bottleneck
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        self.transform_update_timer = self.create_timer(0.05, self.pub_latest_transform)

    def pub_latest_transform(self):
        """This function takes care of sending out the map to odom transform"""
        if self.last_scan_timestamp is None:
            return
        postdated_timestamp = Time.from_msg(self.last_scan_timestamp) + Duration(
            seconds=0.1
        )
        self.transform_helper.send_last_map_to_odom_transform(
            self.map_frame, self.odom_frame, postdated_timestamp
        )

    def loop_wrapper(self):
        """This function takes care of calling the run_loop function repeatedly.
        We are using a separate thread to run the loop_wrapper to work around
        issues with single threaded executors in ROS2"""
        while True:
            self.run_loop()
            time.sleep(0.1)

    def run_loop(self):
        """This is the main run_loop of our particle filter.  It checks to see if
        any scans are ready and to be processed and will call several helper
        functions to complete the processing.

        You do not need to modify this function, but it is helpful to understand it.
        """
        if self.scan_to_process is None:
            return
        msg = self.scan_to_process

        (new_pose, delta_t) = self.transform_helper.get_matching_odom_pose(
            self.odom_frame, self.base_frame, msg.header.stamp
        )
        if new_pose is None:
            # we were unable to get the pose of the robot corresponding to the scan timestamp
            if delta_t is not None and delta_t < Duration(seconds=0.0):
                # we will never get this transform, since it is before our oldest one
                self.scan_to_process = None
            return
        t_start = time.time()
        (r, theta) = self.transform_helper.convert_scan_to_polar_in_robot_frame(
            msg, self.base_frame
        )
        print("r[0]={0}, theta[0]={1}".format(r[0], theta[0]))
        # clear the current scan so that we can process the next one
        self.scan_to_process = None

        self.odom_pose = new_pose
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
            self.odom_pose
        )
        print("x: {0}, y: {1}, yaw: {2}".format(*new_odom_xy_theta))

        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
        elif not self.particle_cloud:
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)
        elif self.moved_far_enough_to_update(new_odom_xy_theta):
            # we have moved far enough to do an update!
            self.update_particles_with_odom()  # update based on odometry
            self.update_particles_with_laser(r, theta)  # update based on laser scan
            self.publish_particles(self.last_scan_timestamp)
            self.update_robot_pose()  # update robot's pose based on particles
            self.resample_particles()  # resample particles to focus on areas of high density
        # publish particles (so things like rviz can see them)
        self.publish_particles(self.last_scan_timestamp)

        print(f"elapsed: {time.time() - t_start}")

    def moved_far_enough_to_update(self, new_odom_xy_theta):
        return (
            math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0])
            > self.d_thresh
            or math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1])
            > self.d_thresh
            or math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2])
            > self.a_thresh
        )

    def update_robot_pose(self):
        """Update the estimate of the robot's pose given the updated particles.
        There are two logical methods for this:
            (1): compute the mean pose
            (2): compute the most likely pose (i.e. the mode of the distribution)
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()

        # TODO: assign the latest pose into self.robot_pose as a geometry_msgs.Pose object
        # just to get started we will fix the robot's pose to always be at the origin

        confidences = []

        # Add all of the confidences of each particle in the cloud to a list
        for particle in self.particle_cloud:
            confidences.append(particle.w)

        # Find the index of the particle with the highest confidence
        max_confidence_particle_index = confidences.index(max(confidences))

        # Index the particle cloud with the best confidence particle index
        best_particle = self.particle_cloud[max_confidence_particle_index]

        # Update the robots pose in rviz with the best particles position
        self.robot_pose = best_particle.as_pose()

        if hasattr(self, "odom_pose"):
            self.transform_helper.fix_map_to_odom_transform(
                self.robot_pose, self.odom_pose
            )
        else:
            self.get_logger().warn(
                "Can't set map->odom transform since no odom data received"
            )

    def update_particles_with_odom(self):
        """Update the particles using the newly given odometry pose.
        The function computes the value delta which is a tuple (x,y,theta)
        that indicates the change in position and angle between the odometry
        when the particles were last updated and the current odometry.
        """
        # Convert odom pose object to x, y, theta tuple
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
            self.odom_pose
        )
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            # Update old odom pose
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (
                new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                new_odom_xy_theta[2] - self.current_odom_xy_theta[2],
            )
            # Update new odom pose
            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return
        # Relabeling for the sake of making the code look more like math
        t1 = old_odom_xy_theta
        # Index 2 in tuple is theta
        t1_theta = t1[2]
        t2 = new_odom_xy_theta
        t2_theta = t2[2]
        # Transformation matrix with rotation (upper left) and translation right hand column, first two rows) for t1 pose
        t1_to_odom = np.array(
            [
                [cos(t1_theta), -sin(t1_theta), t1[0]],
                [sin(t1_theta), cos(t1_theta), t1[1]],
                [0, 0, 1],
            ]
        )
        # Transformation matrix with rotation (upper left) and translation right hand column, first two rows) for t2 pose
        t2_to_odom = np.array(
            [
                [cos(t2_theta), -sin(t2_theta), t2[0]],
                [sin(t2_theta), cos(t2_theta), t2[1]],
                [0, 0, 1],
            ]
        )
        # Create t2 in t1 transform
        t2_in_1 = np.linalg.inv(t1_to_odom) @ t2_to_odom
        # Apply transform to all particles
        for particle in self.particle_cloud:
            # Make transform for each particle
            particle_transform = particle.make_homogeneous_transform()
            # Apply transform to each particle
            particle.update_pose_from_transform(particle_transform @ t2_in_1)

    def resample_particles(self):
        """Resample the particles according to the new particle weights.
        The weights stored with each particle should define the probability that a particular
        particle is selected in the resampling step.  You may want to make use of the given helper
        function draw_random_sample in helper_functions.py.
        """
        noise_std = 0.02
        self.normalize_particles()
        probabilities = []
        # Make a list of all of the particle weights
        for particle in self.particle_cloud:
            probabilities.append(particle.w)

        # Draw a random sample of particles from the particle cloud based off
        # a distribution of the particle weights
        self.particle_cloud = draw_random_sample(
            self.particle_cloud, probabilities, self.n_particles
        )

        # Add noise to the x, y, and theta components to each particle based off
        # class attributes for standard deviation
        for particle in self.particle_cloud:
            # Generate noise for each particle
            x_noise = np.random.normal(0.0, noise_std)
            y_noise = np.random.normal(0.0, noise_std)
            theta_noise = self.distribution_scale * np.random.normal(0.0, noise_std)

            # Apply noise to each particle
            particle.x += x_noise
            particle.y += y_noise
            particle.theta += theta_noise
        self.normalize_particles()

    def update_particles_with_laser(self, r, theta):
        """Updates the particle weights in response to the scan data
        r: the distance readings to obstacles
        theta: the angle relative to the robot frame for each corresponding reading
        """

        for particle in self.particle_cloud:
            # Accumulate error for each particle
            accumulated_error = 0
            for range_index, range in enumerate(r):
                # Convert cartesian to polar coordinates
                x = range * cos(theta[range_index])
                y = range * sin(theta[range_index])
                range_pose = np.array([x, y, 1]).T

                # Convert NEATO frame laser scan to particle frame
                particle_transform = particle.make_homogeneous_transform()
                range_in_map = particle_transform @ range_pose

                # Check if laser scan data is nan
                if np.isnan(range_in_map[0]) or np.isnan(range_in_map[1]):
                    print("isnan")
                    continue

                # Get distance of particle laser scan to occupancy field
                error = self.occupancy_field.get_closest_obstacle_distance(
                    range_in_map[0], range_in_map[1]
                )

                # Apply a NaN penalty if error is NaN otherwise, add error
                # to accumulated error
                if np.isnan(error):
                    accumulated_error += self.nan_penalty
                else:
                    accumulated_error += error

            assert not np.isnan(accumulated_error)

            # Take inverse of accumulated error to get weight
            particle.w = 1 / accumulated_error

        self.normalize_particles()

    def update_initial_pose(self, msg):
        """Callback function to handle re-initializing the particle filter based on a pose estimate.
        These pose estimates could be generated by another ROS Node or could come from the rviz GUI
        """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """Initialize the particle cloud.
        Arguments
        xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                  particle cloud around.  If this input is omitted, the odometry will be used
        """
        # Initialize xy_theta / robot's initial pose to odom pose if no initial pose is provided
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
                self.odom_pose
            )
        # Initialize particle cloud
        self.particle_cloud = []
        # Create standard deviations for xy distributions
        xy_standard_deviation = 0.02  # 0.1
        # Create theta standard deviation for theta distribution
        theta_standard_deviation = 0.001
        # Set scale for theta distribution. We do this to make it more steep and less wide.
        self.distribution_scale = 10
        # xy_theta is a tuple, so we need to extract each component of the robot's location
        x = xy_theta[0]
        y = xy_theta[1]
        theta = xy_theta[2]
        # Create distributions / sample n_particles from them
        self.xs = np.random.normal(x, xy_standard_deviation, self.n_particles)
        self.ys = np.random.normal(y, xy_standard_deviation, self.n_particles)
        self.thetas = self.distribution_scale * np.random.normal(
            theta, theta_standard_deviation, self.n_particles
        )
        # Create particle objects using these randomly generated xy_theta values and append them to the particle cloud
        for i in range(self.n_particles):
            self.particle_cloud.append(
                Particle(self.xs[i], self.ys[i], self.thetas[i], 1 / self.n_particles)
            )
        # Normalize the particle weights
        self.normalize_particles()
        # Update the robot's pose
        self.update_robot_pose()

    def normalize_particles(self):
        """Make sure the particle weights define a valid distribution (i.e. sum to 1.0)"""
        # TODO: implement thisdraw_random_sample(self.x_distrobution, )
        weighted_sum = 0
        for particle in self.particle_cloud:
            weighted_sum += particle.w
        for particle in self.particle_cloud:
            particle.w = particle.w / weighted_sum

    def publish_particles(self, timestamp):
        msg = ParticleCloud()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = timestamp
        for p in self.particle_cloud:
            msg.particles.append(Nav2Particle(pose=p.as_pose(), weight=p.w))
        self.particle_pub.publish(msg)

    def scan_received(self, msg):
        self.last_scan_timestamp = msg.header.stamp
        # we throw away scans until we are done processing the previous scan
        # self.scan_to_process is set to None in the run_loop
        if self.scan_to_process is None:
            self.scan_to_process = msg


def main(args=None):
    rclpy.init()
    n = ParticleFilter()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
