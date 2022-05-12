#!/usr/bin/env python
import rospy
import tf2_ros
import numpy as np
import tf2_geometry_msgs
import tf.transformations
from nav_msgs.msg import Odometry


def odom_callback(msg):
    br = tf.TransformBroadcaster()

    # Create 4x4 numpy optical to world transformation
    optical_to_world_numpy_rot = tf.transformations.quaternion_matrix([0, -0.1218693, 0, 0.9925462])
    optical_to_world_numpy_trans = tf.transformations.translation_matrix([-0.3, 0, 0])

    # Publish transform
    br.sendTransform(tf.transformations.translation_from_matrix(optical_to_world_numpy_trans),
                     tf.transformations.quaternion_from_matrix(optical_to_world_numpy_rot),
                     rospy.Time.now(),
                     "base",
                     "t265_pose_frame")

def main():
    # Create ROS node
    rospy.init_node('tf_sensors')

    # Subscribe to VIO odom topic
    rospy.Subscriber('/t265/odom/sample', Odometry, odom_callback)

    # Spin it
    rospy.spin()


if __name__ == "__main__":
    main()
