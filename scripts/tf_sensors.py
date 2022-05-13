#!/usr/bin/env python
import rospy
import tf2_ros
import numpy as np
import tf2_geometry_msgs
import tf.transformations
from nav_msgs.msg import Odometry


def odom_callback(msg):
    br = tf.TransformBroadcaster()

    # T265 rot and translation
    t265_world_rot = tf.transformations.quaternion_matrix([0, -0.1218693, 0, 0.9925462])
    t265_world_trans = tf.transformations.translation_matrix([-0.33118, 0, -0.0045])

    # Publish static transform for t265 pose
    br.sendTransform(tf.transformations.translation_from_matrix(t265_world_trans),
                     tf.transformations.quaternion_from_matrix(t265_world_rot),
                     rospy.Time.now(),
                     "base",
                     "t265_link")

def main():
    # Create ROS node
    rospy.init_node('tf_sensors')

    # Subscribe to VIO odom topic
    rospy.Subscriber('/t265/odom/sample', Odometry, odom_callback)

    # Spin it
    rospy.spin()


if __name__ == "__main__":
    main()

