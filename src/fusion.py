#!/usr/bin/env python

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovariance, TransformStamped, Vector3, Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf
import time

tracker_pose = None
amcl_orientation = None
tf_pub = None
sec_since_last_update = None
tf_listener = None
pose_pub = None

def tf_callback(msg):
    global tracker_pose
    global tf_listener
    if sec_since_last_update < rospy.Time.now():
        try:
            pose = tf_listener.lookupTransform("/map", "/vr_tracker_right", rospy.Time(0))
        except Exception as e:
            print(e)
            print("No Transform found for: vr_controller_right")
        tracker_pose = pose[0]
        publish_new_tf()

def amcl_callback(msg):
    global amcl_orientation
    pose = msg.pose.pose
    #pose = msg.pose.pose
    #print(pose)
    amcl_orientation = pose.orientation
    if sec_since_last_update < rospy.Time.now():
        publish_new_tf()

def publish_new_tf():
    global sec_since_last_update
    sec_since_last_update = rospy.Time.now()
    global tracker_pose
    global amcl_orientation
    global tf_pub
    global pose_pub

    pose_cov = PoseWithCovariance()

    tf_msg = TFMessage()
    msg = TransformStamped()
    vec = Vector3(tracker_pose[0], tracker_pose[1], 0)
    point = Point(tracker_pose[0],tracker_pose[1], 0)

    msg.transform.rotation = amcl_orientation
    msg.transform.translation = vec
    msg.child_frame_id = "base"
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()

    pose_cov.pose.position = point
    pose_cov.pose.orientation = amcl_orientation
    odom = Odometry()
    odom.pose = pose_cov
    odom.child_frame_id = "base"
    odom.header.frame_id = "map"
    odom.header.stamp = rospy.Time.now()


    tf_msg.transforms.append(msg)
    #print(tf_msg)
    print(odom)
    tf_pub.publish(tf_msg)
    #pose_pub.publish(odom)


if __name__ == '__main__':
    #global tf_pub
    rospy.init_node('fusion_tracking')
    rospy.Subscriber("/tf", TFMessage, tf_callback)
    rospy.Subscriber("/pepper_robot/amcl_pose", PoseWithCovarianceStamped, amcl_callback)
    tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=10)
    pose_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
    sec_since_last_update = rospy.Time.now()
    tf_listener = tf.TransformListener()
    time.sleep(0.5)
    rospy.spin()
