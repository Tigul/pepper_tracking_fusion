import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Vector3
from nav_msgs.msg import Odometry

tracker_pose = None
amcl_orientation = None
tf_pub = None
sec_since_last_update = None

def tf_callback(msg):
    global tracker_pose
    if not msg.transforms ==  []:
        transform = msg.transforms[0]
        if transform.child_frame_id == 'base_link':
            tracker_pose = transform.transform.translation
            if sec_since_last_update < rospy.Time.now():
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


    tf_msg = TFMessage()
    msg = TransformStamped()
    #vec = Vector3(tracker_pose.x, tracker_pose.y, tracker_pose.z)

    msg.transform.rotation = amcl_orientation
    msg.transform.translation = tracker_pose
    msg.child_frame_id = "b"
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()

    tf_msg.transforms.append(msg)
    #print(tf_msg)
    tf_pub.publish(tf_msg)


if __name__ == '__main__':
    #global tf_pub
    rospy.init_node('fusion_tracking')
    rospy.Subscriber("/tf", TFMessage, tf_callback)
    rospy.Subscriber("/odom", Odometry, amcl_callback)
    tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=10)
    sec_since_last_update = rospy.Time.now()
    rospy.spin()
