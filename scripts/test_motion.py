import rospy
import tf
# import tf2_ros


from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int16, Bool
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
# from autonet_r1.srv import SetOdom, SetOdomResponse
# from autonet_r1.src.tools.tf_tools import *
import math

# from Motor2 import Motor
# from Odometry_calc import OdometryCalc
import json
# from PID import PID
odom_x = 0
odom_y = 0
odom_yaw = 0
odom_z = 0
def odom_clb(data):
    global odom_x, odom_y, odom_yaw, odom_z
    odom_x = data.pose.pose.position.x
    odom_y = data.pose.pose.position.y
    odom_z = data.pose.pose.position.z
    odom_yaw = tf.transformations.euler_from_quaternion([
        data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]

rospy.init_node("test_motion")

odom_sub = rospy.Subscriber("/odom", Odometry, odom_clb)


cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
tw = Twist()
rospy.sleep(2)
print("A")
tw.angular.z = -1
cmd_vel.publish(tw)
rospy.sleep(3)
tw.angular.z = 0
cmd_vel.publish(tw)

rospy.sleep(2)
print("L")
tw = Twist()
tw.linear.x = 0.1
cmd_vel.publish(tw)
rospy.sleep(7)
tw.linear.x = 0
cmd_vel.publish(tw)
rospy.sleep(2)
# d = 0.8

