#!/usr/bin/env python
 
import rospy
import std_msgs
import tf
 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
 
pose = PoseWithCovarianceStamped()
 
def fnc_callback(odom):
    #build posewithcovariancestamped msg
    global pose
    pose.header = odom.header
    pose.pose = odom.pose
   
if __name__=='__main__':
    rospy.init_node('odom2pose')
   
    sub=rospy.Subscriber('odom', Odometry, fnc_callback)
    pub=rospy.Publisher('pose', PoseWithCovarianceStamped, queue_size=1000)
    rate=rospy.Rate(50)
 
    while not rospy.is_shutdown():
   
        pub.publish(pose)
        rate.sleep()
