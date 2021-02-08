#!/usr/bin/env python  
import roslib
import rospy
import tf
from nav_msgs.msg import Odometry

def pose_callback(msg, turtlename):
    #create a transform broadcaster
    transform_broadcaster = tf.TransformBroadcaster()
    #convert 90 degrees to quaternion
    orientation = msg.pose.pose.orientation
    position = msg.pose.pose.position
    rotation_quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    #translation vector
    translation_vector = (position.x, position.y, 0)
    #time
    current_time = rospy.Time.now()

    transform_broadcaster.sendTransform(translation_vector, rotation_quaternion,
        current_time, turtlename+"_frame", "world")




if __name__ == '__main__':
    #get the parameter of the turtle name 
    turtlename = rospy.get_param('~turtle')
    #init the node
    rospy.init_node('turtle_tf_broadcaster')
    #subscribe to the Pose topic
    rospy.Subscriber('/%s/pose' % turtlename, Odometry, pose_callback, turtlename)
    rospy.spin()
