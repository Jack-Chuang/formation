#!/usr/bin/env python 
import rospy
import math
#import sympy as sym
from rospy.rostime import Duration
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Zigzag:
    
    def __init__(self):
        self.points = [[4, 0], [4, 2], [0, 2], [0, 4], [4, 4], [4, 6]]
        self.follow = Twist() 
        self.rate = rospy.Rate(10)
        self.firstname = rospy.get_param('turtle1')
        self.secondname = rospy.get_param('turtle2')
        self.thirdname = rospy.get_param('turtle3')
        self.k_linear = 4.0
        self.x1, self.y1, self.z1, self.a1 = 0, 0, 0, 0
        self.x2, self.y2, self.z2, self.a2 = 0, 0, 0, 0
        self.x3, self.y3, self.z3, self.a3 = 0, 0, 0, 0
        self.turtle1 = rospy.Subscriber('/%s/odom' % self.firstname, Odometry, self.turtle1_odom)
        self.turtle2 = rospy.Subscriber('/%s/odom' % self.secondname, Odometry, self.turtle2_odom)
        self.turtle3 = rospy.Subscriber('/%s/odom' % self.thirdname, Odometry, self.turtle3_odom)
        self.pub1 = rospy.Publisher('/%s/cmd_vel' % self.firstname, Twist, queue_size=1)
        self.pub2 = rospy.Publisher('/%s/cmd_vel' % self.secondname, Twist, queue_size=1)
        self.pub3 = rospy.Publisher('/%s/cmd_vel' % self.thirdname, Twist, queue_size=1)

        while not rospy.is_shutdown():

            #get the next goal point    
            for i in range(len(self.points) - 1):
                if (self.a1 == 0) or (self.a1 == 180):
                    goal_location_1_x = self.points[i][0]
                    goal_location_1_y = self.points[i][1]
                    goal_location_2_x = self.points[i][0]
                    goal_location_2_y = self.points[i][1] + 0.5                
                    goal_location_3_x = self.points[i][0]
                    goal_location_3_y = self.points[i][1] - 0.5

                if (self.a1 == 90) or (self.a1 == 270):
                    goal_location_1_x = self.points[i][0]
                    goal_location_1_y = self.points[i][1]
                    goal_location_2_x = self.points[i][0] - 0.5
                    goal_location_2_y = self.points[i][1]                  
                    goal_location_3_x = self.points[i][0] + 0.5
                    goal_location_3_y = self.points[i][1] 

                


        

    def turtle1_odom(self, msg):
        self.msg1 = msg
        turtle1_position = self.msg1.pose.pose.position
        turtle1_orientation = self.msg1.pose.pose.orientation
        self.x1 = turtle1_position.x
        self.y1 = turtle1_position.y
        self.z1 = turtle1_position.z
        turtle1_orientation_list = [turtle1_orientation.x, turtle1_orientation.y, turtle1_orientation.z, turtle1_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(turtle1_orientation_list)
        self.a1 = yaw
        

    def turtle2_odom(self, msg):
        self.msg2 = msg
        turtle2_position = self.msg2.pose.pose.position
        turtle2_orientation = self.msg2.pose.pose.orientation
        self.x2 = turtle2_position.x
        self.y2 = turtle2_position.y
        self.z2 = turtle2_position.z
        turtle2_orientation_list = [turtle2_orientation.x, turtle2_orientation.y, turtle2_orientation.z, turtle2_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(turtle2_orientation_list)
        self.a2 = yaw

    def turtle3_odom(self, msg):
        self.msg3 = msg
        turtle3_position = self.msg3.pose.pose.position
        turtle3_orientation = self.msg3.pose.pose.orientation
        self.x3 = turtle3_position.x
        self.y3 = turtle3_position.y
        self.z3 = turtle3_position.z
        turtle3_orientation_list = [turtle3_orientation.x, turtle3_orientation.y, turtle3_orientation.z, turtle3_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(turtle3_orientation_list)
        self.a3 = yaw
    

if __name__=='__main__':

    rospy.init_node('go_to_goal')
    try:
        f = Zigzag()
    except rospy.ROSInterruptException:  
        pass