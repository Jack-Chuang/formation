#!/usr/bin/env python 
import rospy
import math
#import sympy as sym
from rospy.rostime import Duration
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Follow:

    def __init__(self):
        self.follow = Twist() 
        self.rate = rospy.Rate(10)
        self.l_d = rospy.get_param('distance')
        self.phi_d = math.radians(rospy.get_param('bear_angle'))
        self.leadername = rospy.get_param('leader_turtle')
        self.followername = rospy.get_param('follower_turtle')
        self.x2, self.y2, self.z2, self.a2, self.x3, self.y3, self.z3, self.a3 = 0, 0, 0, 0, 0, 0, 0, 0
        self.turtle2 = rospy.Subscriber('/%s/odom' % self.leadername, Odometry, self.turtle2_odom)
        self.turtle3 = rospy.Subscriber('/%s/odom' % self.followername, Odometry, self.turtle3_odom)
        self.pub = rospy.Publisher('/%s/cmd_vel' % self.followername, Twist, queue_size=1)

        while not rospy.is_shutdown():

            # calculate desire postures p_d
            x_d_0 = self.x2 + self.l_d * math.cos(self.phi_d + self.a2)
            y_d_0 = self.y2 + self.l_d * math.sin(self.phi_d + self.a2)

            time_begin = rospy.Time.now()
            self.rate.sleep()
            time_end = rospy.Time.now()
            duration = (time_end - time_begin).to_sec()

            x_d_1 = self.x2 + self.l_d * math.cos(self.phi_d + self.a2)
            y_d_1 = self.y2 + self.l_d * math.sin(self.phi_d + self.a2)

            self.rate.sleep()

            x_d_2 = self.x2 + self.l_d * math.cos(self.phi_d + self.a2)
            y_d_2 = self.y2 + self.l_d * math.sin(self.phi_d + self.a2)


            vx_d_0 = (x_d_1 - x_d_0) / duration
            vy_d_0 = (y_d_1 - y_d_0) / duration

            vx_d_1 = (x_d_2 - x_d_1) / duration
            vy_d_1 = (y_d_2 - y_d_1) / duration

            ax_d = (vx_d_1 - vx_d_0) / duration
            ay_d = (vy_d_1 - vy_d_0) / duration

            theta_d = math.atan2(vy_d_1, vx_d_1)

            # nominal feedforward commands
            v_d = math.sqrt(math.pow(vx_d_1, 2) + math.pow(vy_d_1, 2))
            if (math.pow(vx_d_1, 2) + math.pow(vy_d_1, 2)) == 0:
                w_d = 0
            else:
                w_d = (vx_d_1 * ay_d - vy_d_1 * ax_d) / (math.pow(vx_d_1, 2) + math.pow(vy_d_1, 2))

            # posture error
            x_e = x_d_2 - self.x3
            y_e = y_d_2 - self.y3
            theta_e = theta_d - self.a3
            # convert to follower local coordinate
            e_x = math.cos(self.a3) * x_e + math.sin(self.a3) * y_e
            e_y = -math.sin(self.a3) * x_e + math.cos(self.a3) * y_e
            e_theta = theta_e
            
            # constant
            eps = 0.9
            b = 15
            k_x = 2*eps*math.sqrt(math.pow(w_d,2) + b * math.pow(v_d,2))
            k_y = b*abs(v_d)

            x_vel3 = v_d * math.cos(e_theta) + k_x * e_x
            if (e_theta + k_x * e_theta) == 0:
                a_vel3 = 0
            else:
                a_vel3 = w_d + k_y * e_y * math.sin(e_theta) / e_theta + k_x * e_theta
            
            if x_vel3 > 0.5:
                x_vel3 = 0.5
            if a_vel3 > 0.2:
                a_vel3 = 0.2

            self.follow.linear.x = x_vel3
            self.follow.linear.y = 0
            self.follow.linear.z = 0
            self.follow.angular.x = 0
            self.follow.angular.y = 0
            self.follow.angular.z = a_vel3
            self.pub.publish(self.follow)
        

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
    # get the parameter of the turtle name

    rospy.init_node('three_turtle_3')
    try:
        f = Follow()
    except rospy.ROSInterruptException:  
        pass