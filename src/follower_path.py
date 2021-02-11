#!/usr/bin/env python 
import rospy
import math
import matplotlib.pyplot as plt
from rospy.rostime import Duration
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Follow:

    def __init__(self):
        #initialization
        self.follow = Twist() 
        self.rate = rospy.Rate(20)
        self.vel_1_angular = 0
        self.vel_1_linear = 0
        self.count = 0
        self.x1, self.y1, self.z1, self.a1, self.x2, self.y2, self.z2, self.a2 = 0, 0, 0, 0, 0, 0, 0, 0

        #All the parameters
        self.l_d = rospy.get_param('~distance')
        self.phi_d = math.radians(rospy.get_param('~bear_angle'))
        self.leadername = rospy.get_param('~leader_turtle')
        self.followername = rospy.get_param('~follower_turtle')

        #All the publisher
        self.pub = rospy.Publisher('/%s/cmd_vel' % self.followername, Twist, queue_size=1)

        #All the subscriber
        self.turtle1 = rospy.Subscriber('/%s/odom' % self.leadername, Odometry, self.turtle1_odom)
        self.turtle2 = rospy.Subscriber('/%s/odom' % self.followername, Odometry, self.turtle2_odom)
        self.leader_speed = rospy.Subscriber('/%s/cmd_vel' % self.leadername, Twist, self.turtle_vel_1)
        
        self.log = []

        while not rospy.is_shutdown():
            
            k = 0
            # calculate desire postures p_d
            if self.vel_1_linear == 0:
                if self.phi_d * self.vel_1_angular < 0:
                    k = 0
                else:
                    k = 1

            #implement LYAPUNOV STABLE TIME VARYING STATE TRACKING CONTROLLER
            x_d_0 = self.x1 + self.l_d * math.cos(self.phi_d + self.a1)
            y_d_0 = self.y1 + self.l_d * math.sin(self.phi_d + self.a1)

            time_begin = rospy.Time.now()
            self.rate.sleep()
            time_end = rospy.Time.now()
            duration = (time_end - time_begin).to_sec()
            if duration == 0:
                continue

            x_d_1 = self.x1 + self.l_d * math.cos(self.phi_d + self.a1)
            y_d_1 = self.y1 + self.l_d * math.sin(self.phi_d + self.a1)

            self.rate.sleep()

            x_d_2 = self.x1 + self.l_d * math.cos(self.phi_d + self.a1)
            y_d_2 = self.y1 + self.l_d * math.sin(self.phi_d + self.a1)


            vx_d_0 = (x_d_1 - x_d_0) / duration
            vy_d_0 = (y_d_1 - y_d_0) / duration

            vx_d_1 = (x_d_2 - x_d_1) / duration
            vy_d_1 = (y_d_2 - y_d_1) / duration

            ax_d = (vx_d_1 - vx_d_0) / duration
            ay_d = (vy_d_1 - vy_d_0) / duration

            theta_d = math.atan2(vy_d_1, vx_d_1) + k*math.pi

            # nominal feedforward commands
            v_d = math.sqrt(math.pow(vx_d_1, 2) + math.pow(vy_d_1, 2))

            if k == 1:
                v_d *= -1
            
            if (math.pow(vx_d_1, 2) + math.pow(vy_d_1, 2)) == 0:
                w_d = 0
            else:
                w_d = (vx_d_1 * ay_d - vy_d_1 * ax_d) / (math.pow(vx_d_1, 2) + math.pow(vy_d_1, 2))

            # posture error
            x_e = x_d_2 - self.x2
            y_e = y_d_2 - self.y2
            theta_e = self.angle_trans(theta_d - self.a2)
            if abs(theta_e) > 3:
                continue

            # convert to follower local coordinate
            e_x = math.cos(self.a2) * x_e + math.sin(self.a2) * y_e
            e_y = -math.sin(self.a2) * x_e + math.cos(self.a2) * y_e
            e_theta = theta_e
            
            # constant
            eps = 1.4
            b = 10
            k_x = 2*eps*math.sqrt(math.pow(w_d, 2) + b * math.pow(v_d, 2))
            k_y = b*abs(v_d)

            x_vel2 = v_d * math.cos(e_theta) + k_x * e_x
            if e_theta == 0:
                a_vel2 = 0
            else:
                a_vel2 = w_d + k_y * e_y * math.sin(e_theta) / e_theta + k_x * e_theta
            
            if x_vel2 > 0.5:
                x_vel2 = 0.5
            if x_vel2 < -0.5:
                x_vel2 = -0.5
            if a_vel2 > 0.35:
                a_vel2 = 0.35
            if a_vel2 < -0.35:
                a_vel2 = -0.35

            if self.vel_1_linear < 0.02 and (abs(self.vel_1_angular) < 0.02):
                 a_vel2 = 0 
                 x_vel2 = 0

            self.follow.linear.x = x_vel2
            if abs(x_vel2) < 0.01:
                self.follow.linear.x = 0
            self.follow.linear.y = 0
            self.follow.linear.z = 0
            self.follow.angular.x = 0
            self.follow.angular.y = 0
            self.follow.angular.z = a_vel2
            self.pub.publish(self.follow)
            
            self.count+=1
            self.log.append([e_x,e_y,theta_e])
            if self.count > 50:
                plt.plot(self.log[1:])
                plt.legend(['e_x','e_y','e_theta'])
                plt.savefig('/home/jack/catkin_ws/src/formation/error_plot/%s_follower_data.png' % self.followername)
                plt.clf()
                self.count = 0

    def turtle1_odom(self, msg):
        self.msg1 = msg
        turtle1_position = self.msg1.pose.pose.position
        turtle1_orientation = self.msg1.pose.pose.orientation
        self.x1 = turtle1_position.x
        self.y1 = turtle1_position.y
        self.z1 = turtle1_position.z
        turtle1_orientation_list = [turtle1_orientation.x, turtle1_orientation.y, turtle1_orientation.z, turtle1_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(turtle1_orientation_list)
        self.a1 = self.angle_trans(yaw)

    def turtle2_odom(self, msg):
        self.msg2 = msg
        turtle2_position = self.msg2.pose.pose.position
        turtle2_orientation = self.msg2.pose.pose.orientation
        self.x2 = turtle2_position.x
        self.y2 = turtle2_position.y
        self.z2 = turtle2_position.z
        turtle2_orientation_list = [turtle2_orientation.x, turtle2_orientation.y, turtle2_orientation.z, turtle2_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(turtle2_orientation_list)
        self.a2 = self.angle_trans(yaw)
    
    def turtle_vel_1(self, msg):
        self.msg111 = msg
        self.vel_1_linear = self.msg111.linear.x
        self.vel_1_angular = self.msg111.angular.z

    def angle_trans(self, angle):
        #calculate the correct difference between two angles
        self.angle = angle
        while self.angle > math.pi:
            self.angle -= 2*math.pi
        while self.angle < -math.pi:
            self.angle += 2*math.pi
        return self.angle

if __name__=='__main__':
    # get the parameter of the turtle name

    rospy.init_node('two_turtle_2')
    try:
        f = Follow()
    except rospy.ROSInterruptException:  
        pass
