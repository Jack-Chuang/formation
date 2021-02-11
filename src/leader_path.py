#!/usr/bin/env python 
import rospy
import math
from rospy.rostime import Duration
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class go_to_Goal:
    def __init__(self):
        #initialization
        self.follow = Twist()
        self.rate = rospy.Rate(10)
        self.sleeper = rospy.Rate(5)
        self.x, self.y, self.z, self.a1 = 0, 0, 0, 0

        #All the parameters
        self.name = rospy.get_param('~turtle')
        self.others1 = rospy.get_param('~other_turtle1')
        self.others2 = rospy.get_param('~other_turtle2')
        self.offset = rospy.get_param('~offset')

        #All the publisher
        self.pub_vel = rospy.Publisher('/%s/cmd_vel' % self.name, Twist, queue_size=1)

        #All the subscriber
        self.turtle = rospy.Subscriber('/%s/odom' % self.name, Odometry, self.turtle_odom)
        
        #generate trajectory
        self.traj = self.traj_trans(3, self.offset, 10)
        self.traj.pop(0)

        while not rospy.is_shutdown():
            for i in range(len(self.traj)):
                point = self.traj[i]
                if self.go_to_goal(point[0], point[1]):
                    continue
                else:
                    print ('invalid goal, return to last point')
                    if self.go_to_goal(self.traj[i-1][0],self.traj[i-1][1]):
                        continue

            self.follow.linear.x = 0
            self.follow.angular.z = 0
            self.pub = rospy.Publisher('/%s/cmd_vel' % self.name, Twist, queue_size=1)
            self.pub_vel.publish(self.follow)
            break

    def turtle_odom(self, msg):
        self.msg1 = msg
        turtle1_position = self.msg1.pose.pose.position
        turtle1_orientation = self.msg1.pose.pose.orientation
        self.x = turtle1_position.x
        self.y = turtle1_position.y
        self.z = turtle1_position.z
        turtle1_orientation_list = [turtle1_orientation.x, turtle1_orientation.y, turtle1_orientation.z, turtle1_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(turtle1_orientation_list)
        self.a1 = yaw

    def traj_trans(self, length, offset, num):
        #path generator
        dir = 1
        ans = [[0, offset]]
        for i in range(1, num):
            x = ans[-1][0]
            y = ans[-1][1]
            ans.append([x - offset + dir*length, y])
            ans.append([x - offset + dir*length, y - 2 * dir * offset + 3])
            # ans.append([0, ans[-1][1]])
            dir *= -1
        return ans

    def angle_trans(self, angle):
        #calculate the correct difference between two angles
        self.angle = angle
        while self.angle > math.pi:
            self.angle -= 2*math.pi
        while self.angle < -math.pi:
            self.angle += 2*math.pi
        return self.angle

    def go_to_goal(self, x_goal, y_goal):
        mission_time_begin = rospy.Time.now()

        #PID parameters
        K_angular = [1, 0.1, 4]
        K_linear = [1.2, 0.1, 4]
        I = 0 
        I_linear = 0
        I_angular = 0

        desired_angle_goal = math.atan2(y_goal - self.y, x_goal - self.x)
        angle = desired_angle_goal - self.a1
        angle = self.angle_trans(angle)

        #turn the robot until it faces the next waypoint
        while (abs(angle) > 0.01):
            desired_angle_goal = math.atan2(y_goal - self.y, x_goal - self.x)
            angle_0 = desired_angle_goal - self.a1
            time_begin = rospy.Time.now()
            self.rate.sleep()
            time_end = rospy.Time.now()
            duration = (time_end - time_begin).to_sec()
            desired_angle_goal = math.atan2(y_goal - self.y, x_goal - self.x)
            angle = desired_angle_goal - self.a1
            angle = self.angle_trans(angle)

            I = I + K_angular[1] * duration * (angle+angle_0)/2

            angular_speed = (angle * K_angular[0] + I + K_angular[1] * duration * (angle+angle_0)/2 + K_angular[2] * (angle-angle_0)/duration)
            linear_speed = 0
            if angular_speed > 0.2:
                angular_speed = 0.2
            if angular_speed < -0.2:
                angular_speed = -0.2

            self.follow.linear.x = linear_speed
            self.follow.angular.z = angular_speed
            self.pub_vel.publish(self.follow)  

        while (True):

            desired_angle_goal = math.atan2(y_goal - self.y, x_goal - self.x)
            angle_0 = desired_angle_goal - self.a1
            angle_0 = self.angle_trans(angle_0)
            distance_0 = abs(math.sqrt(((x_goal - self.x) ** 2) + ((y_goal - self.y) ** 2)))
            time_begin = rospy.Time.now()
            self.rate.sleep()
            time_end = rospy.Time.now()
            duration = (time_end - time_begin).to_sec()
            desired_angle_goal = math.atan2(y_goal - self.y, x_goal - self.x)
            angle = desired_angle_goal - self.a1
            angle = self.angle_trans(angle)
            distance = abs(math.sqrt(((x_goal - self.x) ** 2) + ((y_goal - self.y) ** 2)))

            #angular PID and linear PID
            I_angular =  I_angular + K_angular[1] * duration * (angle+angle_0)/2

            I_linear = I_linear + K_linear[1] * duration * (distance+distance_0)/2

            angular_speed =  (angle * K_angular[0] + I_angular + K_angular[1] * duration * (angle+angle_0)/2 + K_angular[2] * (angle-angle_0)/duration)
           
            linear_speed = distance * K_linear[0] + I_linear + K_linear[1] * duration * (distance+distance_0)/2 + K_linear[2] * (distance-distance_0)/duration

            #angular speed and linear speed control
            if linear_speed > 0.3:
                linear_speed = 0.3
            if angular_speed > 0.1:
                angular_speed = 0.1
            if angular_speed < -0.1:
                angular_speed = -0.1

            self.follow.linear.x = linear_speed
            self.follow.angular.z = angular_speed
            self.pub_vel.publish(self.follow)

            if (rospy.Time.now() - mission_time_begin).to_sec() > 60:
                return False
            if (distance < 0.02):
                self.follow.linear.x = 0
                self.follow.angular.z = 0
                self.pub_vel.publish(self.follow)
                self.rate.sleep()
                return True
            

if __name__ == '__main__':


    rospy.init_node('turtlesim_motion_pose', anonymous=True)
    try:
        f = go_to_Goal()


    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")