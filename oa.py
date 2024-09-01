#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry  # Import Odometry message
from sensor_msgs.msg import LaserScan
import tf
from math import atan2, sqrt, pow, pi
from tf.transformations import euler_from_quaternion
import numpy as np
import time


kp_distance = 1
ki_distance = 0.01
kd_distance = 0.5

kp_angle = 1
ki_angle = 0.03
kd_angle = 0.05


class GotoPoint():
    def __init__(self):
        rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.odom_sub = rospy.Subscriber(
            'odom', Odometry, self.odom_callback)  # Subscribe to odometry topic
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        self.odom_data = None
        self.scan_data = None
        v_x = 0
        v_y = 0

        while self.odom_data is None:  # Wait for odometry data to be available
            rospy.loginfo("Waiting for odometry data...")
            r.sleep()
        while self.scan_data is None:
            rospy.loginfo("Waiting for scan data...")
            r.sleep()
        (position, rotation) = self.get_odom()

        last_rotation = 0

        (goal_x, goal_y, goal_z) = self.getkey()
        if goal_z > 180 or goal_z < -180:
            print("you input wrong z range.")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)

        goal_distance = sqrt(pow(goal_x - position.x, 2) +
                             pow(goal_y - position.y, 2))
        # distance is the error for length, x,y
        distance = goal_distance
        previous_distance = 0
        total_distance = 0

        previous_angle = 0
        total_angle = 0

        while distance > 0.05:
            (position, rotation) = self.get_odom()
            (data1, data2, data3) = self.get_scan()
            while ((data1 < 0.5) or (data2 < 0.5) or (data3 < 0.5)):
                print("obstacle_detected")
                move_cmd.linear.x = 0.22
                if (data2 < 0.5):
                    move_cmd.angular.z = -0.5
                    move_cmd.linear.y = -0.15
                elif (data3 < 0.5):
                    move_cmd.angular.z = 0.5
                    move_cmd.linear.y = 0.15
                else:
                    move_cmd.angular.z = -0.5
                    move_cmd.linear.y = -0.15
                self.cmd_vel.publish(move_cmd)
                (data1, data2, data3) = self.get_scan()
                time.sleep(2)
            x_start = position.x
            y_start = position.y
            # path_angle = error
            path_angle = atan2(goal_y - y_start, goal_x - x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation

            diff_angle = path_angle - previous_angle
            diff_distance = distance - previous_distance

            distance = sqrt(pow((goal_x - x_start), 2) +
                            pow((goal_y - y_start), 2))

            control_signal_distance = kp_distance*distance + \
                ki_distance*total_distance + kd_distance*diff_distance

            control_signal_angle = kp_angle*path_angle + \
                ki_angle*total_angle + kd_distance*diff_angle

            move_cmd.angular.z = (control_signal_angle) - rotation
            # move_cmd.linear.x = min(linear_speed * distance, 0.1)
            (v_x, v_y) = self.unicycle(control_signal_distance, path_angle)
            v_x=max(v_x,-0.1)
            v_y=max(v_y,-0.1)
            move_cmd.linear.x = min(v_x, 0.1)
            move_cmd.linear.y = min(v_y, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            previous_distance = distance
            total_distance = total_distance + distance
            print("Current position and rotation are: ", (position, rotation))

        (position, rotation) = self.get_odom()
        print("Current position and rotation are: ", (position, rotation))

        print("reached :)   ^_^")

        while abs(rotation - goal_z) > 0.05:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
            self.cmd_vel.publish(move_cmd)
            r.sleep()

        # rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        return

    def odom_callback(self, odom_msg):
        self.odom_data = odom_msg

    def scan_callback(self, scan_msg):
        self.scan_data = scan_msg
        # print(self.scan_data)

    def getkey(self):
        global x_input, y_input, z_input
        x = x_input
        y = y_input
        z = z_input
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z

    def get_odom(self):
        if self.odom_data is not None:
            x = self.odom_data.pose.pose.position.x
            y = self.odom_data.pose.pose.position.y
            orientation_q = self.odom_data.pose.pose.orientation
            orientation_list = [orientation_q.x,
                                orientation_q.y, orientation_q.z, orientation_q.w]
            (_, _, yaw) = euler_from_quaternion(orientation_list)
            return (Point(x, y, 0), yaw)
        else:
            rospy.logerr("Odometry data not available!")
            return (Point(), 0)

    def get_scan(self):
        if self.scan_data is not None:
            # print(self.scan_data.ranges[0],self.scan_data.ranges[15],self.scan_data.ranges[345])
            return (self.scan_data.ranges[0], self.scan_data.ranges[15], self.scan_data.ranges[345])
        else:
            print("scan_data is not available!")

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def unicycle(self,control_signal_distance, path_angle):
        v_x = control_signal_distance*(np.cos(path_angle))
        v_y = control_signal_distance*(np.sin(path_angle))
        return (v_x, v_y)


print("Enter final x position")
x_final = input()
print("Enter final y position")
y_final = input()
print("Enter final angle position")
angle_final = input()

final = [x_final, y_final, angle_final]
final_position = np.array(final)
x_input = final_position[0]
y_input = final_position[1]
z_input = final_position[2]
time.sleep(5)

while not rospy.is_shutdown():
    GotoPoint()
