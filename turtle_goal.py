#!/usr/bin/env python

import rospy
import tf2_ros
from turtlesim.msg import Pose as TurtlePose
import geometry_msgs
from nav_msgs.msg import Path
import tf_conversions
from geometry_msgs.msg import PoseStamped, Twist
import math
import numpy as np

class Planner:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        rospy.init_node('turtle_goal')
        self.sub_pose = rospy.Subscriber('turtle1/pose', TurtlePose, self.pose_callback)
        self.sub_goal = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.goal_callback)
        self.pub_pose =rospy.Publisher('turtle1/pose_stamped', PoseStamped, queue_size=10)
        self.pub_path =rospy.Publisher('turtle1/path', Path, queue_size=10)
        self.pub_cmd =rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
        self.goal = None
        self.pose = None

    def compute_command(self):    
        if self.pose is None or self.goal is None:
            return
        x_goal = self.goal.pose.position.x
        y_goal = self.goal.pose.position.y
        euler_goal = tf_conversions.transformations.euler_from_quaternion([
            self.goal.pose.orientation.x,
            self.goal.pose.orientation.y,
            self.goal.pose.orientation.z,
            self.goal.pose.orientation.w,
            ])
        theta_goal = euler_goal[2]
        rospy.logdebug('received goal {:g} {:g} {:g}'.format(x_goal,y_goal,theta_goal))
        
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        euler = tf_conversions.transformations.euler_from_quaternion([
            self.pose.pose.orientation.x,
            self.pose.pose.orientation.y,
            self.pose.pose.orientation.z,
            self.pose.pose.orientation.w,
            ])
        theta = euler[2]
        
        cmd = Twist()
        theta_error = theta_goal - theta
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]])
        v = R.T.dot(np.array([x_goal - x, y_goal - y]))
        x_error = v[0]
        y_error = v[1]

        dir_goal = math.atan2(y_error,x_error)
        delta = dir_goal - theta
        dist = math.sqrt(x_error**2 + y_error**2)
        rospy.loginfo('{:g} {:g} {:g} {:g}'.format(x_error,y_error,theta_error,delta))
        if dist > 0.1:
            cmd.angular.z = 0.5 * dir_goal
            cmd.linear.x = 0.5*max(min(dist,1),0)
        else:
            cmd.angular.z = 0.5 * (theta_goal-theta)
        self.pub_cmd.publish(cmd)

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.compute_command()
            rate.sleep()

    def pose_callback(self,msg):
        # rospy.loginfo('turtle pose {:g} {:g}'.format(msg.data.x, msg.data.y))  
        rospy.logdebug('received turtle pose {:g} {:g} {:g}'.format(msg.x,msg.y,msg.theta))
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = msg.x
        pose.pose.position.y = msg.y
        pose.pose.position.z = 0
        pose.pose.orientation.w = math.cos(msg.theta/2)
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = math.sin(msg.theta/2)
        self.pub_pose.publish(pose)
        self.pose = pose
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id = 'turtle1'
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)

    def goal_callback(self,msg):
        self.goal = msg
        #compute path
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'map'
        if self.pose is not None:
            path.poses.append(self.pose)
            path.poses.append(self.goal)
            self.pub_path.publish(path)


planner = Planner()
planner.loop()
