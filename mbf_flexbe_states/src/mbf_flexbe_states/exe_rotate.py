#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached

from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import Twist

import rospy
import tf

class ExeRotateState(EventState):

    def __init__(self, cmd_topic, Kp, max_vel, thr_rad):
        super(ExeRotateState, self).__init__(output_keys = ['outcome'],
                                                input_keys = ['path'],
                                                outcomes = ['done'])

        # Controller gain
        self._Kp = Kp
        self._max_vel = max_vel

        # Controller threshold
        self._thr = thr_rad

        # Velocity command interface
        self._cmd_topic = cmd_topic
        self._pub = ProxyPublisher({self._cmd_topic: Twist}) # pass required clients as dict (topic: type)

        # Current state feedback interface
        self._tf_listener = tf.TransformListener()
        self._error = False

    def execute(self, userdata):
        
        if self._error:
            Logger.loginfo("Reading current orientation is failed!")
            return 'done'
        else:
            # check goal is achieved
            if abs(self.goal_pose - self.cur_pose) < self._thr :
                return 'done'

            # control
            cmd = Twist()
            cmd.linear.x = 0
            cmd.angular.z = self._Kp*(self.goal_pose - self.cur_pose)

            if cmd.angular.z > self._max_vel :
                cmd.angular.z = self._max_vel
            elif cmd.angular.z < -self._max_vel :
                cmd.angular.z = -self._max_vel

            self._pub.publish(self._cmd_topic, cmd)

        try:
            # get pose feed back
            [cur_trans, cur_quat] = self._tf_listener.lookupTransform('/map','/base_footprint',rospy.Time(0))
            cur_euler = tf.transformations.euler_from_quaternion( cur_quat )
            self.cur_pose = cur_euler[2]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self._error = True

    def on_enter(self, userdata):

        # Set goal orientation as last element of given path
        goal_euler = self.QuaternionToEuler( userdata.path.poses[0].pose.orientation )
        self.goal_pose = goal_euler[2]

        # Check current pose can be read
        try:
            [cur_trans, cur_quat] = self._tf_listener.lookupTransform('/map','/base_footprint',rospy.Time(0))
            cur_euler = tf.transformations.euler_from_quaternion( cur_quat )
            self.cur_pose = cur_euler[2]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self._error = True

    def on_exit(self, userdata):
        pass

    def QuaternionToEuler(self, quat):
        return tf.transformations.euler_from_quaternion( [quat.x, quat.y, quat.z, quat.w] )