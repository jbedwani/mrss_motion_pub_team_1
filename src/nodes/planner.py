#!/usr/bin/env python3
# Reads the map output (see map_broadcaster.py) and publishes twist commands to reach the goal

import numpy as np
from potential_field_planner import PotentialFieldPlanner
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import json


class Planner:
    def __init__(self):
        super().__init__()

        # Initialize Node
        rospy.init_node('Planner', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        # Initialize subscriber
        self.map_sub = rospy.Subscriber('/map', String, self.map_callback)
        self.map = None
        
        # Intialize publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        self.cmd = None
        self.rate = rospy.Rate(10)  # Publisher frequency

       # set the planner attributes
        self.time_step = 0.001
        self.k_att     = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0.5]])
        self.k_rep     = 1
        self.vel_max   = 0.5
        self.planner = None
        # END MRSS

    def map_callback(self, msg):
        self.map = json.loads(msg.data)

        # TODO BEGIN MRSS: Use map for planning
        goal = self.map[list(self.map.keys())[-1]]
        goal_pos = np.array(goal['trans'])
        obs_1 = self.map[list(self.map.keys())[-4]]
        obs_1_pos = np.array(obs_1['trans'])
        robot = self.map[list(self.map.keys())[0]]
        robot_pos = self.map[list(self.map.keys())[0]]['trans']


        if self.planner is not None and goal_pos != prev_goal_pos:
            self.planner   = PotentialFieldPlanner(goal_pos, 1/self.rate, self.k_att, self.k_rep, self.vel_max)
            self.planner.set_obstacle_distance(1.5)
            self.planner.set_obstacle_position(obs_1_pos)
        # END MRSS

        # Twist
        self.cmd = geometry_msgs.msg.Twist()

        # TODO BEGIN MRSS: Update the current command
        n_goal = np.linalg.norm(goal)
        if n_goal < 0.1: 
            self.cmd.linear.x = 0.
            self.cmd.linear.y = 0.
            self.cmd.angular.z = 0.
        else:
            # self.cmd.linear.x = goal[0]/n_goal * 0.15
            # self.cmd.linear.y = goal[1]/n_goal * 0.15
            # self.cmd.angular.z = 0.
            pos_des, lin_vel =	self.planner.get_avoidance_force(robot_pos)
            self.cmd.linear.x = lin_vel[0] * 0.15
            self.cmd.linear.y = lin_vel[1] * 0.15
            self.cmd.angular.z = 0.
            
        prev_goal_pos = goal_pos
        # END MRSS

    def spin(self):
        '''
        Spins the node.
        '''
        try:
            while not rospy.is_shutdown():
                if self.cmd is not None:
                    # Publish
                    self.cmd_pub.publish(self.cmd)
                else:
                    rospy.logwarn("SKIP")

                self.rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down planner.")

    def on_shutdown(self):
        '''
        Called on node shutdown.
        '''
        pass


if __name__ == '__main__':
    try:
        node = Planner()
        node.spin()
    except rospy.ROSInterruptException:
        pass
