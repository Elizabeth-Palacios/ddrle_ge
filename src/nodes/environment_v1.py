#!/usr/bin/env python

import rospy
import numpy as np
import math
import json
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from std_msgs.msg import String,  Float32MultiArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from target_v1 import Target

class Behaviour(object):
    def __init__(self):
        self.pub_cmd_vel         = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.reset_proxy         = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy       = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy         = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.target_position     = Target()
        self.dirPath             = os.path.dirname(os.path.realpath(__file__))
        self.dirPath             = self.dirPath.replace('enviromentv1_/nodes', 'enviromentv1_/save_models/save_model_final')
        self.goal_x              = 0
        self.goal_y              = 0
        self.initial_steps       = 0
        self.initGoal            = True
        self.action              = int
        self.last_heading        = 0
        self.heading             = 0
        self.min_range           = 0.18
        self.goal_distance       = 0
        self.current_distance    = 0
        self.best_time           = 0
        # Crash fine tuning
        self._reverse            = -1
        self.__crashed           = False
        self.__crash_counter     = 0
        self._distancegoal       = 0.3
        self.initial_time        = time.time()
        self._maximo_reward      = 300#70
        self._maximo_reward_angle      =30#70
        self.turn                = np.ones(40)*2
        self.cont_step           = 0
        self.Exp_rq              = 0
        self.get_goalbox         = False

    def fix_angle(self,angle):
        """
          fix an angle to be between 180 and -180 degree
        """
        if angle > pi:
            angle -= 2 * pi
        elif angle < -pi:
            angle += 2 * pi
        return angle

    def get_Distance_goal(self,x,y):
        '''
        Calculate the initial distance to the goal
        '''
        self.goal_distance= (math.hypot(self.goal_x - x, self.goal_y - y))
        self._goal_distance_initial=self.goal_distance
        return self.goal_distance


    def get_current_Distance(self,x,y):
        '''
        Calculate the actual distance to the goal
        '''
        return math.hypot(self.goal_x - x, self.goal_y - y)

    def set_reward(self, state, done, action):
        '''
        Calculate reward(distance-angle-wall-time)
        scan_data + [heading, current_distance,wall_dist,obstacle_angle]
        '''
        heading             = state[-4]
        current_distance    = state[-3]
        wall_dist           = state[-2]
        goal_heading_initial= np.degrees(state[-1])
        # Save the last steps in an array
        self.turn[0:-1]     = self.turn[1:]
        self.turn[-1]       = action

        last_distance = self.goal_distance
        # print("last_heading: ",goal_heading_initial)
        # If the robot is close to the wall,
        # it loses points because that action can lead to a collision.
        # If the robot is close to the wall, it loses points because that
        # action can lead to a collision.
        if wall_dist<0.25:
            wall_reward = -5
        else:
            wall_reward = 0


        # Reward angle
        # if action ==2 :
        #     self.reward_current_angle = 0.0
        if action ==5 :
            self.reward_current_angle = 0.0
        else:
            # if current_distance <= 2*self._distancegoal:
                # self.reward_current_angle = (np.cos(-abs(heading)+abs(self.last_heading)))*np.sign(-abs(heading)+abs(self.last_heading))*1
                self.reward_current_angle = ((np.exp(-abs(np.degrees(self.last_heading))) - np.exp(-abs(np.degrees(heading))))/(np.exp(-abs(goal_heading_initial-6))-1))*self._maximo_reward_angle
        # else:)
            #     self.reward_current_angle = 0.0
        print(np.degrees(heading),np.degrees(self.last_heading),goal_heading_initial)
        # if (0<current_distance < 2*self._distancegoal):
        #      self.last_heading = math.pi
        # else:
        self.last_heading = heading

        #Reward goal and best time
        if (0<current_distance < self._distancegoal) and (-pi/2< heading <pi/2):
            # Calculate the goal_time
            self.initial_steps = (self._goal_distance_initial+0.7)/0.15
            #Calculate time used
            t_steps = time.time() - self.initial_time
            self.initial_time = time.time()
            #Calculate best_time
            self.best_time = self.initial_steps/t_steps
            if self.best_time > 1:
                self.best_time1 = 1
            else:
                self.best_time1 = -(1-self.best_time)
                #Reward best_time
            reward_bt = 100*self.best_time1
                #Reward goal
            print("win",self.best_time,self.best_time1,self.initial_steps,reward_bt,t_steps)
            self.get_goalbox = True
            self._cont_step = self.cont_step
            self.cont_step = 0
            print("goal",self.initial_steps,t_steps,self.best_time,reward_bt)

        if action ==5 :
            distance_rate = 0
        else:
            if abs(current_distance-self.goal_distance)>0.8*0.15: # avoid get negative reward for reset
                distance_rate =0
                self.reward_current_angle =0
            elif current_distance <self._distancegoal:
                distance_rate =0
            else:
                distance_rate = ((np.exp(-last_distance) - np.exp(-current_distance))/(np.exp(-(self._goal_distance_initial -self._distancegoal))-1))*self._maximo_reward
                # distance_rate = (last_distance-current_distance)*self._maximo_reward
<<<<<<< HEAD
        print("reward_distance: ",self._goal_distance_initial, last_distance, current_distance, distance_rate, "angle: ", heading, self.reward_current_angle, "wall_reward: ", wall_reward)
        self.goal_distance = current_distance
        # print("reward_distance: ", last_distance, current_distance, distance_rate, "angle: ", self.last_heading, heading, self.reward_current_angle, "wall_reward: ", wall_reward)
        # print("angle: ", self.last_heading, heading, np.degrees(heading),self.reward_current_angle)
=======
        print("reward_distance: ",self._goal_distance_initial, last_distance, current_distance, distance_rate, "angle: ", np.degrees(goal_heading_initial), np.degrees(self.last_heading), np.degrees(heading), self.reward_current_angle, "wall_reward: ", wall_reward,"action :",action)
        self.goal_distance = current_distance
>>>>>>> 2c945caeae0c71c2854785081f1e324ffd769773

        reward = distance_rate  + self.reward_current_angle +wall_reward
        # print(self.reward_current_angle,distance_rate,reward)

        #Reward collision
        if done:
            rospy.loginfo("Collision!!")
            reward = -1000
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 1000 +reward_bt
            self.pub_cmd_vel.publish(Twist())
            # print("regoal: ", reward, reward_bt)
        return reward

    def reset_gazebo(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

    def reset(self,x,y):
        if self.initGoal:
            self.goal_x, self.goal_y = self.target_position.getPosition()
            self.initGoal = False
        self.goal_distance = self.get_Distance_goal(x,y)
