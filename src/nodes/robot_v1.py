#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
from pathfinding import pathfinding
import time
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from environment_v1 import Behaviour

class Robot():
    """
    Data robot
    """

    def __init__(self, number_action, environment, min_action_time=0.25):
        self.number_action              = number_action
        self.environment                = environment
        self.reset_proxy                = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy              = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy                = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.pub_cmd_vel                = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom                   = rospy.Subscriber('odom', Odometry, self.get_Odometry)
        # Acces to the robot position
        self.robot_position_i           = Pose()
        self.robot_position_y           = 0
        self.robot_position_x           = 0
        # Technical characteristics
        self.action                     = None
        self.heading                    = 0
        self.min_range                  = 0.18
        self._distancegoal              = 0.3
        self.__avoid_distance           = 1
        self.max_angular_vel            = 1

        # Avoid countinf multiple crashes en the same step
        self.__crashed                  = False
        self.__crash_counter            = 0
        # The minimum time one action is allowed to have
        self.__min_action_time          = min_action_time
        self.__action_time              = 0
        # Save the data laser to avoid asking ros all the time
        self.__step_cache               = -1
        self.step                       = 0
        # Defines specific movements
        self.__forward_action           = int((number_action-2)/2.)
        self.__backward_action          = int(number_action-1)
        self.__free_counter             = 0
        self.__process                  = "driving_to_goal"
        # Define an array with the velocite_angular that the robot has to move for step
        self.__angle_actions            = np.array([((self.number_action-4-action)* self.max_angular_vel*0.5)*self.__min_action_time for action in range(number_action-1)])
        self.__timer_list               = np.zeros(10)+min_action_time
        self.force_update               = False
        # Initialize pathfinding
        self.__pathfinding              = pathfinding(self.laser_angles/360.*2*np.pi)
        self.old_goal                   = (np.nan,np.nan)
        self.diff_time = 0.4
        self.last_win= True
        self.cn=0
        self.stand=True
    @property
    def angle_action(self):
        return np.array([((self.number_action-4-action)* self.max_angular_vel*0.5)*self.diff_time for action in range(self.number_action-1)])


    def evolve(self):
        """
          Make one step with the robot
        """


        # call the scan data method
        scan = np.array(self.scan_data)

        # Update the map, probably not necessary every step
        if (self.step % 1 == 0) and (self.step!=0):
            self.__pathfinding.update_map(self.position,self.environment.target_position.position,self.heading,scan)


        # Construct a path
        if (self.old_goal != self.environment.target_position.position) or (self.step % 5 == 0) and (self.__process == "follow_path"):
            self.__pathfinding.construct_path(self.position,self.environment.target_position.position)
            # print("Constructing path")

        # Finish the actual __process
        finished = False
        if self.__process=="collision":
            laser_angles = np.array(self.laser_angles)
            #laser_angles expressed in degrees
            if abs(laser_angles[np.argmin(scan)])>90:
                action = self.__forward_action
            else:
                action = self.__backward_action
            self.__coll_count +=1
        elif self.__process == "follow_path":
            self.__desired_angle = self.__pathfinding.follow_path(self.position,self.environment.target_position.position)
            action,_  = self.rotate_to_angle()
        elif self.__process== "driving_to_goal":
            self.__desired_angle = 0
            action,_  = self.rotate_to_angle()
            # self.__process= "follow_path"
        elif self.__process == "change_angle":
            action,finished = self.rotate_to_angle()

        # For debugging
        self.__pathfinding.monitor(self.position,self.environment.target_position.position)
        # print(self.__process,action)
        # Change the process
        self.change_process(finished)
        self.old_goal = self.environment.target_position.position
        return action

    def reset(self):
        """
          Reset the robot
        """
        self.__process      = "driving_to_goal"
        self.__free_counter = 0
        self.__coll_count   = 0

    def change_process(self, finished):
        """
          Change the optimal process to reach the goal
        """
        free_dist = self.free_dist_goal
        goal_dist = self.environment.get_current_Distance(self.robot_position_x,self.robot_position_y)

        if finished:
            self.__process = "follow_path"
        elif self.__process=="follow_path":
            # Only drive to goal if the distance to the goal is okay and not blocked
            if (free_dist>goal_dist):
                self.__desired_angle = self.__find_good_angle()
                self.__process="driving_to_goal"
        elif self.__process=="driving_to_goal":
            if self.status_regions["front"]<=self.__avoid_distance/1.5:
                self.__process="follow_path"
        elif self.__process=="collision":
            if self.__coll_count >= 15:
                # self.__desired_angle = self.parallel_angle
                self.__process="follow_path"

    @property
    def position(self):
        return (self.robot_position_x, self.robot_position_y)

    @property
    def process(self):
        return self.__process
    @process.setter
    def process(self,value):
        self.__process = value

    def __find_good_angle(self):
        '''
        Look for an obstacle free angle
        '''
        scan         = np.array(self.scan_data)
        laser_angles = np.array(self.laser_angles)
        mask         = scan>self.__avoid_distance
        indices      = np.arange(len(laser_angles))
        ii_g         = np.argsort(abs((np.deg2rad(laser_angles[mask])-self.heading)))[0]
        idx          = indices[mask][ii_g]
        towards_goal = np.deg2rad(laser_angles[idx])

        res = towards_goal
        if idx+1>=len(scan):
            nidx = -1
        else:
            nidx = idx+1

        if scan[idx-1]>scan[nidx]:
            res = res+np.deg2rad(laser_angles[idx]-laser_angles[idx-1])
        else:
            res = res+np.deg2rad(laser_angles[idx]-laser_angles[nidx])
        res=self.environment.fix_angle(res-self.heading)

        return res


    def rotate_to_angle(self):
        """
          Rotate to a given angle
        """
        eta=0
        # aa=self.__angle_actions+eta*self.__angle_actions
        aa=self.angle_action+eta*self.angle_action
        diff_angles = self.heading-self.__desired_angle
        if diff_angles<0:
            mask   = ((diff_angles-aa)<0)
        elif diff_angles>=0:
            mask   = (diff_angles-aa)>=0
        if abs(diff_angles) <np.deg2rad(10):
            helper = np.argmin(abs(diff_angles-aa[mask]))
            mask2=np.ones(len(aa),dtype=bool)
        else:
            mask2 =( np.arange(len(aa))==1 )| (np.arange(len(aa))==3)
            helper = np.argmin(abs(diff_angles-aa[mask&mask2]))
        action = np.arange(len(aa))[mask&mask2][helper]
        # print("change_angle",action,self.heading-self.__desired_angle)
        return action, (action==2)


    def perform_action(self, action):
        '''
        Calculates the time needed to excecute the actions
        and executes them
        '''

        self.diff_time = time.time()-self.__action_time
        # if diff_time< self.__min_action_time:
            # print("sleeping:",self.__min_action_time-diff_time)
            # time.sleep(self.__min_action_time-diff_time)
        self.__action_time = time.time()

        max_angular_vel = 1.5
        if action != self.__backward_action:
            ang_vel = ((self.number_action - 1)/2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()
        # if  (self.old_goal != self.environment.target_position.position):
            # vel_cmd.linear.x = 0


        if action == self.__backward_action:
            vel_cmd.linear.x = -0.15
            vel_cmd.angular.z = 0

        elif action == self.__forward_action:
            vel_cmd.linear.x = 0.15
            vel_cmd.angular.z = 0
            self.stand=False

        else:
            if not self.stand:
                vel_cmd.linear.x = 0.15

            else:
                vel_cmd.linear.x = 0
            vel_cmd.angular.z = ang_vel

        # if self.last_win:
        #     vel_cmd.linear.x = 0
            # vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)

        # if self.environment.get_goalbox or self.environment.collision or self.cn<10:
        #      self.last_win=True
        #      self.cn+=1
        #      print("turn")
        # else:
        #     self.last_win=False



        return

    @property
    def scan_data(self):
        '''
        Get data of the laser
        '''
        if self.__step_cache == self.step and not self.force_update:
            scan_data=self.__scan_data_cache
        else:
            data = None
            while data is None:
                try:
                    data = rospy.wait_for_message('scan', LaserScan, timeout=5)
                except:
                    pass
            scan = data
            scan_data = []
            for i in range(len(scan.ranges)):
                if scan.ranges[i] == float("Inf"):
                    scan_data.append(3.5)
                elif np.isnan(scan.ranges[i]):
                    scan_data.append(0)
                else:
                    scan_data.append(scan.ranges[i])
            if np.any(np.isnan(np.array(scan_data))):
                raise Exception("it's nan sensor")

            self.__step_cache      = self.step
            self.__scan_data_cache = scan_data
            self.force_update      = False

        return scan_data

    def get_Odometry(self, odom):
        '''
        Position and orientation to the robot
        '''
        self.robot_position_i = odom.pose.pose.position
        self.robot_position_x =self.robot_position_i.x
        self.robot_position_y =self.robot_position_i.y
        robot_angle = odom.pose.pose.orientation
        angles_robot_list = [robot_angle.x, robot_angle.y, robot_angle.z, robot_angle.w]
        _, _, yaw = euler_from_quaternion(angles_robot_list)
        self.theta= yaw
        self.environment.goal_angle = math.atan2(self.environment.goal_y - self.robot_position_y, self.environment.goal_x - self.robot_position_x)
        self.heading = self.environment.goal_angle - yaw
        if self.heading > pi:
            self.heading -= 2 * pi
        elif self.heading < -pi:
            self.heading += 2 * pi

    @property
    def state(self):
        '''
        Get state of the robot
        '''
        current_distance= self.environment.get_current_Distance(self.robot_position_x,self.robot_position_y)
        heading = self.heading
        scan_data = self.scan_data
        done=False
        if ((self.min_range >= min(scan_data) > 0) and (not self.__crashed)) \
            or ((self.min_range >= min(scan_data) > 0) and (self.__crash_counter>5)) :
            done = True
            self.__crashed = True
            self.__crash_counter = 0
            self.__process = "collision"
            self.__coll_count = 0
        elif (self.min_range >= min(scan_data) > 0) and (self.__crashed):
             self.__crash_counter += 1
        elif (self.min_range < min(scan_data)):
             self.__crashed = False

        wall_dist = min(self.scan_data)
        obstacle_angle = np.argmin(self.scan_data)

        return scan_data + [heading, current_distance,wall_dist,obstacle_angle], done

    def next_values(self,action):
        '''
        Call reward function and return next_state, reward,done
        '''
        state, done = self.state
        reward = self.environment.set_reward(state, done, action)
        return np.asarray(state), reward, done


    @property
    def status_regions(self):
        scan_data =self.scan_data
        regions = {
        'right':  min(scan_data[16:20]),
        'sright':  max(scan_data[15:19]),
        'fright': min(scan_data[18:22]),
        'front':  min(min(scan_data[21:25]), min(scan_data[0:4])),
        'fleft':  min(scan_data[2:6]),
        'left':   min(scan_data[5:9]),
        'sleft':   max(scan_data[5:9]),
        'backl':  scan_data[11],
        'backr':  scan_data[13],
        'back':  scan_data[12],
        'stop': min(min(scan_data[21:25]),min(scan_data[0:3])) }
        return regions

    @property
    def laser_angles(self):
        """
          Returns the angles of the laser
        """
        scan_data = self.scan_data
        angles = 360./(len(scan_data)-1)*np.arange(len(scan_data))
        angles[angles>180] = angles[angles>180]-360
        return angles


    @property
    def free_dist_goal(self):
        """
        Calculates the free distance to the goal, using laser information
        """
        scan_data = np.array(self.scan_data)
        sortkey = np.argsort(abs(np.rad2deg(self.heading)-self.laser_angles))[0:3]

        return np.min(scan_data[sortkey])

    @property
    def parallel_angle(self):
        '''
        Calculates the parallel_angle from the robot to the obstacle
        '''
        scan_data = np.array(self.scan_data)
        scanpairs=[[0,1],[23,0],[1,2],[23,22],[2,3],[21,20],[23,1],[2,22]]
        # scanpairs=[[0,1],[22,23],[1,2],[22,21],[2,3],[21,20],[22,1],[2,21]]

        dists = scan_data[np.array(scanpairs).flatten()]
        med_dists = np.median(dists)
        std_dist = np.std(dists)
        all_angles = np.zeros(len(scanpairs))
        for ind,s in enumerate(scanpairs):
            a=scan_data[s[0]]
            b=scan_data[s[1]]
            if (a >med_dists+1.5*std_dist) or (b >med_dists+1.5*std_dist):
                 all_angles[ind] = np.nan
                 continue

            gamma = abs(self.laser_angles[s[0]]-self.laser_angles[s[1]])
            c = np.sqrt(a**2 +b**2 -2*a*b*np.cos(math.radians(gamma)))
            phi = np.rad2deg(np.arccos((b**2-c**2-a**2)/(-2*c*a)))
            par_angle = math.radians(180-gamma - phi + self.laser_angles[s[0]])
            regions = self.status_regions
            if regions['right'] < regions['left']:
                value_regions= -1
            else:
                value_regions=1
            all_angles[ind] = par_angle*value_regions
        return np.nanmedian(all_angles)
