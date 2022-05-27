#!/usr/bin/env python
#################################################################################

import rospy
import numpy as np
import time
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

class Target(object):
    def __init__(self):
        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.modelPath = self.modelPath.replace('ddrl_ge/src/nodes',
                                                'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_square/goal_box/model.sdf')
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()
        self.target_position = Pose()
        # FOR BOX
        self.init_goal_x = -0.5
        self.init_goal_y = 0.5
        # FOR CORRIDOR
        # self.init_goal_x = 0.5
        # self.init_goal_y = 0.5
        self.target_position.position.x = self.init_goal_x
        self.target_position.position.y = self.init_goal_y
        self.modelName = 'goal'
        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y
        self.last_index = 0
        self.check_model = False
        self.index = 0


    def respawnModel(self):
        while True:
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(self.modelName, self.model, 'robotos_name_space', self.target_position, "world")
                rospy.loginfo("Goal position : %.1f, %.1f ", self.target_position.position.x,
                              self.target_position.position.y )
                break
    def deleteModel(self):
        while True:
                rospy.wait_for_service('gazebo/delete_model')
                del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                del_model_prox(self.modelName)
                break

    def getPosition(self, position_check=False, delete=False):
        if delete:
            self.deleteModel()
        while position_check:

            #goal for box
            goal_x_list = [-1, 1.8, 0.6, 1.9,  0.7, 0.2, -1.3, -1, -1.9,  0.5,   2, 0.5, 0, -0.1, -2,  -0.5]
            goal_y_list = [-2,-1.8, 0,  -0.5, -1.9, 1.5, -0.9,  1,  1.1, -1.5, 1.5, 1.8, -1, 1.6, -0.8, 0.5]

            # goal for corridor
            # goal_x_list = [0.5,1,    2  ,3 , 4  ,7 , 9  ,  12,   4.5, 7.5,-1,-4, -1,-5,  -7.5,-9,-10,-11,9.5]
            # goal_y_list = [0.5,1,   0.5, 1,-0.8, 0.8,  -1.5,0,  -1.5,-1.5,-1,-0.5,1,-1.2,1,   -1, 1,  0, 1.5]

            self.index = np.random.randint(0, len(goal_y_list), 1)[0]


            # self.index = random.randrange(0, len(goal_y_list))
            if self.last_index == self.index:
                position_check = True
            else:
                self.last_index = self.index
                position_check = False

            self.target_position.position.x = goal_x_list[self.index]
            self.target_position.position.y = goal_y_list[self.index]

        time.sleep(0.5)
        self.respawnModel()

        self.last_goal_x = self.target_position.position.x
        self.last_goal_y = self.target_position.position.y

        return self.target_position.position.x, self.target_position.position.y

    @property
    def position(self):
        return (self.target_position.position.x ,self.target_position.position.y)
