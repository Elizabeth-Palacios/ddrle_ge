#!/usr/bin/env python
# -*- coding: utf-8 -*-

#################################################################################
#Copyright 2022 Elizabeth
#
#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#distributed under the License is distributed on an "AS IS" BASIS,
#See the License for the specific language governing permissions and
#limitations under the License.
#################################################################################

import numpy as np
import matplotlib.pyplot as plt
import sys


class pathfinding(object):
    def __init__(self,laser_angles,x_min=-4,x_max=4,y_min=-4,y_max=4,size_s=0.07,debug=False):
        """
          Constructor
        """
        # Map stuff
        self.x_min           =    x_min
        self.x_max           =    x_max
        self.y_min           =    y_min
        self.y_max           =    y_max
        self.size_s          =    size_s
        self.rotate          =    lambda a: np.array([[np.cos(a),np.sin(a)],[-np.sin(a),np.cos(a)]])
        self.laser_angles    = laser_angles
        self.max_laser_range = 3.5
        self.__init_map(x_min,x_max,y_min,y_max,size_s)
        self.__debug         = debug
        if debug:
            dummy = self.box
            dummy[0,0]=2
            self.__debug_im   = plt.imshow(dummy,extent=(self.x_min,self.x_max,self.y_max,self.y_min),vmin=0,vmax=2)
            self.__scat_rob   = plt.scatter(0,0,marker="*",color="r")
            self.__scat_tar   = plt.scatter(0,0,marker="s",color="w")
            self.__scat_focus = plt.scatter(0,0,marker="d",color="g")

        pass

    def __init_map(self,x_min,x_max,y_min,y_max,size_s):
        """
          Initialize the map arrays. Use 3 arrays, one for the free space, one
          for the blocked space and a combined one.
        """
        # Save the x- and y- coordinates
        self.x_coordinates   = np.arange(x_min,x_max,size_s)
        self.y_coordinates   = np.arange(y_min,y_max,size_s)
        self.box             = np.zeros([len(self.x_coordinates),len(self.y_coordinates)])
        self.box_free        = np.zeros([len(self.x_coordinates),len(self.y_coordinates)])
        self.box_wall        = np.zeros([len(self.x_coordinates),len(self.y_coordinates)])


    def update_map(self,robot_position,target_position,heading,lasers):
        """
          Update the map.
            robot_position  - Tuple (or array) of x- and y- coordinate of the robot.
            target_position - Tuple (or array) of x- and y- coordinate of the goal.
            heading         - Heading of the robot.
            lasers          - Information of the laser distances (as array)
        """
        ro_pos         = np.array(robot_position)
        ta_pos         = np.array(target_position)
        hea            = heading
        l_d            = lasers
        vector         = (ta_pos-ro_pos)/np.linalg.norm(ta_pos-ro_pos)
        self.lis_laser = []
        for i,j in enumerate(l_d):
            rotation_angle = hea-self.laser_angles[i]
            matrix         = self.rotate(rotation_angle)
            rot_vector     = np.matmul(matrix,vector)*j + ro_pos
            mask_x         = np.argmin(abs(self.x_coordinates-rot_vector[0]))
            mask_y         = np.argmin(abs(self.y_coordinates-rot_vector[1]))
            s_piece_x      = np.linspace(ro_pos[0],rot_vector[0],1e2)
            s_piece_y      = np.linspace(ro_pos[1],rot_vector[1],1e2)
            mask2_x        = np.argmin(abs(self.x_coordinates-s_piece_x[:,np.newaxis]),axis=1)
            mask2_y        = np.argmin(abs(self.y_coordinates-s_piece_y[:,np.newaxis]),axis=1)
            mask_coor      = np.array([mask2_x,mask2_y])
            unique         = np.unique(mask_coor,axis=1)

            self.box_free[unique[0,:],unique[1,:]]+=1  # FREE
            if j !=self.max_laser_range:
                self.box_wall[mask_x,mask_y]+=50    # WALL
            self.lis_laser.append(rot_vector)
        self.box [self.box_free>=1]=1
        self.box[self.box_wall/self.box_free >7]=2


    def construct_path(self,robot_position,target_position):
        """
          Constructs a path from the robot to the target.
        """
        robot_position  = np.array(robot_position)
        target_position = np.array(target_position)
        # Get the position of the robot and target in box coordinates
        robot_position  = np.array([np.argmin(abs(self.x_coordinates-robot_position[0])),np.argmin(abs(self.y_coordinates-robot_position[1]))])
        target_position = np.array([np.argmin(abs(self.x_coordinates-target_position[0])),np.argmin(abs(self.y_coordinates-target_position[1]))])

        # Make the walls of the map bigger
        tmp_map              = np.zeros(self.box.shape)
        tmp_map[self.box==2] = 2
        for i in range(1,3):
            # i = 1
            tmp_map[i:,:]  = tmp_map[i:,:]+tmp_map[:-i,:]
            tmp_map[:-i,:] = tmp_map[i:,:]+tmp_map[:-i,:]
            tmp_map[:,i:]  = tmp_map[:,:-i]+tmp_map[:,i:]
            tmp_map[:,:-i] = tmp_map[:,:-i]+tmp_map[:,i:]
        tmp_map[tmp_map>2] = 2

        for i in range(1,5):
            tmp_map[target_position[0]-i,target_position[1]] = 0
            tmp_map[target_position[0]+i,target_position[1]] = 0
            tmp_map[target_position[0],target_position[1]-i] = 0
            tmp_map[target_position[0],target_position[1]+i] = 0
        tmp_map[robot_position[0],robot_position[1]] = 0
        self.tmp_map=tmp_map
        path = np.array(self.__a_star(tmp_map.T,tuple(robot_position),tuple(target_position)))
        # Translate to real coordinates again
        try:
            self.path_cords = np.array([self.x_coordinates[path[:,0]],self.y_coordinates[path[:,1]]]).T
        except:
            pass

    def follow_path(self,robot_position,target_position):
        """
          Gives a heading the robot has to make
        """
        try:
            # Make things to arrays
            robot_position  = np.array(robot_position)
            target_position = np.array(target_position)

            # Calculate closest point to the robot
            dists_rob       = np.sqrt((self.path_cords[:,1]-robot_position[0])**2.+(self.path_cords[:,0]-robot_position[1])**2.)
            idx_min         = np.argmin(dists_rob)
            self.path_cords = self.path_cords[(np.arange(len(self.path_cords))<idx_min)]

            # Sort the points
            foc_point_idx   = np.argsort((self.path_cords[:,1]-robot_position[0])**2.+(self.path_cords[:,0]-robot_position[1])**2.)
            foc_point       = self.path_cords[foc_point_idx][1]
            # Swap the axis
            foc_point       = np.array([foc_point[1],foc_point[0]])
            # self.__scat_focus.set_offsets(np.array([foc_point[1],foc_point[0]]))

            # Calculate everything of the triangle
            a               = np.linalg.norm(foc_point-robot_position)
            b               = np.linalg.norm(foc_point-target_position)
            c               = np.linalg.norm(target_position-robot_position)
            angle1          = np.arccos((b**2+c**2-a**2)/(2.0*b*c))
            angle2          = np.arccos((a**2+b**2-c**2)/(2.0*a*c))
            angle3          = np.pi-angle1-angle2

            # Angle 3 is the angle we need
            angle=angle3

            # If something goes wrong
            if np.isnan(angle):
                angle = 0

            # Check the sign of the angle, super stupid but I did not know how to make it better...
            vec = target_position-robot_position
            newpoint_positiv  = np.matmul(self.rotate(angle),vec)+robot_position
            newpoint_negative = np.matmul(self.rotate(-angle),vec)+robot_position
            d1 = np.linalg.norm(newpoint_positiv-foc_point)
            d2 = np.linalg.norm(newpoint_negative-foc_point)
            if d1<d2:
                return angle
            else:
                return -angle

        except (IndexError,AttributeError,ValueError) as e:
            return 0

    def monitor(self,robot_pos,target_pos):
        """
          Debug the code and show the map and path on a plot.
        """
        if self.__debug:
            try:
                self.__debug_im.set_array(self.tmp_map)
                self.__scat_rob.set_offsets(np.array([robot_pos[1],robot_pos[0]]))
                self.__scat_tar.set_offsets(np.array([target_pos[1],target_pos[0]]))
                # self.construct_path(robot_pos,target_pos)
                # print("sh",p.shape)
                plt.plot(self.path_cords[:,0],self.path_cords[:,1])
                plt.ion()
                plt.show(block=False)
                plt.pause(0.01)
            except:
                pass

    def __a_star(self,map,start,goal,eps=1.0):
        """
          Find a path to the goal.
          map   - 2D array, containing "2" for a wall.
          start - Start of the path (tuple).
          goal  - End of the path (tuple).
          eps   - Weighting of the heuristic function, 1 for normal A* finding.
        """
        def reconstruct_path(d):
            goal_swap = (goal[1],goal[0])
            start_swap = (start[1],start[0])

            full_path = [goal_swap]
            current   = goal_swap
            count=1
            while current !=start_swap:
                count+=1
                try:
                    current = d[current]
                except:
                    print(current,d)
                    sys.exit()
                full_path.append(current)
            return np.array(full_path)

        goal_swap  = (goal[1],goal[0])
        start_swap = (start[1],start[0])

        # Heuristic function, distance to the goal
        x_dim = map.shape[1]
        y_dim = map.shape[0]
        heuristic = np.zeros([x_dim,y_dim])

        heuristic = eps*np.sqrt((np.arange(x_dim)[:,np.newaxis]-goal[0])**2+(np.arange(y_dim)-goal[1])**2).T
                    # np.sqrt((np.arange(x_dim)[:,np.newaxis]-start[0])**2+(np.arange(y_dim)-start[1])**2)
        # Start at the start point
        openset = [start_swap]
        # Scores, initially infinity
        gscore = np.zeros(map.shape)+np.infty
        fscore = np.zeros(map.shape)+np.infty
        gscore[start_swap] = 0.
        fscore[start_swap] = heuristic[start_swap]

        # Empty dictionary
        camefrom = {}

        while len(openset)>0:
            # Get the best path
            tmp_score        = np.zeros(map.shape)+np.infty
            index            = tuple(zip(*openset))
            tmp_score[index] = fscore[index]
            current          = np.unravel_index(tmp_score.argmin(), tmp_score.shape)
            # print(fscore.min())
            # Found the goal, do whatever
            if current == goal_swap:
                return reconstruct_path(camefrom)

            # Remove the point from our investigation
            openset.remove(current)

            # Go through the neighbors
            for xtmp in range(-1,2):
                for ytmp in range(-1,2):
                    if xtmp == current[0] and ytmp == current[1]:
                        continue

                    neighbor        = (current[0]+xtmp, current[1]+ytmp)
                    if (neighbor[0]<0 or neighbor[0]>=y_dim) or \
                        (neighbor[1]<0 or neighbor[1]>=x_dim):
                        continue
                    if map[neighbor]==2:
                        continue
                    dist            = np.sqrt(xtmp**2+ytmp**2)
                    tentative_score = gscore[current]+dist
                    # Check if its the best ever guess
                    if tentative_score<gscore[neighbor]:
                        camefrom[neighbor] = current
                        gscore[neighbor]   = tentative_score
                        fscore[neighbor]   = tentative_score + heuristic[neighbor]
                        # print(fscore[current] ,fscore[neighbor] )
                        if not neighbor in openset:
                            openset.append(neighbor)
