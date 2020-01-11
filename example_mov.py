#!/usr/bin/env python
# coding: utf-8

import sys
import time
import threading
import pybullet as p
from qibullet import SimulationManager
from qibullet import PepperVirtual
from qibullet import NaoVirtual
from qibullet import RomeoVirtual
import pandas as pd
import math

class Robot_Setting():
    def __init__(self,robot_name,joint_parameters):
        self.name = robot_name
        self.joint_parameters = joint_parameters
        self.l_joint_parameter = self.get_joint_params('L')
        self.r_joint_parameter = self.get_joint_params('R')
        self.l_joint_angle_offset = self.get_joint_angle_offset()
        self.l_joint_axis_offset = self.get_joint_axis_offset()
        self.conversion_axis = self.get_conversion_axis()

    def get_joint_params(self,LorR):
        params = [None,None,None,None]
        #add offset when create new slide bar
        offset = 1
        if(self.name == 'nao'):
            params = [14,15,16,17] if(LorR=='L') else [20,21,22,23]
        elif(self.name == 'pepper'):
            params = [5,6,7,8] if(LorR=='L') else [11,12,13,14]
        elif(self.name == 'romeo'):
            params = [21,22,23,24] if(LorR=='L') else [29,30,31,32]
        for i in range(4):
            params[i] += offset
        return params

    #offset initial pose to make the robot equivalent as our model
    def get_joint_angle_offset(self):
        ##calibration ... needed up to the model for now. this param should be take into account
        #when set the offset!!
        adjust = math.pi/5
        adjust2 = math.pi/4
        ######
        offset = [None,None,None,None]
        if(self.name == 'nao'):
            return [math.pi/2,adjust,-math.pi/2,0.0]
        elif(self.name == 'pepper'):
            return [math.pi/2,adjust,-math.pi/2,0.0]
        elif(self.name == 'romeo'):
            return [math.pi/2,adjust2,-math.pi/2,0.0]
        else:
            return offset

    #offset axis direction to make the rotational direction to be same as calculated model
    def get_joint_axis_offset(self):
        offset = [None,None,None,None]
        if(self.name == 'nao'):
            return [-1.0,1.0,1.0,-1.0]
        elif(self.name == 'pepper'):
            return [-1.0,1.0,1.0,-1.0]
        elif(self.name == 'romeo'):
            return [-1.0,1.0,1.0,-1.0]
        else:
            return offset

    def get_conversion_axis(self):
        offset = [None,None,None,None]
        if(self.name == 'nao'):
            return [1.0,-1.0,-1.0,-1.0]
        elif(self.name == 'pepper'):
            return [1.0,-1.0,-1.0,-1.0]
        elif(self.name == 'romeo'):
            return [1.0,-1.0,-1.0,-1.0]
        else:
            return offset

    def convert_angle_to_match_the_robot(self,joint_index,angle):
        converted_angle =  (self.l_joint_axis_offset[joint_index] * angle + self.l_joint_angle_offset[joint_index])

        # print('joint_index           : ',joint_index)
        # print('offset-angle          : ',self.l_joint_angle_offset[joint_index])
        # print('offset-axis           : ',self.l_joint_axis_offset[joint_index])
        # print('angle                 : ',angle)
        # print('converted_angle : ',converted_angle)
        return float(converted_angle)

    def convert_to_R(self,angle,index):
        converted_angle = angle * self.conversion_axis[index]
        return float(converted_angle)

    def scaling_angle(self,joint_index,df,t,scale):
        angle = float(df['th' + str(joint_index+1)][t])
        #change scale up to the param of slidebar
        #since there's no prev_angle when t=0, get prev_angle t>0
        prev_angle = float(df['th' + str(joint_index+1)][t-1]) if(t > 0) else angle
        return prev_angle + scale * (angle - prev_angle)

    def move(self,robot,df,t,scale):

        for joint_parameter in self.joint_parameters:
            #left arm
            if(joint_parameter[0] in  self.l_joint_parameter):
                # print('Joint_name = ',joint_parameter[1])
                joint_index = self.l_joint_parameter.index(joint_parameter[0])

                #apply scale only th3,4
                angle = self.scaling_angle(joint_index,df,t,scale) if(joint_index >1) else float(df['th' + str(joint_index+1)][t])

                robot.setAngles(
                    joint_parameter[1],
                    self.convert_angle_to_match_the_robot(joint_index,angle),
                    1.0)
            #right arm
            if(joint_parameter[0] in  self.r_joint_parameter):
                # print('Joint_name = ',joint_parameter[1])
                joint_index = self.r_joint_parameter.index(joint_parameter[0])
                # angle = float(df['th' + str(joint_index+1)][t])

                #apply scale only elbow joint
                angle = self.scaling_angle(joint_index,df,t,scale) if(joint_index >1) else float(df['th' + str(joint_index+1)][t])
                robot.setAngles(
                    joint_parameter[1],
                    self.convert_to_R(self.convert_angle_to_match_the_robot(joint_index,angle),joint_index),
                    1.0)

def get_df(mode):
    path =  '/Users/atok/workspace/research/2018/training/kinematics/BDinNarrativeScene/GPy-result/6D/' + mode + '/sub01-09/Posterior/df' + mode + 'joint_angles.csv'
    return pd.read_csv(path)

if __name__ == "__main__":
    simulation_manager = SimulationManager()

    if (sys.version_info > (3, 0)):
        rob = input("Which robot should be spawned? (pepper/nao/romeo): ")
        agent_num = input("How many robots should be loaded? (1 or 2):")
    else:
        rob = raw_input("Which robot should be spawned? (pepper/nao/romeo): ")
        agent_num = input("How many robots should be loaded? (1 or 2):")

    client = simulation_manager.launchSimulation(gui=True)

    if rob.lower() == "nao":
        robot  = simulation_manager.spawnNao(client, spawn_ground_plane=True)
        if(agent_num == "2"):
            robot2 = simulation_manager.spawnNao(client, translation=[0, 0.5, 0],spawn_ground_plane=True)
    elif rob.lower() == "pepper":
        robot  = simulation_manager.spawnPepper(client, spawn_ground_plane=True)
        if(agent_num == "2"):
            robot2 = simulation_manager.spawnPepper(client, translation=[0, 1.0, 0],spawn_ground_plane=True)
    elif rob.lower() == "romeo":
        robot  = simulation_manager.spawnRomeo(client, spawn_ground_plane=True)
        if(agent_num == "2"):
            robot2 = simulation_manager.spawnRomeo(client, translation=[0, 1.0, 0],spawn_ground_plane=True)
    else:
        print("You have to specify a robot, pepper, nao or romeo.")
        simulation_manager.stopSimulation(client)
        sys.exit(1)

    time.sleep(1.0)
    joint_parameters = list()

    #create slidebar to change the scale of movement
    joint_parameters.append((
        p.addUserDebugParameter(
            'scale',
            1.0,
            50.0,
            1.0),
        'scale'))

    for name, joint in robot.joint_dict.items():
        if "Finger" not in name and "Thumb" not in name:
            joint_parameters.append((
                p.addUserDebugParameter(
                    name,
                    joint.getLowerLimit(),
                    joint.getUpperLimit(),
                    robot.getAnglesPosition(name)),
                name))

####################################
    #make robot instance
    robot_model = Robot_Setting(rob.lower(),joint_parameters)
    print(robot_model.name,' setting is loaded')

    df_N = get_df('N')
    df_P = get_df('P')

    print(joint_parameters)
    for i in range(len(df_P)):
        t1 = time.time()
        scale = p.readUserDebugParameter(0)
        # print('****** scale = ',scale)
        robot_model.move(robot,df_P,i,scale)
        if(agent_num == '2'):
            robot_model.move(robot2,df_N,i,scale)
        t2 = time.time()
        t = t2 - t1
        #adjust the publishing rate
        if(t < 0.1):
            time.sleep(0.1-t)
        t3 = time.time()
        print('process time: ',t3-t1)

    print('*****DONE')

    # while True:
    #     for joint_parameter in joint_parameters:
    #         robot.setAngles(
    #             joint_parameter[1],
    #             p.readUserDebugParameter(joint_parameter[0]), 1.0)
