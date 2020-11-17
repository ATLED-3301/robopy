#!/usr/bin/env python3

import numpy as np
import numpy as np 
from kinematichs.support.matrixs import mdot 
from kinematichs.support.base_func import jacobian_g
from kinematichs.support.base_func import jacobian_a 
from kinematichs.serial.parametrization import Joint
from kinematichs.support.base_func import euler_angles
import sys 
import time

class Parallel_chain():
    def __init__(self,serials,name,Tg = np.eye(4)):
        """CHAIN CLASS 
        ---
        Description:

            create parallel class for robotichs.
        ---
        Keyword arguments:

            serials --> list of serials connected to the parallel
            name --> name of the parallel

        ---
        Types:

            serial --> list
            name --> string
        ---
        Returns:
            parallel
        """
        
        self.Tg = Tg
        self.name = name
        self.serials = serials
        self.serial_names = self.get_serial_names(serials)
        self.n_joints = self.get_njoint()

    def get_njoint(self):
        n_joints = 0
        for serial in self.serials:
            n_joints += len(serial.q_list)            
        return n_joints 

    def get_serial_names(self,serial):
        names = []
        for s in self.serials:
            names.append(s.name)
        return names            

    def move_serial(self,serial_name,goals,active="all"):
        """ 
        Description:

            function to move single serial.
        ---
        Keyword arguments:

            serial_name --> name of the serial to move
            goals --> array of goals to reach
            active --> serial joint to move

        ---
        Types:

            serial_name --> string
            goals --> array 
            active --> list[]
        ---
        Returns:
            trajectory in joint space
        """
        id = self.serial_names.index(serial_name)
        self.serials[id].IK_solve_points(goals,active)
        trj_id = np.asarray(self.serials[id].joint_trajectory)
        trj_shape = trj_id.shape
        np.set_printoptions(threshold=sys.maxsize)
        parallel_trj = np.zeros((trj_id.shape[0],self.n_joints))
        for i,serial in enumerate(self.serials):
            if serial != self.serials[id]:
                q_array = np.full_like(trj_id,serial.q_list)
                new_col_len = len(serial.q_list)
                prv_len = 0
                for k in range(0,i):
                    prv_len += len(self.serials[k].q_list)
                parallel_trj[:,prv_len:prv_len+new_col_len] = q_array
            else:
                new_col_len = len(serial.q_list)
                prv_len = 0
                for k in range(0,i):
                    prv_len += len(self.serials[k].q_list)
                parallel_trj[:,prv_len:prv_len+new_col_len] = trj_id
        return np.asarray(parallel_trj)

    def get_trj_static(self,N):
        trj = np.zeros((N,self.n_joints))
        for i,serial in enumerate(self.serials):
            supp = np.zeros((N,len(serial.q_list)))
            q_array = np.full_like(supp,serial.q_list)
            new_col_len = len(serial.q_list)
            prv_len = 0
            for k in range(0,i):
                prv_len += len(self.serials[k].q_list)
            trj[:,prv_len:prv_len+new_col_len] = q_array
        return np.asarray(trj)



    def IK_solve_floating_base_points(self,goals,active = "all"):
        """ 
        Description:

            function to move floating base of the parallel robot.
        ---
        Keyword arguments:

            goals --> array of goals to reach
            active --> serial joint to move

        ---
        Types:

            goals --> array 
            active --> list[]
        ---
        Returns:
            trajectory in joint space of all serials
        """
        parallel_trj = np.zeros((len(goals),self.n_joints))
        for i,serial in enumerate(self.serials):
            if active == "all":
                act_array = np.ones(len(serial.joint_names_actv))
            else:
                act_array = np.zeros(len(serial.joint_names_actv))
                for act in active[i]:
                    if act in serial.joint_names_actv:
                        num = serial.joint_names_actv.index(act)
                        act_array[num] = 1 
            serial.joint_trajectory = []
            serial.space_trajectory = []
            space_trajectory = []
            for goal in goals :
                serial_endpoint = serial.global_T_matrixs[-1][:3,3] - np.asarray(goal[0:3])
                serial_endax = euler_angles(serial.global_T_matrixs[-1]) - np.asarray(goal[3:6])
                end = np.concatenate((serial_endpoint,serial_endax)) 
                space_trajectory.append([end[0], end[1],   end[2] ,end[3],end[4],end[5]])
            if active == "all":
                self.move_serial(self.serial_names[i],space_trajectory,active = "all")
            else :
                self.move_serial(self.serial_names[i],space_trajectory,active = active[i])                

        start_index = 0

        for serial in self.serials:
            delta_index = len(serial.q_list)
            end_index = start_index+delta_index
            parallel_trj[:,start_index:end_index] += serial.joint_trajectory
            start_index += delta_index 

        parallel_trj = np.asarray(parallel_trj).reshape(len(goals),self.n_joints)
        return parallel_trj





