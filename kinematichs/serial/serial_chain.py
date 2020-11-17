#!/usr/bin/env python3
import numpy as np 
from kinematichs.support.matrixs import mdot 
from kinematichs.support.base_func import jacobian_g
from kinematichs.support.base_func import jacobian_a 
from kinematichs.serial.parametrization import Joint
from kinematichs.support.base_func import euler_angles
import time


class Kinematichs_chain():
    def __init__(self,par,name):
        """CHAIN CLASS 
        ---
        Description:

            create chain class for robotichs.
        ---
        Keyword arguments:

            par --> paramtrization of the robot constructed listing joint in serial order
            name --> name of the robot serial

        ---
        Types:

            par --> list
            name --> string
        ---
        Returns:
            chain
        """

        self.par0 = par 
        self.par = par 
        self.active_par = self.get_active_par(par)
        self.joint_types = self.get_joint_types()
        self.T_matrixs = None
        self.global_T_matrixs = None
        self.J_g = None
        self.J_a = None 
        self.q_list = None
        self.name = self.get_name(name)
        self.joint_names = self.get_joint_names(par)
        self.joint_names_actv = self.get_joint_names_actv(par)

        self.get_T_matrixs()
        self.get_T_globals()
        self.get_q()
        self.get_J_a()
        self.get_J_g()
        
        self.stepsize = 0.9
        self.K = 0.9
        self.itermax = 100
        self.min_error= 10^-3

        self.space_trajectory = None
        self.joint_trajectory = None

        self.set_joint_var(np.zeros(len(self.active_par)))

    def set_IK_parameters(self,K,itermax,min_error):
        self.K = K
        self.itermax = itermax
        self.min_error = min_error

    def get_name(self,name):       
        if isinstance(name, str):
            return name
        else : 
            print(f"name must be string , ho {type(name)}")
            return None

    def get_joint_names(self,par):
        names = []
        for j in par :
            names.append(j.name)
        return names

    def get_joint_names_actv(self,par):
        names = []
        for j in par :
            if j.is_active == True : 
                names.append(j.name)
        return names

    def get_joint_types(self):
        par = self.par
        types_list = []
        for p in par : 
            joint_type = p.type
            types_list.append(joint_type)
        return types_list 

    def get_T_matrixs(self):
        par = self.par
        T_list = np.empty((len(par),par[0].matrix.shape[0],par[0].matrix.shape[1]))
        for i,p in enumerate(par):
            mat = p.matrix
            T_list[i] = mat
        self.T_matrixs = np.asarray(T_list)

    def get_active_par(self,par):
        par_nonfixed = []
        for p in par :
            if p.type == "f":
                pass
            else:
                par_nonfixed.append(p)
        return par_nonfixed

    def get_T_globals(self):
        matrixs = self.T_matrixs
        m = np.eye(4)
        global_T = []
        for i in range(0,len(matrixs)):
            m = mdot(m,matrixs[i])
            global_T.append(m)

        self.global_T_matrixs = global_T

    def get_J_g(self):
        types = self.joint_types
        matrixs = self.global_T_matrixs 
        J_g = jacobian_g(matrixs,types)
        self.J_g = J_g

    def get_J_a(self):
        types = self.joint_types
        matrixs = self.global_T_matrixs
        J_a = jacobian_a(matrixs,types)
        self.J_a = J_a

    def set_joint_var(self,vars):
        q_list = []
        for i,p in enumerate(self.active_par) :
            p.set_q(vars[i])
            q_list.append(vars[i])
        self.q_list = q_list

    def get_q(self):
        par = self.par 
        q_list = []
        for p in par:
            if p.type != "f":
                q = p.q
                q_list.append(q)
        self.q = q_list


    def _IK_solve_point(self,goal,act_array): 
        self.active_par = self.get_active_par(self.par)
        self.joint_types = self.get_joint_types()
        self.get_q()
        stepsize = self.stepsize
        types = self.joint_types
        K = self.K
        itermax = self.itermax
        minerror = self.min_error
        validcoords = []
        for i,g in enumerate(goal):
            if np.isnan(g) == False:
                validcoords.append(i)
        goal = np.asarray(goal)
        i = 0
        a_p_array = self.global_T_matrixs[-1][:3,3]
        a_z_array = self.global_T_matrixs[-1][:3,2]
        a_array = np.concatenate((a_p_array,a_z_array))
        error = np.subtract(goal,a_array)[validcoords]
        q_new = self.q_list

        while i<=itermax and np.any(abs(error)>minerror)==True:
            matrixs = self.global_T_matrixs
            q = np.asarray(self.q_list)
            a_p_array = matrixs[-1][:3,3]
            a_z_array = euler_angles(matrixs[-1])
            a_array = np.concatenate((a_p_array,a_z_array))
            error = (goal[validcoords]-a_array[validcoords])
            J_a = jacobian_a(matrixs,types)[validcoords,:]
            Jqud = np.dot(J_a.T, J_a)
            M_LB = np.dot(  np.linalg.pinv( Jqud + K * np.eye(Jqud.shape[0]) ) , J_a.T )
            dq = stepsize * np.dot( M_LB , error )
            q_new = q + dq * act_array
            self.set_joint_var( q_new )
            self.get_T_matrixs()
            self.get_T_globals()
            i += 1
        self.joint_trajectory.append(self.q_list)
        self.space_trajectory.append(a_array)

    def IK_solve_points(self,goals,active = "all"):
        if active == "all":
            act_array = np.ones(len(self.joint_names_actv))
        else:
            act_array = np.zeros(len(self.joint_names_actv))
            for act in active:
                if act in self.joint_names_actv:
                    num = self.joint_names_actv.index(act)
                    act_array[num] = 1 
        self.joint_trajectory = []
        self.space_trajectory = []
        for goal in goals :
            self._IK_solve_point(goal,act_array)
        return np.asarray(self.joint_trajectory) , np.asarray(self.space_trajectory)

##-----------------------------------------------------------------------###
#in development
    def stack_of_tasks(self,tasks_list):
        self.joint_trajectory = []
        self.space_trajectory = []

        for tasks in tasks_list:
            self._stack_of_tasks(tasks)

        return self.joint_trajectory , self.space_trajectory

    def _stack_of_tasks(self,tasks):
        self.active_par = self.get_active_par(self.par)
        self.joint_types = self.get_joint_types()
        self.get_q()

        types = self.joint_types
        #K = self.K
        itermax = self.itermax
        #minerror = self.min_error
     
        tasks_validcoords = []
        for task in tasks:
            task_validcoords = []
            for i,c in enumerate(task.goal):
                if c is not np.nan:
                    task_validcoords.append(i)
            tasks_validcoords.append(task_validcoords)

        i = 0

        space_trajectory = []
        joint_trajectory = []

        while i<=itermax :
            matrixs = self.global_T_matrixs
            J_a = jacobian_a(matrixs,types)[tasks_validcoords , :]
            
            JpJ = np.dot ( np.linalg.pinv(J_a) , J_a)
            Pr = JpJ 
            
            q = np.asarray(self.q_list)
            for task in tasks:
                
                ee_name = task.ee_name
                k = self.joint_names_actv.index(ee_name)
                goal = task.goal

                matrixs = self.global_T_matrixs[0:k]
                a_p_array = self.global_T_matrixs[k][:3,3]
                a_z_array = self.global_T_matrixs[k][:3,2]

                a_array = np.concatenate((a_p_array,a_z_array))
                error = np.subtract(goal,a_array)[tasks_validcoords]
                J_a = jacobian_a(matrixs,types)[tasks_validcoords]

                Pr += - mdot( np.asarray(np.linalg.pinv( np.dot(J_a , Pr) ) , J_a , Pr ) ) 
                dq = mdot(Pr, np.asarray( np.linalg.pinv(J_a),error ) ) - mdot( Pr, np.asarray( np.linalg.pinv( np.dot(J_a,Pr,self.q_list) ) ) )
                
            q_new = q + dq
            self.set_joint_var( q_new )
            self.get_T_matrixs()
            self.get_T_globals()
            space_trajectory.append(a_array)
            joint_trajectory.append(q_new)
            i += 1
            
        self.joint_trajectory.append(joint_trajectory)
        self.space_trajectory.append(space_trajectory)
