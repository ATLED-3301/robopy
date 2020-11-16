import numpy as np
from numpy.linalg import inv 
from scipy.spatial import distance 
from math import sin 
from math import cos 
from numba import jit
import numpy as np
from kinematichs.support.matrixs import mdot,Tx,Tz,Ty



def r_jacobian(ax_previus, previus_point, final_point):

    # cross per prodotto vettore
    # dot per prodotto scalare tra matrici

    O_p = previus_point
    p_f = final_point
    k_p = ax_previus

    Jp_point = - np.cross(k_p,  O_p - p_f)
    Jo_point = k_p

    J_coloumn = np.append(Jp_point,Jo_point)

    return J_coloumn

def t_jacobian(ax_previus, previus_point, final_point):

    # cross per prodotto vettore
    # dot per prodotto scalare tra matrici

    k_p = ax_previus

    Jp_point = k_p
    Jo_point = 0

    J_coloumn = Jp_point.append(Jo_point)

    return J_coloumn


def jacobian_g(serial_robot_matrixs,joint_type_list):
    mat = serial_robot_matrixs
    a = False
    i = 0
    while i<len(joint_type_list) and a != 1 :
        if joint_type_list[i] != "f":
            Or = serial_robot_matrixs[0]
            p_ax = Or[:3,2]
            p_p = Or[:3,3]
            a = True
        else:
            i += 1
    f_p = serial_robot_matrixs[-1][:3,3]
    cl_mat = []
    cl_j_type = []
    for i,m in enumerate(mat):
        #print(i)
        if joint_type_list[i] == "f":
            pass
        else:
            cl_j_type.append(joint_type_list[i])
            cl_mat.append(m)
    Jmat = []
    for i,mat in enumerate(cl_mat):
        if cl_j_type[i] == "r":
            J = r_jacobian(p_ax,p_p,f_p)
            Jmat.append(J)
        elif cl_j_type[i] == "t":
            J = t_jacobian(p_ax,p_p,f_p)
            Jmat.append(J)
        else:
            print(f"joint type error: must be r or t or f,got {cl_j_type[i]}")
        p_ax = mat[:3,2]
        p_p = mat[:3,3]
    
    return np.asarray(Jmat).T

def zyz_trasf(phi,theta,psi):
    T = mdot(Ty(phi,0,0,0),Tz(theta,0,0,0),Ty(psi,0,0,0))
    return T

def jacobian_a(serial_robot_matrixs,joint_type_list):
    mat = serial_robot_matrixs
    a = False
    i = 0
    while i<len(joint_type_list) and a != 1 :
        if joint_type_list[i] != "f":
            Or = serial_robot_matrixs[0]
            p_ax = Or[:3,2]
            p_p = Or[:3,3]
            a = True
        else:
            i += 1
    f_p = serial_robot_matrixs[-1][:3,3]
    cl_mat = []
    cl_j_type = []
    for i,m in enumerate(mat):
        if joint_type_list[i] == "f":
            pass
        else:
            cl_j_type.append(joint_type_list[i])
            cl_mat.append(m)
    Jmat = []
    for i,mat in enumerate(cl_mat):
        if cl_j_type[i] == "r":
            J = r_jacobian(p_ax,p_p,f_p)
            Jmat.append(J)
        elif cl_j_type[i] == "t":
            J = t_jacobian(p_ax,p_p,f_p)
            angles = euler_angles(mat)
            phi = angles[0]
            theta = angles[1]
            psi = angles[2]
            T = zyz_trasf(phi,theta,psi)
            J[3:6] = np.linalg.pinv(T) * J[3:6]
            Jmat.append(J)
        else:
            print(f"joint type error: must be r or t or f,got {cl_j_type[i]}")
        p_ax = mat[:3,2]
        p_p = mat[:3,3]
    
    return np.asarray(Jmat).T


@jit(forceobj=True)
def vector_angle(u,v):
    angle = np.arccos( np.dot(u ,v) / ( np.power(np.dot(v,v),1/2) * np.power(np.dot(u,u),1/2 ) ) )
    if np.isnan(angle).any(0) == True:
        angle = 0
    return angle

@jit
def solve_rr(x,y,L12,L23,elbow):
    arg = ( np.power(L12,2) + np.power(L23,2) - np.power(x,2) - np.power(y,2) ) / (2 * L12 * L23)
    if elbow == "down":
        q2 = np.pi - np.arccos(arg) 
    elif elbow == "up" :
        q2 = -np.arccos(-arg) 
    else:
        print("wrong format,insert 1:up,or 2:down")
    q1 = np.arctan2(y,x) - np.arctan2((L23 * np.sin(q2)) , (L12 + L23 * np.cos(q2)))

    return q1,q2

@jit(forceobj=True)
def euler_angles(R):
    phi = np.arctan2(R[1,2],R[0,2])
    theta = np.arctan2(np.power(np.power(R[0,2],2)+np.power(R[1,2],2),1/2),R[2,2])
    psi = np.arctan2(R[2,1],-R[2,1])
    angles = np.asarray([phi,theta,psi])
    return angles
