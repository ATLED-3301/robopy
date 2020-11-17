#!/usr/bin/env python3
import numpy as np
from numpy.linalg import inv 
from scipy.spatial import distance 
from math import sin
from math import cos 
from numba import jit

@jit(forceobj=True)
def vector_angle(u,v):
    num = np.arccos( np.dot(u ,v) )
    den = np.power(np.dot(v,v),1/2) * np.power(np.dot(u,u),1/2 )
    angle = num / den 
    if np.isnan(angle).any(0) == True:
        angle = 0
    return angle

@jit(forceobj=True)
def Rz(theta_z):
    a = theta_z
    T = np.array([[cos(a),-sin(a), 0,0 ],[sin(a),cos(a),0,0],[0,0,1,0],[0,0,0,1]],dtype = np.float32)
    return T

@jit(forceobj=True)
def Ry(theta_y):
    a = theta_y
    T =  np.array([[cos(a),0, sin(a),0 ],[0,1,0,0],[-sin(a),0,cos(a),0],[0,0,0,1]],dtype = np.float32)
    return T

@jit(forceobj=True)
def Rx(theta_x):
    a = theta_x
    T =  np.array([[1,0, 0,0 ],[0,cos(a),-sin(a),0],[0,sin(a),cos(a),0],[0,0,0,1]],dtype = np.float32)
    return T

@jit(forceobj=True)
def T_tr(dx,dy,dz):
    T =  np.array([[1 ,0 ,0 ,dx],[0,1,0,dy],[0,0,1,dz],[0,0,0,1]],dtype = np.float32)
    return T

@jit(forceobj=True)
def Tx(theta_x,dx,dy,dz):
    a = theta_x
    T =  np.array([[1,0, 0,dx ],[0,cos(a),-sin(a),dy],[0,sin(a),cos(a),dz],[0,0,0,1]],dtype = np.float32)
    return T

@jit(forceobj=True)
def Ty(theta_y,dx,dy,dz):
    a = theta_y
    T =  np.array([[cos(a),0, sin(a),dx ],[0,1,0,dy],[-sin(a),0,cos(a),dz],[0,0,0,1]],dtype = np.float32)
    return T

@jit(forceobj=True)
def Tz(theta_z,dx,dy,dz):
    a = theta_z
    T = np.array(   [   [cos(a),     -sin(a),    0,     dx],
                        [sin(a),      cos(a),    0,     dy],
                        [0     ,           0,    1,     dz],
                        [0     ,           0,    0,     1 ]
                        ] ,
                        dtype = np.float32 )
    return T

@jit
def solve_rr(x,y,L12,L23,elbow):
    num = np.power( L12, 2 ) + np.power( L23 , 2 ) - np.power( x , 2 ) - np.power( y , 2 )
    den = 2 * L12 * L23
    arg = num / den
    if elbow == "down":
        q2 = np.pi - np.arccos(arg) 
    elif elbow == "up" :
        q2 = - np.arccos(- arg) 
    else:
        print("wrong format,insert 1:up,or 2:down")
    q1 = np.arctan2(y,x) - np.arctan2((L23 * np.sin(q2)) , (L12 + L23 * np.cos(q2)))

    return q1,q2

@jit(forceobj=True)
def euler_angles(R):
    phi = np.arctan2(R[1,2] , R[0,2])
    theta = np.arctan2( np.power(np.power(R[0,2], 2) + np.power(R[1,2] ,2) ,1/2), R[2,2] )
    psi = np.arctan2( - R[2,1], R[2,1] )
    angles = np.asarray( [phi, theta, psi] )
    return angles

@jit(forceobj=True)
def mdot(*args):
    T = np.eye( args[0].shape[0] )
    for arg in args:
        T = np.dot( T , arg )
    return T

def T_DH(d_n,theta_n,r_n,alpha_n):
    ctheta = np.cos( theta_n )
    calpha = np.cos( alpha_n )
    stheta = np.sin( theta_n )
    salpha = np.sin( alpha_n )

    T = np.asarray  ( [ [ctheta ,   -stheta*calpha  , stheta*salpha     , r_n * ctheta  ] ,
                        [stheta ,   ctheta*calpha   , -ctheta*salpha    , r_n*stheta    ] ,
                        [0      ,   salpha          , calpha            , d_n           ] ,
                        [0      ,   0               , 0                 ,1              ] ]
                    )

    return T
