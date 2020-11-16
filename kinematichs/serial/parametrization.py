import numpy as np 
from kinematichs.support.matrixs import mdot 
from kinematichs.support.base_func import jacobian_g
from kinematichs.support.base_func import jacobian_a 

class Joint():
    def __init__(self, d ,theta ,r ,alpha ,activparam,name):
        """JOINT CLASS 
        ---
        Description:

            create joint class for robotichs.
        ---
        Keyword arguments:

            d --> d traslation of z axis DH to next joint.
            theta --> theta rotation on z axis of DH to next joint.
            r --> r traslation on x axis of DH to next joint.
            alpha --> alpha rotation ton x axis of HD to next joint.
            activeparam --> select "d","theta","r","alpha", paramenter of joint movement.
            name --> name of joint.
        ---
        Types:

            d : float64
            theta : float64
            r : float64
            activeparam : string
            name : string
        ---
        Returns:
            joint 
        """
        
        self.d = d 
        self.theta = theta
        self.r = r 
        self.alpha = alpha 
        self.active = self.get_q_active(activparam)
        self.type_base = self.get_type()
        self.type = self.get_type()
        self.q = 0
        self.matrix = self.getmatrix(d ,theta ,r ,alpha )
        self.name = self.get_name(name)
        self.is_active = self.act_state()
    
    def get_name(self,name):
        if isinstance(name, str) :
            return name
        else : 
            print(f"name must be string , got {type(name)}")
            return None

    def get_q_active(self,activparam):
        actlist = set(["d","theta","r","alpha","none"])
        if (activparam in actlist)==False :
            print(f"active parameter must be in list : {actlist} , got {activparam}")
        else: 
            return activparam
        
    def get_type(self):
        active = self.active
        if active == "none":
            return "f"
        elif active == "d" or active == "r":
            return "t"
        elif active == "theta" or "alpha" : 
            return "r"

    def getmatrix(self,d ,theta ,r ,alpha):
        ctheta = np.cos( theta )
        calpha = np.cos( alpha )
        stheta = np.sin( theta )
        salpha = np.sin( alpha )

        T = np.asarray  ( [ [ctheta ,   -stheta * calpha  , stheta * salpha     , r * ctheta  ] ,
                            [stheta ,   ctheta * calpha   , - ctheta * salpha    , r * stheta ] ,
                            [0      ,   salpha          , calpha            , d               ] ,
                            [0      ,   0               , 0                 ,1              ] ]
                        )
        return T 

    def set_q(self,q):
        theta = self.theta 
        alpha = self.alpha 
        r = self.r
        d = self.d 
        active = self.active
        if active == "theta" :
            theta = theta + q
        elif active == "d" :
            d = d + q
        elif active == "r" :
            r = r + q
        elif active == "alpha" :
            alpha = alpha + q
        elif active == "none" :
            pass 

        T = self.getmatrix(d,theta,r,alpha)
        self.matrix = T 

    def reset(self):
        d = self.d 
        theta = self.theta 
        r = self.r 
        alpha = self.alpha
        self.matrix  = self.getmatrix(d,theta,r,alpha)

    def turn_off(self):
        self.type = "f"

    def turn_on(self):
        self.type = self.type_base

    def act_state(self):
        if self.type != "f":
            return True
        else:
            return False 
