import numpy as np

class Trajectory():
    def __init___(self,points,t0,tf):
        self.points = self.get_points(points) 
        self.times = self.get_times(points, t0 , tf)
        self.dt = self.get_dt(points, t0 , tf)
        self.values = self.getvalues()
    
    def get_points(self,points):
        p_shape = points.shape[1]
        if p_shape != 3:
            print(f"points must be an array of shape 3xN , got {p_shape}xN")
            return None
        else : 
            return points

    def get_dt(self, points, t0 , tf):
        dt = (tf-t0) / points.shape[1]
        return dt

    def get_times(self, points, t0 , tf):
        times = np.arange(t0, tf, points.shape[1], dtype=None)
        return times 

    def getvalues(self):
        values = np.asarray( [ self.points , self.times ] )
        return values 

    

