import numpy as np
import matplotlib.pylab as plt
from mpl_toolkits.mplot3d import Axes3D
import bezier
from geomdl import BSpline

# Define the Bezier curve
#nodes = np.array([
#        [0.0, 2, 1],
#        [0.0, 0, 1],
#        [0,2,1] 
#        ])

#curve = bezier.Curve.from_nodes(nodes)

#print(curve)

#t_fine = np.linspace(0, 1, 60) # Curvilinear coordinate
#points_fine = curve.evaluate_multi(t_fine)
#points_fine.shape  # (2, 60)

# Interpolation on regular x coordinates
#x_xregular = np.linspace(0, 10, 70)

#t_xregular = np.interp(x_xregular, points_fine[0], t_fine)
#points_xregular = curve.evaluate_multi(t_xregular)

# Plot

#ax.scatter(*[(4)/3,1/3,4/3])
#plt.plot(*nodes, '-o', label='definition nodes')
#plt.plot(*points_fine, label='Bezier curve')
#plt.plot(*points_xregular, 'ok', label='regularly spaced along x')
#plt.xlabel('x'); plt.ylabel('y'); plt.legend()





def bezier_fromnodes(begin,end,control_points,N = 10):
	c_p = np.asarray(control_points,dtype = np.float64)
	begin = np.asarray(begin,dtype = np.float64)
	end = np.asarray(end,dtype = np.float64)
	#print(control_points.dtype)
	#print(begin,end,control_points)
	bez_par = np.row_stack([begin,c_p,end]).T
	#print(bez_par)
	curve = bezier.Curve.from_nodes(bez_par)
	t = np.linspace(0, 1, N) # Curvilinear coordinate
	points = np.asarray(curve.evaluate_multi(t))
	return points




def generate_trj_bezier(begin,end,control_points,N = 100):
	"""genera una traiettoria mediante curve di beier dato punto di inizio, fine e nodi di
	controllo per le posizioni x y z , per gli ngoli phi theta e psi genera una traiettoria
	lineare"""
	begin_points = begin[0:3]
	begin_angles = begin[3:6]
	end_points = end[0:3]
	end_angles = end[3:6]
	control_points = np.asarray(control_points)
	points = bezier_fromnodes(begin_points,end_points,control_points,N).T
	angles = np.linspace(begin_angles, end_angles, num=N, endpoint=False)
	coords = np.hstack((points,angles))
	return coords


def generate_bspline(control_nodes,N = 100,degree = 3):
	"""genera una b-spline dato punto di inizio, fine e nodi di
	controllo per le posizioni x y z , per gli angoli phi theta e psi genera una traiettoria
	lineare"""
	control_nodes = np.asarray(control_nodes)
	control_points = []
	for con in control_nodes:
		c = con[0:3]
		control_points.append(c)
	np.asarray(control_points)
	control_angles = []
	for con in control_nodes:
		c = con[3:6]
		control_angles.append(c)
	np.asarray(control_angles)

	angles = np.linspace(control_angles[0], control_angles[-1], num=N, endpoint=False)
	points = bspline(control_points, n=N, degree=degree, periodic=False)
	coords = np.hstack((points,angles))
	return coords


import scipy.interpolate as si
def bspline(cv, n=100, degree=3, periodic=False):
    """ Calculate n samples on a bspline

        cv :      Array ov control vertices
        n  :      Number of samples to return
        degree:   Curve degree
        periodic: True - Curve is closed
                  False - Curve is open
    """

    # If periodic, extend the point array by count+degree+1
    cv = np.asarray(cv)
    count = len(cv)

    if periodic:
        factor, fraction = divmod(count+degree+1, count)
        cv = np.concatenate((cv,) * factor + (cv[:fraction],))
        count = len(cv)
        degree = np.clip(degree,1,degree)

    # If opened, prevent degree from exceeding count-1
    else:
        degree = np.clip(degree,1,count-1)


    # Calculate knot vector
    kv = None
    if periodic:
        kv = np.arange(0-degree,count+degree+degree-1)
    else:
        kv = np.clip(np.arange(count+degree+1)-degree,0,count-degree)

    # Calculate query range
    u = np.linspace(periodic,(count-degree),n)


    # Calculate result
    return np.array(si.splev(u, (kv,cv.T,degree))).T