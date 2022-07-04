# Q1: (3.92, 0.69, 0.35)
# Q3: ICP is sensitive to outliers caused by moving objects.
# Q4: When the vehicle is moving quickly, it is important to account for the
# time differences between individual LIDAR pulses.# 
# It is important to identify shiny and highly reflective objects in the 
# environment, as LIDAR measurements of those objects may be invalid.    
# Q5: Pv = Cvl*Pl


# Q2: Programming quiz

from numpy import *

def sph_to_cart(epsilon, alpha, r):
  """
  Transform sensor readings to Cartesian coordinates in the sensor
  frame. The values of epsilon and alpha are given in radians, while 
  r is in metres. Epsilon is the elevation angle and alpha is the
  azimuth angle (i.e., in the x,y plane).
  """
  p = zeros(3)  # Position vector 
  
  # Your code here
  p[0] = r*cos(epsilon)*cos(alpha)
  p[1] = r*cos(epsilon)*sin(alpha)
  p[2] = r*sin(epsilon)
  
  return p
  
def estimate_params(P):
  """
  Estimate parameters from sensor readings in the Cartesian frame.
  Each row in the P matrix contains a single 3D point measurement;
  the matrix P has size n x 3 (for n points). The format is:
  
  P = [[x1, y1, z1],
       [x2, y2, z2], ...]
       
  where all coordinate values are in metres. Three parameters are
  required to fit the plane, a, b, and c, according to the equation
  
  z = a + bx + cy
  
  The function should retrn the parameters as a NumPy array of size
  three, in the order [a, b, c].
  """
  param_est = zeros(3)
  
  # Your code here
  n = shape(P)[0]
  A = array([[1, P[i,0], P[i,1]] for i in range(n)])
  b = array([P[i,2] for i in range(n)])
  param_est = linalg.pinv(A)@b

  return param_est

# ====================
# MAIN 
# ====================

# print(sph_to_cart(1,1,1))
estimate_params(array([[1, 2, 1],[4, 5, 1], [3, 1, 1]]))