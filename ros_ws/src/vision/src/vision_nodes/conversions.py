#Basic libraries
import numpy as np
import math

def quaternion_to_theta(q):

  #Get quaternion
  (x, y, z, w) = (q[0], q[1], q[2], q[3])

  #Calculate angles
  t1 = +2.0 * (w * z + x * y)
  t2 = +1.0 - 2.0 * (y * y + z * z)
  theta = math.atan2(t1, t2)
  
  return theta 
