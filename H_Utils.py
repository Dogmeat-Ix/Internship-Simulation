#!/usr/bin/env python
# coding: utf-8

# In[1]:

import pinocchio as pin

import numpy as np
from numpy.linalg import inv, pinv, eig, norm, svd, det


# In[2]:


def angleSE3(x, y, z, axis, angle):
    """
    Minimal helper function: return the SE3 configuration of a stepstone, with some
    ad-hoc configuration.
    """
    # x,y are integers
    # y is the altitude in centimer
    # axis is 3D, rotation axis in the plane
    # angle is the angle of inclination, in radian

    axis = np.array(axis, np.float64)
    axis /= norm(axis)
    return pin.SE3(pin.AngleAxis(angle,axis).matrix(), np.array([x, y, z]))
    


# In[ ]:




