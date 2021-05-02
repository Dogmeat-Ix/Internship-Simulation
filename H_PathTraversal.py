#!/usr/bin/env python
# coding: utf-8

# ### Cost Functions
# 

# In[14]:


import pinocchio as pin
from pinocchio.utils import *

import example_robot_data as robex
import numpy as np
from numpy.linalg import inv, pinv, eig, norm, svd, det
from scipy.optimize import fmin_bfgs
import time

import vizutils
import math

import H_Utils as ut

# In[13]:


class Cost3d:
    def __init__(self, rmodel, rdata, frameIndex=None, ptarget=None, viz=None):
        self.rmodel = rmodel
        self.rdata = rdata
        self.ptarget = ptarget if ptarget is not None else  np.array([0.5, 0.1, 0.27])
        self.frameIndex = frameIndex if frameIndex is not None else robot.model.nframes - 1
        self.viz = viz

    def residual(self, q):
        pin.framesForwardKinematics(self.rmodel, self.rdata, q)
        M = self.rdata.oMf[self.frameIndex]
        return M.translation - self.ptarget

    def calc(self, q):
        return sum(self.residual(q) ** 2)

    # --- Callback
    def callback(self, q):
        if self.viz is None:
            return
       # pin.framesForwardKinematics(self.rmodel, self.rdata, q)
       # M = self.rdata.oMf[self.frameIndex]
        
        self.viz.display(q)
        time.sleep(0.05) #exagerating the time to make it visible


# In[6]:


class Cost6d:

    def __init__(self, rmodel, rdata,robotReach, frameIndex=None, Mtarget=None, viz=None):
        self.rmodel = rmodel
        self.rdata = rdata
        self.Mtarget = Mtarget if Mtarget is not None else pin.SE3(pin.utils.rotate('z', 90),np.array([0,0,.2]))  # x, y, z
        self.frameIndex = frameIndex if frameIndex is not None else robot.model.nframes-1
        self.robotReach = robotReach
        self.viz = viz

    def residual(self, q):
        '''Compute score from a configuration'''
        pin.forwardKinematics(self.rmodel, self.rdata, q)
        M = pin.updateFramePlacement(self.rmodel, self.rdata, self.frameIndex)
        self.deltaM = self.Mtarget.inverse() * M
        return pin.log(self.deltaM).vector

    def calc(self, q):
        return sum(self.residual(q) ** 2)

    def callback(self, q):
        if self.viz is None:
            return
        #pin.framesForwardKinematics(self.rmodel, self.rdata, q)
        #M = self.rdata.oMf[self.frameIndex]
        self.robotReach.move(q)

        self.viz.display(q)
        time.sleep(0.0005) #exagerating the time to make it visible
        
        


# In[7]:


class CostPosture:
    def __init__(self, rmodel, rdata, robot,qref=None, viz=None):
        # viz, rmodel and rdata are taken to respect the API but are not useful.
        self.rmodel = rmodel
        self.qref = qref if qref is not None else pin.randomConfiguration(rmodel)

        self.removeFreeFlyer = robot.model.joints[1].nq == 7  # Don't penalize the free flyer if any.
        self.qref = pin.normalize(self.rmodel, self.qref)

    def residual(self, q):
        if self.removeFreeFlyer:
            return (q - self.qref)[7:]
        else:
            return q - self.qref

    def calc(self, q):
        return sum(self.residual(q) ** 2)


# In[8]:


class SumOfCostFreeFlyer:
    def __init__(self, rmodel, rdata, costs, weights):
        self.rmodel = rmodel
        self.costs = costs
        self.weights = np.array(weights)

    def calc(self, q):
        q = pin.normalize(self.rmodel, q)
        # Alternatively, add this extra term to the sum
        extra = sum((q - pin.normalize(self.rmodel, q)) ** 2) * 100
        return sum(self.weights * [cost.calc(q) for cost in self.costs])   + extra

    def callback(self, q):
        q = pin.normalize(self.rmodel, q)
        for c in self.costs:
            if hasattr(c, 'callback'):
                c.callback(q)
       
        time.sleep(0.0005)


# ## Inverse Kinematics and Movement

# In[9]:


class Segment:
    def __init__(self, name, startq, endq, viz, size=None):
        self.name   = 'world/seg/segStart_' + str(name)
        self.startq = startq
        self.endq   = endq
        self.size   = size if size is not None else .5
        self.viz    = viz
        
        vizutils.addViewerSphere(self.viz, self.name, .02, [.2, 1, .1, 1])
        vizutils.applyViewerConfiguration(self.viz, self.name, startq)    
        
        
    def getOrientation(self):
        ## see how the orientation relates to the transform 
        ##myradians = math.atan2(targetY-gunY, targetX-gunX)
        theta = math.atan2(self.endq[1]-self.startq[1],self.endq[0]-self.startq[0])
        return theta
        
    def remove(self):
        vizutils.addViewerSphere(self.viz, self.name, .0002, [.2, 1, .1, 0])


# In[10]:


def genSegments(start, end,viz, segSize=None):
    #segSize = .3 ## Max step size
    segSize   = segSize if segSize is not None else .3


    distance = math.hypot(end[0] - start[0], end[1] - start[1]) 
    print ("OG distance: " + str(distance))

    segAmount = math.ceil(distance / segSize)
    print("segments: " + str(segAmount))

    xstep = (end[0] - start[0])/segAmount
    ystep = (end[1] - start[1])/segAmount

    segments = []
    for i in range(segAmount):
        xiTmp = [start[0]+xstep*i,start[1]+ystep*i,.2,1,0,0,0]
        xfTmp = [xiTmp[0]+xstep,xiTmp[1]+ystep,.2,1,0,0,0]
    
        segments.append(Segment(i,xiTmp,xfTmp,viz))
        
    return segments


# In[11]:


### travel through the segments until final destination


# In[12]:


# In[12]:    

def travelSegments(segments, start,rmodel, rdata,robot,robotReach,viz):
    qguess = start
    for seg in segments:
        deg = seg.getOrientation()
        print('qguess: '+ str(qguess))
        print('orientation: '+ str(deg))
        
        MtargetEnd = ut.angleSE3(seg.endq[0], seg.endq[1], seg.endq[2], [0,0,1], deg)

        
        print(MtargetEnd)
        
        #MtargetEnd = pin.SE3(pin.utils.rotate('z', 0), np.array(seg.endq[:3]))
        
        cost6d = Cost6d(rmodel, rdata, robotReach, frameIndex = 1,Mtarget = MtargetEnd, viz=viz)
       # mycost = SumOfCostFreeFlyer(rmodel, rdata,[cost6d,CostPosture(rmodel, rdata,robot, qref = qguess )], [1, 1e-3])
        
    
        qopt = fmin_bfgs(cost6d.calc, qguess, callback=cost6d.callback)
        qguess = qopt
        viz.display(qopt)
    
    viz.viewer['world/seg'].delete() #Deletes all figures in /dir
    return qopt


# In[ ]:
