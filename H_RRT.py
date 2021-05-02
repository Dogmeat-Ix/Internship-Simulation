#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import vizutils
import random
import math

import H_Utils as ut


# In[ ]:


#constants
XDIM = 5
YDIM = 5
WINSIZE  = [XDIM, YDIM]
EPSILON  = 0.3
NUMNODES = 5000
RADIUS   = 0.9


# ### Utility Functions

# In[ ]:


def dist(p1,p2):
    return math.sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))


def step_from_to(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = math.atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON*math.cos(theta), p1[1] + EPSILON*math.sin(theta)


def chooseParent(nn,newnode,nodes):
    for p in nodes:
        if dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and p.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y]):
            nn = p
    newnode.cost=nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y])
    newnode.parent=nn
    return newnode,nn


# In[ ]:


def reWire(nodes,newnode):
    for i in range(len(nodes)):
        p = nodes[i]
        if p!=newnode.parent and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and newnode.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < p.cost:
            p.parent = newnode
            p.cost=newnode.cost+dist([p.x,p.y],[newnode.x,newnode.y]) 
            nodes[i]=p  
              
    return nodes


# In[ ]:


def drawSolutionPath(start,goalXY,nodes,viz):
    path = []
    pink = [.9, .01, .95,1]
    nn = nodes[0]
    for p in nodes:
        if dist([p.x,p.y],[goalXY[0],goalXY[1]]) < dist([nn.x,nn.y],[goalXY[0],goalXY[1]]):
            nn = p
    while nn!=start:
        vizutils.addViewerSphere(viz, nn.name, .04, pink)
        path.append(nn)
        nn=nn.parent
        
    return path    


# In[ ]:
    
    
class Node:
    x = 0
    y = 0
    cost=0  
    parent=None
    name = None
    angle = 0
    q = [0,0,0,.2, 1, 0, 0, 0] #quaternion ;)
    def __init__(self,xcoord, ycoord,name):
        self.x = xcoord
        self.y = ycoord
        self.q = [xcoord,ycoord,.2, 1, 0, 0, 0]
        self.name = 'world/nodes/n_' + str(name)
        
    def position(self):
        return self.q

    def setParent(self,node,angle):
        self.parent = node
        self.angle = angle
        
        

# In[ ]:


class Objective:
    def __init__(self, name, q,viz):
        self.viz    = viz
        self.name   = str(name) if name is not None else 'world/obj/aux'
        self.q      = q
        self.v      = 0
        
        vizutils.addViewerSphere(self.viz, self.name, .04, [.2, .1, 1, .5])
        vizutils.applyViewerConfiguration(self.viz, self.name, q)    
    
    def position(self):
        return self.q
    
    def move(self, q):
        self.q = q
        vizutils.addViewerSphere(self.viz, self.name, .04, [.2, 1, .1, .5])
        vizutils.applyViewerConfiguration(self.viz, self.name, self.q)           
    
    def remove(self):
        self.viz.viewer[self.name].delete() #Deletes all figures in /dir


# In[ ]:


#validation:

def checkVictory(node,goalXY):
    vic = False
    if dist([node.x,node.y],[goalXY[0],goalXY[1]]) < EPSILON :
        vic = True
    return vic


def checkInObstacle(coords,obs,bw):
    inside = False
    
    #X
    inX = False
    if coords[0] >= obs.xyzquat[0]-obs.sizex/2-bw and coords[0] <= obs.xyzquat[0]+obs.sizex/2+bw:
        inX = True    
        
    #Y
    inY = False
    if coords[1] >= obs.xyzquat[1]-obs.sizey/2-bw and coords[1] <= obs.xyzquat[1]+obs.sizey/2+bw:
        inY = True
        
    if inX and inY:
        inside = True
        
    return inside   
    

def checkCobs(coords,cObs,buffWidth):
    for obs in cObs:
        if checkInObstacle(coords,obs,buffWidth):
            #print("inside")
            return [True, obs]
    return [False, None]


# In[ ]:


def RRT(startXY, goalXY,buffWidth,cObs,robotReach,viz):
 
    nodes = []
    objs  = []  
    
    nodes.append(Node(startXY[0],startXY[1],'start')) 
    start=nodes[0]
    

    
    for i in range(NUMNODES):
        rand = Node(random.random()*XDIM, random.random()*YDIM,'rand')
        nn = nodes[0]
        for p in nodes:
            if dist([p.x,p.y],[rand.x,rand.y]) < dist([nn.x,nn.y],[rand.x,rand.y]):
                nn = p
        interpolatedNode= step_from_to([nn.x,nn.y],[rand.x,rand.y])

        ## check if is in freeSpace
        angle = math.atan2(rand.y-nn.y,rand.x-nn.x)
        robotReach.isValidPosition(interpolatedNode,angle,cObs)
        inObstacle = checkCobs(interpolatedNode,cObs,buffWidth)

        if not inObstacle[0]: 
            
            newnode = Node(interpolatedNode[0],interpolatedNode[1],i)
            [newnode,nn]=chooseParent(nn,newnode,nodes);
       
            nodes.append(newnode)
            obj = Objective(newnode.name,newnode.position(),viz)
            objs.append(obj)
        
        
            nodes=reWire(nodes,newnode)
            
            if checkVictory(newnode,goalXY):
                print("FOUND IT!")
                break
        
    path = []            
    path = drawSolutionPath(start,goalXY,nodes,viz)
    print(str(len(nodes)) + 'nodes')
    return objs,path


# In[ ]:


## now some CLUSTERING!

def clusterPath(path,OPT_RADIUS,viz):
    optPath = []

    p = path[0]
    viz.viewer['world/nodes'].delete() #Deletes all figures in /dir
    while  p.name != 'world/nodes/n_start':
        _x = p.x
        _y = p.y
        while (dist([_x,_y],[p.parent.x,p.parent.y]) < OPT_RADIUS) :
            p = p.parent
            _x = (_x+p.x)/2
            _y = (_y+p.y)/2
        
        optPath.append(Objective(p.name+'_opt',[_x, _y, .2, 1, 0, 0, 0],viz))
        p = p.parent 
        print(p.name)

    return optPath


# In[ ]:

def correctPath(path,cObs,bw):

    for i in range(len(path)):
        print(i)
        nodeXY = [path[i].q[0],path[i].q[1]]
        inObstacle = checkCobs(nodeXY,cObs,bw)
        
        if inObstacle[0]:
            moveNodeToFreeSpace(path[i],inObstacle[1],bw)
            print("oh no")
    return path




# In[ ]:





# In[2]:


#deprecated: xD


# In[ ]:


def moveNodeToFreeSpace(node,obs,bw):
    qNode = node.q
   
    border = obs.xyzquat[0] + obs.sizex/2
    if  qNode[0]  > border and qNode[0]  < border + bw:
        print('+x')
        qNode[0] = obs.xyzquat[0] + obs.sizex/2 + bw   
    
    elif qNode[0]  < border and qNode[0]  > border + bw :
        print('-x')
        qNode[0] = obs.xyzquat[0] - obs.sizex/2 - bw   

    border = obs.xyzquat[1] + obs.sizey/2
    if qNode[1]  > border and qNode[1]  < border + bw :
        print('+y')
        qNode[1] = obs.xyzquat[1] + obs.sizey/2 + bw  
    
    elif qNode[1]  < border and qNode[1]  > border + bw :
        print('-y')
        qNode[1] = obs.xyzquat[1] - obs.sizey/2 - bw

    
    node.move(qNode)
    
## detect witch obstacle is in the way, 
## deduce the corner and create an intermediate waypoint

def moveNodeToCorner(node,obs,bw):
    qNode = node.q
   
    if qNode[0]  < obs.xyzquat[0] + obs.sizex/2 + bw :
        print('+x')
        qNode[0] = obs.xyzquat[0] + obs.sizex/2 + bw   
    
    elif qNode[0]  > obs.xyzquat[0] - obs.sizex/2 - bw :
        print('-x')
        qNode[0] = obs.xyzquat[0] - obs.sizex/2 - bw   

    if qNode[1]  < obs.xyzquat[1] + obs.sizey/2 + bw :
        print('+y')
        qNode[1] = obs.xyzquat[1] + obs.sizey/2 + bw  
    
    elif qNode[1]  > obs.xyzquat[1] - obs.sizey/2 - bw :
        print('-y')
        qNode[1] = obs.xyzquat[1] - obs.sizey/2 - bw

    
    node.move(qNode)
    

