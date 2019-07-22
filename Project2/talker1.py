#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose,Twist
import time
import math
import rospy
from pqp_server.srv import *
import random
import math
import numpy as np
import scipy.spatial
import matplotlib as mpl
mpl.use('TkAgg')
import matplotlib.pyplot as plt

# parameter
N_SAMPLE = 100  # number of sample_points
N_KNN = 10  # number of edge from one sampled point
#N_KNN = math.e*(1+1/3)*math.log(N_SAMPLE)

#print N_KNN
MAX_EDGE_LEN = 10  # [m] Maximum edge length

show_animation = False

def pqp_client(T, R):
    if len(T) != 3 or len(R) != 9:
        print "Incorrect list size for pqp request"
        return True
    rospy.wait_for_service('pqp_server')
    try:
	
        pqp_server = rospy.ServiceProxy('pqp_server', pqpRequest)
        result = pqp_server(T, R)
	print result        
	return result
    except rospy.ServiceException, e:
        print "Service Call Failed: %s"%e


class Node:
    """
    Node class for dijkstra search
    """

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


class KDTree:
    """
    Nearest neighbor search class with KDTree
    """

    def __init__(self, data):
        # store kd-tree
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, inp, k=1):
        u"""
        Search NN

        inp: input data, single frame or multi frame

        """

        if len(inp.shape) >= 2:  # multi input
            index = []
            dist = []

            for i in inp.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist
        else:
            dist, index = self.tree.query(inp, k=k)
            return index, dist

    def search_in_distance(self, inp, r):
        u"""
        find points with in a distance r
        """

        index = self.tree.query_ball_point(inp, r)
        return index


def PRM_planning(sx, sy, gx, gy, ox, oy, rr):

    obkdtree = KDTree(np.vstack((ox, oy)).T)

    sample_x, sample_y = sample_points(sx, sy, gx, gy, rr, ox, oy, obkdtree)
    if show_animation:
        plt.plot(sample_x, sample_y, ".b")

    road_map = generate_roadmap(sample_x, sample_y, rr, obkdtree)

    rx, ry = dijkstra_planning(
        sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y)

    return rx, ry


def is_collision(sx, sy, gx, gy, rr, okdtree):
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    print " new src x and y"
    print (y,x)
    print " new goal x and y"
    print (gy,gx)
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.sqrt(dx**2 + dy**2)

    if d >= MAX_EDGE_LEN:
        return True

    D = rr
    nstep = int(round(d / D))

    for i in range(nstep):
	
        idxs, dist = okdtree.search(np.matrix([x, y]).T)
        if dist[0] <= rr and not pqp_client([x, y, 1], [1,0,0,0,1,0,0,0,1]) :
		print "x and y is"
	        print (x,y)            
		return True  # collision
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    # goal point check
    idxs, dist = okdtree.search(np.matrix([gx, gy]).T)
    if dist[0] <= rr:
        return True  # collision

    return False  # OK


def generate_roadmap(sample_x, sample_y, rr, obkdtree):
    """
    Road map generation

    sample_x: [m] x positions of sampled points
    sample_y: [m] y positions of sampled points
    rr: Robot Radius[m]
    obkdtree: KDTree object of obstacles
    """

    road_map = []
    nsample = len(sample_x)
    skdtree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(nsample), sample_x, sample_y):

        index, dists = skdtree.search(
            np.matrix([ix, iy]).T, k=nsample)
        inds = index[0][0]
        edge_id = []
        #  print(index)
        #print "inds are: "+str(inds)
        for ii in range(1, len(inds)):
            nx = sample_x[inds[ii]]
            ny = sample_y[inds[ii]]
   	    R = [1,0,0,0,1,0,0,0,1]
            if not is_collision(ix, iy, nx, ny, rr, obkdtree):
		#print (inds[ii])
                edge_id.append(inds[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    #  plot_road_map(road_map, sample_x, sample_y)
    #print road_map
    return road_map


def dijkstra_planning(sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(sx, sy, 0.0, -1)
    ngoal = Node(gx, gy, 0.0, -1)

    openset, closedset = dict(), dict()
    openset[len(road_map) - 2] = nstart

    while True:
        if len(openset) == 0:
            print("Cannot find path")
            break

        c_id = min(openset, key=lambda o: openset[o].cost)
        current = openset[c_id]
        #print (current.x, current.y, current.cost, current.pind)
        # show graph
        if show_animation and len(closedset.keys()) % 2 == 0:
            plt.plot(current.x, current.y, "xg")
            plt.pause(0.001)

        if c_id == (len(road_map) - 1):
            print("goal is found!")
           # print "current.pind is: "+str(current.pind)
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.sqrt(dx**2 + dy**2)
            node = Node(sample_x[n_id], sample_y[n_id],
                        current.cost + d, c_id)

            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    # generate final course
    rx, ry = [ngoal.x], [ngoal.y]
    pind = ngoal.pind
    #print "pind is: "+str(pind)
    while pind != -1:
        #print "pind is: " + str(pind)
        n = closedset[pind]
        rx.append(n.x)
        ry.append(n.y)
        pind = n.pind

    return rx, ry


def plot_road_map(road_map, sample_x, sample_y):

    for i in range(len(road_map)):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]

            plt.plot([sample_x[i], sample_x[ind]],
                     [sample_y[i], sample_y[ind]], "-k")


def sample_points(sx, sy, gx, gy, rr, ox, oy, obkdtree):
    maxx = max(ox)
    maxy = max(oy)
    minx = min(ox)
    miny = min(oy)

    sample_x, sample_y = [], []

    while len(sample_x) <= N_SAMPLE:
        tx = (random.random() - minx) * (maxx - minx)
        ty = (random.random() - miny) * (maxy - miny)
        #print (tx, ty)
        index, dist = obkdtree.search(np.matrix([tx, ty]).T)

        if dist[0] >= rr:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y

def distance(x1, x2, y1, y2):
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist
def prm():
    #print(__file__ + " start!!")

    # start and goal position
    sx = 5.0  # [m]      	# sx is the y coordinate of source from top to bottom but not x
    sy = 9.0  # [m] 	# sy is the x coordinate of source from left to right but not y
    #gx = 1.0  # [m]
    #gy = 10 - 7.0  # [m]
    gx = 2.0	   		# gx is the y coordinate of goal point not x from left to right 
    gy = 3.0      		# gy is the x coordiante of goal point not y from left to right 
    robot_size = 0.5  # [m]
    #print pqp_client([gy, gx, 1], [1,0,0,0,1,0,0,0,1])

    ox = []
    oy = []

    for i in np.arange(2, 10, 0.2):
        ox.append(0)
        oy.append(i)
    for i in np.arange(0,5.8, 0.2):
        ox.append(i)  
        oy.append(2)
    for i in np.arange(5.8, 10, 0.2):
        ox.append(i)
        oy.append(0)
    for i in np.arange(10,12, 0.2):
         oy.append(i)
         ox.append(6)
    for i in np.arange(0, 6, 0.2):
        ox.append(i)
        oy.append(10)
    for i in np.arange(6, 10, 0.2):
        oy.append(12)
        ox.append(i)
    for i in np.arange(0,12, 0.2):
        ox.append(10)
        oy.append(i)
    for i in np.arange(0,2, 0.2):
        ox.append(5.8)
        oy.append(i)
    # for i in range(40):
    #     ox.append(20.0)
    #     oy.append(i)
    # for i in range(40):
    #     ox.append(40.0)
    #     oy.append(60.0 - i)

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "^r")
        plt.plot(gx, gy, "^c")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = PRM_planning(gx, gy, sx, sy, ox, oy, robot_size)
    #print rx
    #print ry
    print ""
    path = []
    print "the result path is: "
    for i in range(0, len(rx)):
        path.append((rx[i], ry[i]))
        print (rx[i], ry[i])
    dist = 0
    for i in range(0, len(path)-1):
        dist = dist+distance(path[i][0], path[i+1][0], path[i][1], path[i+1][1])
    print "distance is: " + str(dist)
    assert len(rx) != 0, 'Cannot found path'



    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()

    return path





def talker(path):


    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    temp = path
	
    x = ModelState()
    x.model_name = "piano2"
    x.pose = Pose()
	

    x.pose.orientation.y = 0
    x.pose.orientation.w = 1.0
    x.pose.orientation.x = 0
    x.pose.orientation.z = 0	

    
    while not rospy.is_shutdown():

        x.twist = Twist()
	x.twist.linear.y = 0
	x.twist.angular.y=0
    	
        
	for i in range(0, len(temp)):
       	    x.pose.position.x = temp[i][0]
	    x.pose.position.y = temp[i][1]
	    x.pose.position.z = 0.31
	    #angle_to_turn = atan2 (temp[i+1][1]-temp[i][1], temp[i+1][0]-temp[i+1][0])*math.pi/180
	    x.pose.orientation.z =  random.uniform(0, 1)           
           




	    #x.twist = Twist()
	    #x.twist.linear.x = randint(-60, 60)*math.pi/180
	    
	    rospy.loginfo(x)
            pub.publish(x)

	    time.sleep(0.3)
	   # print "one loop done"
	    
	exit()

if __name__ == '__main__':
    try:
        talker(prm())
    except rospy.ROSInterruptException:
        pass
