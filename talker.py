#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose,Twist
from tf.transformations import euler_from_quaternion
from math import atan2
import random
import math
import copy
import numpy as np
import time



class Sample():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT():


    def __init__(self, start, goal, boundary, obstacleList, expandDist=0.5, goalchance=0):
        
	self.start = Sample(start[0], start[1])
        self.goal = Sample(goal[0], goal[1])
        self.minrand = boundary[0]
        self.maxrand = boundary[1]
        self.expandDist = expandDist
        self.goalchance = goalchance
        self.obstacleList = obstacleList

    def RRT_generate(self):
 
        self.sampleList = [self.start]
        while True:
 
            if random.randint(0, 100) > self.goalchance:		             # allowing random chance toward goal	
                rnd = [random.uniform(self.minrand, self.maxrand)*10, random.uniform(
                    self.minrand, self.maxrand)*10]
                #print "if: " +str(rnd)
            else:

                rnd = [self.goal.x, self.goal.y]
                #print "else: "+ str(rnd)

           # print "Random sample is: " + str((rnd[0], rnd[1]))
            # Find nearest sample
            nindex = self.GetNearestListIndex(self.sampleList, rnd)
            #print(nindex)

            
            nearestSample = self.sampleList[nindex]			#expanding tree
           # print "nearest sample is: " + str((nearestSample.x, nearestSample.y))
            theta = math.atan2(rnd[1] - nearestSample.y, rnd[0] - nearestSample.x)
           # print(rnd[1]-nearestSample.y, rnd[0]-nearestSample.x)
           # print(theta)

            #print "nearestSample is: "+str((nearestSample.x, nearestSample.y))
            #print (nearestSample.x, nearestSample.y)
            sample = copy.deepcopy(nearestSample)
          #  print(sample.x, sample.y)
            sample.x += self.expandDist * math.cos(theta)

            sample.y += self.expandDist * math.sin(theta)
           # print "new sample is: "+ str((sample.x, sample.y))

            sample.parent = nindex

            if not self.CollisionCheck(sample, self.obstacleList):
               # print "the new sample is in collision"
                continue

            self.sampleList.append(sample)
            #print("sampleList:", len(self.sampleList))

            
            dx = sample.x - self.goal.x
            dy = sample.y - self.goal.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDist:
              #  print("Reach Goal!!!")
                break

        path = [[self.goal.x, self.goal.y]]
        lastIndex = len(self.sampleList) - 1
        while self.sampleList[lastIndex].parent is not None:
            sample = self.sampleList[lastIndex]
            path.append([sample.x, sample.y])
            lastIndex = sample.parent
        path.append([self.start.x, self.start.y])

        return path

 

    def GetNearestListIndex(self, sampleList, rnd):
        #for i in range(0, len(sampleList)):
           # print (sampleList[i].x, sampleList[i].y)
        #print ""
        dlist = [(temp.x - rnd[0]) ** 2 + (temp.y - rnd[1])
                 ** 2 for temp in sampleList]
        #print "dlist is: "+ str(dlist)
        #print(rnd)
        #print (sample.x, sample.y)
        minind = dlist.index(min(dlist))
        return minind

    def CollisionCheck(self, sample, obstacleList):

        for (ox, oy) in obstacleList:
            dx = ox - sample.x
            dy = oy - sample.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= 1:
                return False  # collision

        return True  # safe


def distance(x1, x2, y1, y2):
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist

def ackermann():
   # print("start simple RRT path planning")

    # ====Search Path with RRT====
    obstacleList = []

    for i in np.arange(-7.5, 6.5, 0.5):
        obstacleList.append((-9, i))
        obstacleList.append((10, i))
    for i in np.arange(-9, 10, 0.5):
        obstacleList.append((i, -7.5))
        obstacleList.append((i, 6.5))

    obstacleList.append((-4.5, 1))
    obstacleList.append((-4.2, 1))
    obstacleList.append((-4.5, -7.5))
    obstacleList.append((-4.2, -7.5))
    obstacleList.append((-4.5, -7.5))
    obstacleList.append((-4.2, -7.5))
    for i in np.arange(-7.5, 1, 0.5):
        obstacleList.append((-4.5, i))
        obstacleList.append((-4.2, i))
    obstacleList.append((1.2, 6.5))
    obstacleList.append((1.5, -1.5))
    obstacleList.append((1.2, -1.5))
    obstacleList.append((1.5, 6.5))
    for i in np.arange(-1.5, 6.5, 0.5):
        obstacleList.append((1.2, i))
        obstacleList.append((1.5, i))

    obstacleList.append((6, 2.9))
    obstacleList.append((6.3, 2.9))
    obstacleList.append((6, -4.2))
    obstacleList.append((6.3, -4.2))
    for i in np.arange(-4.2, 2.9, 0.5):
        obstacleList.append((6, i))
        obstacleList.append((6.3, i))

    #test box
    # for i in np.arange(-10, 10, 0.5):
    #     obstacleList.append((i, 10))
    #     obstacleList.append((i, -8))
    # for i in np.arange(-8, 10, 0.5):
    #     obstacleList.append((-8, i))
    #     obstacleList.append((8, i))
    #print obstacleList
    start = [-7.5, -6]
    goal = [9, 5]
    # [x,y,size]
    # Set Initial parameters
    #startT = time.time()
    rrt = RRT(start=start, goal=goal, boundary=[-9, 10], obstacleList=obstacleList)
    #end = time.time()
    #print "time of execution is"
    #print end - startT
    path = rrt.RRT_generate()
    path.reverse()

    print "the path is: "
    for i in range(0, len(path)):
        print path[i]
    # Draw final path
    #dist = 0
    #for i in range(0, len(path) - 1):
        #dist = dist + distance(path[i][0], path[i + 1][0], path[i][1], path[i + 1][1])
    #print "distance is: " + str(dist)
 
	
    return path



def talker(path):
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    temp = path
	
    x = ModelState()
    x.model_name = "ackermann_vehicle"
    x.pose = Pose()
	

    x.pose.orientation.y = 0
    x.pose.orientation.w = 1.0
    x.pose.orientation.x = 0
    x.pose.orientation.z = 0

    




    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        x.twist = Twist()
	x.twist.linear.y = 0
	x.twist.angular.y=0
    	
        
	for i in range(0, len(temp)):
       	    x.pose.position.x = temp[i][0]
	    x.pose.position.y = temp[i][1]
	    x.pose.position.z = 0.14
	    #angle_to_turn = atan2 (temp[i+1][1]-temp[i][1], temp[i+1][0]-temp[i+1][0])*math.pi/180
	    x.pose.orientation.z =  random.uniform(0, 1)
  	    rot_q = x.pose.orientation
            (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
            
           
	    

            if(i < len(temp) - 1):
	    	angle_to_goal = atan2 (temp[i+1][1]-temp[i][1], temp[i+1][0]-temp[i+1][0])


	    	if abs(angle_to_goal - theta) > 0.1:
	            x.twist.linear.x = 0.0
		    x.twist.angular.z = 0.0
	        else:
		    x.twist.linear.x = 0.5
		    x.twist.angular.z = 0.0



	    #x.twist = Twist()
	    #x.twist.linear.x = randint(-60, 60)*math.pi/180
	    
	    rospy.loginfo(x)
            pub.publish(x)

	    time.sleep(0.3)
	   # print "one loop done"
	    
	exit()


if __name__ == '__main__':
    try:
        talker(ackermann())
    except rospy.ROSInterruptException:
        pass
