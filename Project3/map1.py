#!/usr/bin/env python
from __future__ import division
from turtlebot_ctrl.srv import TurtleBotControl
from std_msgs.msg import Bool, Float32
import rospy
import numpy as np
import matplotlib as mpl
mpl.use('TkAgg')
import matplotlib.pyplot as plt
import random
import math
from decimal import Decimal
import time
import tf
import copy
import matplotlib.patches as mpatches
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from kobuki_msgs.msg import BumperEvent
from turtlebot_ctrl.srv import TurtleBotControl, TurtleBotControlResponse
from turtlebot_ctrl.msg import TurtleBotScan
from copy import deepcopy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

figure = 1
n =100 #number of particles
start = []
particles = {}

boundary = [(4,4),(4,-7),(-7,-7),(-7,4)]
obstacle1 = [(-1,-1),(-5,-1),(-5,2),(-3,2)]
obstacle2 = [(-2,2),(2,2),(2,-4)]
obstacle3 = [(-1,-2.5),(1.2,-4),(1.2,-5),(-5,-5),(-5,-2.5)]

nan = 11

def inside_polygon(x, y, points):

    n = len(points)
    inside = False
    p1x, p1y = points[0]
    for i in range(1, n + 1):
        p2x, p2y = points[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def outside_polygon(x,y,points):
    return not inside_polygon(x, y, points)

def mydistance(x1, x2, y1, y2):
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist

def drawparticle(x,y,orientation):
    circle = plt.Circle((x, y), 0.15, facecolor='#ffb266', edgecolor='#994c00', alpha=0.5)
    plt.gca().add_patch(circle)
    ax = plt.axes()
    ax.arrow(x, y, 0.5 * math.cos(orientation), 0.5 * math.sin(orientation), head_width=0.2, head_length=0.4, fc='#994c00', ec='#994c00')

def drawstart(x,y,orientation):
    circle = plt.Circle((x, y), 0.15, facecolor='r', edgecolor='r', alpha=0.5)
    plt.gca().add_patch(circle)
    ax = plt.axes()
    ax.arrow(x, y, 0.5 * math.cos(orientation), 0.5 * math.sin(orientation), head_width=0.2, head_length=0.4, fc='r', ec='r')

def drawprediction(x,y,orientation):
    circle = plt.Circle((x, y), 0.15, facecolor='g', edgecolor='g', alpha=0.5)
    plt.gca().add_patch(circle)
    ax = plt.axes()
    ax.arrow(x, y, 0.5 * math.cos(orientation), 0.5 * math.sin(orientation), head_width=0.2, head_length=0.4, fc='g', ec='g')


	

def randomdata():
    result = []
    for i in range(54):
        if random.randint(0,10)>4:
            result.append(nan)
        else:
            result.append(random.uniform(0.45, 10))
    return result

def calculateweight(observedata, correctdata):
    weight = 0
    for i in range(len(observedata)):
        weight = weight + math.e**(-1*abs(observedata[i]-correctdata[i]))
    return weight







def convertscandata(scan_data):
	arr= str(scan_data).split(":")[1]
	arr2= arr.split("\n")[0]
	arr3= arr2.replace("[","")
	arr4= arr3.replace("]","")
	arr4=arr4.replace(" ","")
	arr5=arr4.split(",",100)
	for index,value in enumerate (arr5):
		if value=="nan":
			pass
		else:
		   	arr5[index]=float(value)
	for i in range(len(arr5)):
  	    	if arr5[i] == 'nan':
                    arr5[i] = nan
	return arr5


#def shiftingScanData(scan_data):
"""
#version1
def generateParticles(self,n):
    global particles
    modelstate = self.get_model_state(model_name="mobile_base")
    currx = modelstate.pose.position.x
    curry = modelstate.pose.position.y
    roll = modelstate.pose.orientation.x
    pitch = modelstate.pose.orientation.y
    yaw = modelstate.pose.orientation.z
    w = modelstate.pose.orientation.w
    orientation_list = [roll, pitch, yaw,w]
    (roll, pitch, origyaw) = euler_from_quaternion(orientation_list)
    
    #have to set it back to curryaw
    for i in range(n):
        x = random.uniform(-7, 4)
        y = random.uniform(-7,4)
        if inside_polygon(x,y, obstacle1) or inside_polygon(x,y, obstacle2) or inside_polygon(x,y, obstacle3):
            continue
        yaw = math.radians(random.randint(0, 360))
	self.set_heading_angle(yaw)
	self.set_position((x,y))
	scan_data = rospy.wait_for_message("/turtlebot_scan",TurtleBotScan)
	scan_data = convertscandata(scan_data)
        particles[(x,y,yaw)] =[scan_data,0]
    
    self.set_heading_angle(origyaw)
    self.set_position((currx, curry))
"""
#rui version
def generateParticles(self,n):
    global particles
    modelstate = self.get_model_state(model_name="mobile_base")
    currx = modelstate.pose.position.x
    curry = modelstate.pose.position.y
    roll = modelstate.pose.orientation.x
    pitch = modelstate.pose.orientation.y
    yaw = modelstate.pose.orientation.z
    w = modelstate.pose.orientation.w
    orientation_list = [roll, pitch, yaw,w]
    (roll, pitch, origyaw) = euler_from_quaternion(orientation_list)
    #have to set it back to curryaw
    for i in range(n):
        x = random.uniform(-7, 4)
        y = random.uniform(-7,4)
        if inside_polygon(x,y, obstacle1) or inside_polygon(x,y, obstacle2) or inside_polygon(x,y, obstacle3):
            continue
        
	
	self.set_position((x,y))
	scan_data = rospy.wait_for_message("/turtlebot_scan",TurtleBotScan)
	scan_data = convertscandata(scan_data)
        particles[(x,y,origyaw)] =[scan_data,0]
    

    self.set_position((currx, curry))


def updateparticlesXY(self, distance):
    global particles

    modelstate = self.get_model_state(model_name="mobile_base")
    currx = modelstate.pose.position.x
    curry = modelstate.pose.position.y
    roll = modelstate.pose.orientation.x
    pitch = modelstate.pose.orientation.y
    yaw = modelstate.pose.orientation.z
    w = modelstate.pose.orientation.w
    orientation_list = [roll, pitch, yaw,w]
    (roll, pitch, origyaw) = euler_from_quaternion(orientation_list)
    
    
   

    for key in particles.keys():
	#this may need to be degree
	newx = distance*math.cos(key[2]) 
	newy = distance*math.sin(key[2])
	dg = mydistance(key[0], newx+key[0], key[1], newy+key[1])
	#print "the orientation: "+str(key[2])
        print "the distance the particle moved: "+str(dg)
        particles[(key[0]+newx, key[1]+newy, key[2])] = particles.pop(key)
	newx=0
	newy=0
    for key in particles.keys():
        if inside_polygon(key[0], key[1], obstacle1) or inside_polygon(key[0], key[1], obstacle2) or inside_polygon(key[0], key[1], obstacle3) or outside_polygon(key[0], key[1], boundary):
            del particles[key]
            continue
        x = key[0]
        y = key[1]
        yaw = key[2]
        self.set_heading_angle(yaw)
	self.set_position((x,y))
	scan_data = rospy.wait_for_message("/turtlebot_scan",TurtleBotScan)
	scan_data = convertscandata(scan_data)
        particles[key] =[scan_data,0]
    self.set_heading_angle(origyaw)
    self.set_position((currx, curry))
"""
#version 1
def updateparticlesOri(self, Orientation):
    global particles
    modelstate = self.get_model_state(model_name="mobile_base")
    currx = modelstate.pose.position.x
    curry = modelstate.pose.position.y
    roll = modelstate.pose.orientation.x
    pitch = modelstate.pose.orientation.y
    yaw = modelstate.pose.orientation.z
    w = modelstate.pose.orientation.w
    orientation_list = [roll, pitch, yaw,w]
    (roll, pitch, origyaw) = euler_from_quaternion(orientation_list)
    for key in particles.keys():
        particles[(key[0], key[1], key[2]+Orientation)] = particles.pop(key)

    for key in particles.keys():
        if inside_polygon(key[0], key[1], obstacle1) or inside_polygon(key[0], key[1], obstacle2) or inside_polygon(key[0], key[1], obstacle3) or outside_polygon(key[0], key[1], boundary):
            del particles[key]
            continue
        x = key[0]
        y = key[1]
        yaw = key[2]
        self.set_heading_angle(yaw)
	self.set_position((x,y))
	scan_data = rospy.wait_for_message("/turtlebot_scan",TurtleBotScan)
	scan_data = convertscandata(scan_data)
        particles[key] =[scan_data,0]
    self.set_heading_angle(origyaw)
    self.set_position((currx, curry))
"""
#rui version
def updateparticlesOri(self, Orientation):
    global particles
    modelstate = self.get_model_state(model_name="mobile_base")
    currx = modelstate.pose.position.x
    curry = modelstate.pose.position.y
    roll = modelstate.pose.orientation.x
    pitch = modelstate.pose.orientation.y
    yaw = modelstate.pose.orientation.z
    w = modelstate.pose.orientation.w
    orientation_list = [roll, pitch, yaw,w]
    (roll, pitch, origyaw) = euler_from_quaternion(orientation_list)
    for key in particles.keys():
        particles[(key[0], key[1], origyaw)] = particles.pop(key)

    for key in particles.keys():
        if inside_polygon(key[0], key[1], obstacle1) or inside_polygon(key[0], key[1], obstacle2) or inside_polygon(key[0], key[1], obstacle3) or outside_polygon(key[0], key[1], boundary):
            del particles[key]
            continue
        x = key[0]
        y = key[1]
        yaw = key[2]
        self.set_heading_angle(yaw)
	self.set_position((x,y))
	scan_data = rospy.wait_for_message("/turtlebot_scan",TurtleBotScan)
	scan_data = convertscandata(scan_data)
        particles[key] =[scan_data,0]
    self.set_heading_angle(origyaw)
    self.set_position((currx, curry))

def updateparticlesOri0(self):
    global particles
    modelstate = self.get_model_state(model_name="mobile_base")
    currx = modelstate.pose.position.x
    curry = modelstate.pose.position.y
    roll = modelstate.pose.orientation.x
    pitch = modelstate.pose.orientation.y
    yaw = modelstate.pose.orientation.z
    w = modelstate.pose.orientation.w
    orientation_list = [roll, pitch, yaw,w]
    (roll, pitch, origyaw) = euler_from_quaternion(orientation_list)
    for key in particles.keys():
        particles[(key[0], key[1], 0)] = particles.pop(key)

    for key in particles.keys():
        if inside_polygon(key[0], key[1], obstacle1) or inside_polygon(key[0], key[1], obstacle2) or inside_polygon(key[0], key[1], obstacle3) or outside_polygon(key[0], key[1], boundary):
            del particles[key]
            continue
        x = key[0]
        y = key[1]
        yaw = key[2]
        self.set_heading_angle(yaw)
	self.set_position((x,y))
	scan_data = rospy.wait_for_message("/turtlebot_scan",TurtleBotScan)
	scan_data = convertscandata(scan_data)
        particles[key] =[scan_data,0]
    self.set_heading_angle(origyaw)
    self.set_position((currx, curry))
"""
def updateparticles(self):
    modelstate = self.get_model_state(model_name="mobile_base")
    currx = modelstate.pose.position.x
    curry = modelstate.pose.position.y
    roll = modelstate.pose.orientation.x
    pitch = modelstate.pose.orientation.y
    yaw = modelstate.pose.orientation.z
    w = modelstate.pose.orientation.w
    orientation_list = [roll, pitch, yaw,w]
    (roll, pitch, origyaw) = euler_from_quaternion(orientation_list)
    for key in particles:
        particles[(key[0]+distance, key[1]+distance, key[2])] = particles.pop(key)

    for key in particles.keys():
        if inside_polygon(key[0], key[1], obstacle1) or inside_polygon(key[0], key[1], obstacle2) or inside_polygon(key[0], key[1], obstacle3) or outside_polygon(key[0], key[1], boundary):
            del particles[key]
            continue
        x = key[0]
        y = key[1]
        yaw = key[2]
        self.set_heading_angle(yaw)
	self.set_position((x,y))
	scan_data = rospy.wait_for_message("/turtlebot_scan",TurtleBotScan)
	scan_data = convertscandata(scan_data)
        particles[key] =[scan_data,0]
    self.set_heading_angle(origyaw)
    self.set_position((currx, curry))
"""
    
def iterateshow(self):
    # this is the axis length
    global particles
    global figure
    plt.figure(figure)
    modelstate = self.get_model_state(model_name="mobile_base")
    currx = modelstate.pose.position.x
    curry = modelstate.pose.position.y
    roll = modelstate.pose.orientation.x
    pitch = modelstate.pose.orientation.y
    yaw = modelstate.pose.orientation.z
    w = modelstate.pose.orientation.w
    orientation_list = [roll, pitch, yaw,w]
    (roll, pitch, origyaw) = euler_from_quaternion(orientation_list)
    print "start x is: "+ str(currx)
    print "start y is: "+str(curry)
    print "start angle is: "+str(origyaw)
    #start = [currx, curry, origyaw]
    scan_data = rospy.wait_for_message("/turtlebot_scan",TurtleBotScan)
    scan_data = convertscandata(scan_data)

    drawstart(currx, curry, origyaw)
    plt.axis([-10, 10, -10, 10])

    # this is ploting the boundary of the map
    plt.plot([4, 4, -7, -7, 4], [4, -7, -7, 4, 4], 'b-')

    # obstacle#1
    plt.plot([-1, -5, -5, -3, -1], [-1, -1, 2, 2, -1], 'c-')
    # obstacle#2
    plt.plot([-2, 2, 2, -2], [2, 2, -4, 2], 'c-')
    # obstacle#3
    plt.plot([-1, 1.2, 1.2, -5, -5, -1], [-2.5, -4, -5, -5, -2.5, -2.5], 'c-')
    listofweight = []
    for key in particles.keys():
        
        particles[key][1] = calculateweight(particles[key][0], scan_data)
	if particles[key][1]/54 <0.5:
		print "weight is under 0.8: "+str(particles[key][1])
		del particles[key]
		continue
        listofweight.append(particles[key][1])
	
    print listofweight 

    
    if(len(listofweight)==0):
    	generateParticles(self,n)
	for key in particles.keys():
        
        	particles[key][1] = calculateweight(particles[key][0], scan_data)
		if particles[key][1]/54 <0.5:
			print "weight is under 0.5: "+str(particles[key][1])
			del particles[key]
			continue
        	listofweight.append(particles[key][1])
	max_value = max(listofweight)
    	index = listofweight.index(max_value)
    	for j, (key, value) in enumerate(particles.items()):
            if index == j:
            	prediction = key
    # drawing the prediction
    	drawprediction(prediction[0], prediction[1], prediction[2])
    else:  
    	max_value = max(listofweight)
    	index = listofweight.index(max_value)
    	for j, (key, value) in enumerate(particles.items()):
            if index == j:
            	prediction = key
    # drawing the prediction
    	drawprediction(prediction[0], prediction[1], prediction[2])
	if(len(particles.keys())<n*0.1 and max_value/54<0.9):
    		generateParticles(self,n)
    for key in particles.keys():
    	drawparticle(key[0],key[1],key[2])
    plt.show()
    figure=figure+1

class TurtlebotControlClient:
	def set_heading_angle(self,heading_angle):
		model_state_resp = self.get_model_state(model_name="mobile_base")
		model_state = SetModelState()
		model_state.model_name = "mobile_base"
		model_state.pose = model_state_resp.pose
		model_state.twist = Twist()
		model_state.reference_frame = "world"
		quat = tf.transformations.quaternion_from_euler(0,0,heading_angle)
		model_state.pose.orientation.x = quat[0]
		model_state.pose.orientation.y = quat[1]
		model_state.pose.orientation.z = quat[2]
		model_state.pose.orientation.w = quat[3]
		self.set_model_state(model_state=model_state)

	def set_position(self,position):
		model_state_resp = self.get_model_state(model_name="mobile_base")
		model_state = SetModelState()
		model_state.model_name = "mobile_base"
		model_state.twist = Twist()
		model_state.reference_frame = "world"
		model_state.pose = model_state_resp.pose
		model_state.pose.position.x = position[0]
		model_state.pose.position.y = position[1]
		self.set_model_state(model_state=model_state)
	def __init__(self):
		rospy.init_node("turtlebot_control_client")

		rospy.wait_for_service("turtlebot_control")
		self.turtlebot_control_service = rospy.ServiceProxy("turtlebot_control",TurtleBotControl)
		rospy.wait_for_service("/gazebo/get_model_state")
		self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState)
		
		##trying to set the service for set_model_state
		rospy.wait_for_service("/gazebo/set_model_state")
		self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)

	def run(self):
		global particles
		f_r = open("trajectories.txt", "w+")
		key = ""
		heading = Float32()
		heading.data = 0.0
		distance = Float32()
		return_ground_truth = Bool()
		return_ground_truth.data = True
		scan_data = rospy.wait_for_message("/turtlebot_scan",TurtleBotScan)
		scan_data = convertscandata(scan_data)
		
		print ""
		#print scan_data
		print ""
		modelstate = self.get_model_state(model_name="mobile_base")
		currx = modelstate.pose.position.x
		curry = modelstate.pose.position.y
		print "currx is: "+ str(currx)
		print "curry is: "+str(curry)
		
		#self.set_position((0,1))
		#self.set_heading_angle(3.14)
		"""
		pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
		x = ModelState()
   		x.model_name = "mobile_base"
    		x.pose = Pose()
		x.pose.position.x = 0
    		x.pose.position.y = 1.0
		"""
                #attempting to set model_state
		
		
		while key != 's':
			key = raw_input("PRESS CONTROL KEYS:\n(The rotation keys rotate the turtlebot with respect to x-axis)\nl : +45 degree\na : -45 degree\nt : +90 degree\nv : -90 degree\nj : 0 degree\nf : -180 degree\nh : +135 degree\ng: -135 degree\n\nd : to move 1 cm\nm : to move sqrt(2) cm (diagonally)\n\ns : to stop\n")
			distance.data = 0.0
			upmode = -1
			if key == 'l':
				heading.data = np.pi/4
				upmode = 0
			elif key == 'a':
				heading.data = -np.pi/4
				upmode = 1
				
			elif key == 't':
				heading.data = np.pi/2
				upmode = 2	
				
			elif key == 'v':
				heading.data = -np.pi/2
				upmode = 3
				
			elif key == 'j':
                                heading.data = 0
				upmode = 4
				
                        elif key == 'f':
                                heading.data = -np.pi
				upmode = 5
                        elif key == 'h':
                                heading.data = 3*np.pi/4
				upmode = 6
			elif key == 'g':
                                heading.data = -3*np.pi/4
				upmode = 7
			elif key == 'd':
				distance.data = 1.0
				upmode = 8
			elif key == 'm':
				distance.data = 1.414 
				upmode = 9

			print("Heading: "+str(heading))
			print("Distance: "+str(distance))
			f_r.write("Heading: "+str(heading)+"\n")
			f_r.write("Distance: "+str(distance)+"\n")
			output = self.turtlebot_control_service(heading, distance, return_ground_truth)

			if upmode == 0:
				updateparticlesOri(self,math.radians(45)) 
			elif upmode ==1:
				updateparticlesOri(self, math.radians(-45))
			elif upmode ==2:
				updateparticlesOri(self, math.radians(90))
			elif upmode ==3:
				updateparticlesOri(self, math.radians(-90))
			elif upmode ==4:
				updateparticlesOri0(self)
			elif upmode ==5:
				updateparticlesOri(self, math.radians(-180))
			elif upmode ==6:
				updateparticlesOri(self, math.radians(135))
			elif upmode ==7:
				updateparticlesOri(self, math.radians(-135))
			elif upmode ==8:
				modelstate = self.get_model_state(model_name="mobile_base")
				newx = modelstate.pose.position.x
				newy = modelstate.pose.position.y
				
				dg = 1
				print "the distance the robot moved: "+str(dg)
				updateparticlesXY(self, dg)
			elif upmode == 9:
				modelstate = self.get_model_state(model_name="mobile_base")
				newx = modelstate.pose.position.x
				newy = modelstate.pose.position.y
				
				dg = math.sqrt(2)
				print "the distance the robot moved: "+str(dg)
				updateparticlesXY(self, dg)
			print(output)
			f_r.write(str(output)+"\n")
			
			iterateshow(self)
			
		f_r.close()
		
		rospy.spin()
	def start(self):
	    # this is the starting point
	    global particles
	    global figure
	    plt.figure(figure)
	    modelstate = self.get_model_state(model_name="mobile_base")
	    currx = modelstate.pose.position.x
	    curry = modelstate.pose.position.y
	    roll = modelstate.pose.orientation.x
    	    pitch = modelstate.pose.orientation.y
    	    yaw = modelstate.pose.orientation.z
            w = modelstate.pose.orientation.w
    	    orientation_list = [roll, pitch, yaw,w]
    	    (roll, pitch, origyaw) = euler_from_quaternion(orientation_list)
	    print "start x is: "+ str(currx)
	    print "start y is: "+str(curry)
	    print "start angle is: "+str(origyaw)
            start = [currx, curry, origyaw]
	    scan_data = rospy.wait_for_message("/turtlebot_scan",TurtleBotScan)
	    scan_data = convertscandata(scan_data)

	    
	    drawstart(currx,curry,origyaw)

	    listofweight = []
	    generateParticles(self,n)


	    #this is the axis length
	    plt.axis([-10, 10, -10, 10])

	    #this is ploting the boundary of the map
	    plt.plot([4, 4, -7, -7, 4], [4, -7, -7, 4, 4], 'b-')

	    #obstacle#1
	    plt.plot([-1,-5,-5,-3,-1], [-1,-1,2,2,-1], 'c-')
	    #obstacle#2
	    plt.plot([-2,2,2,-2],[2,2,-4,2],'c-')
	    #obstacle#3
	    plt.plot([-1,1.2,1.2,-5,-5,-1], [-2.5,-4,-5,-5,-2.5,-2.5], 'c-')

	   
	    for key in particles:
		drawparticle(key[0],key[1],key[2])
		particles[key][1] = calculateweight(particles[key][0], scan_data)
		print particles[key][1]
		listofweight.append(particles[key][1])


	    max_value = max(listofweight)
	    index = listofweight.index(max_value)

	    for j, (key, value) in enumerate(particles.items()):
		if index==j:
		    prediction = key
	    #drawing the prediction
	    drawprediction(prediction[0], prediction[1], prediction[2])
	    
	    plt.show()
	    
	    figure = figure+1
if __name__ == "__main__":
	try:
		plt.ion()
		turtlebot_control_client = TurtlebotControlClient()
		turtlebot_control_client.start()
		turtlebot_control_client.run()
	except rospy.ROSInterruptException:
		pass
