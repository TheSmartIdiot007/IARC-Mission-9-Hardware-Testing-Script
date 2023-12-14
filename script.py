#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import *
import geometry_msgs
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget

#global variable
latitude =0.0
longitude=0.0


#########################################        CHANGING MODES USING SERVICES

def setGuidedMode():
	rospy.wait_for_service('/mavros/set_mode')
	try:
		flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
		#http://wiki.ros.org/mavros/CustomModes for custom modes
		isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
	except rospy.ServiceException as e:
		print ("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled")%e
		
def setStabilizeMode():
	rospy.wait_for_service('/mavros/set_mode')
	try:
		flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
		isModeChanged = flightModeService(custom_mode='STABILIZE') #return true or false
	except rospy.ServiceException as e:
		print ("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled")%e

def setLandMode():
	rospy.wait_for_service('/mavros/cmd/land')
	try:
		landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
		#http://wiki.ros.org/mavros/CustomModes for custom modes
		isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
	except rospy.ServiceException as e:
		print ("Service takeoff call failed: %sThe vehicle cannot land ")%e
		  
def setArm():
	rospy.wait_for_service('/mavros/cmd/arming')
	try:
		armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
		armService(True)
	except rospy.ServiceException as e:
		print ("Service takeoff call failed: %s"%e)
		
def setDisarm():
	rospy.wait_for_service('/mavros/cmd/arming')
	try:
		armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
		armService(False)
	except rospy.ServiceException as e:
		print ("Service takeoff call failed: %s"%e)


def setTakeoffMode(height):
	rospy.wait_for_service('/mavros/cmd/takeoff')
	try:
		takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
		takeoffService(altitude = height, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
	except rospy.ServiceException as e:
		print ("Service takeoff call failed: %s"%e)










##############################################  MANEUVERING THROUGH PUBLISHER/ SUBSCRIBERS


def setGui_Arm_Tkf(height):
	setGuidedMode()
	setArm()
	setTakeoffMode(height)


'''
def localPosition():
	loc_pos = listener()
	print(loc_pos)
	local_point = Point()
	local_point.x = loc_pos[0]
	local_point.y = loc_pos[1]
	local_point.z = loc_pos[2]
	return local_point
'''

x_current = 0
y_current = 0
z_current = 0


def callback(data):
	#rospy.loginfo( rospy.get_caller_id() + "I heard %s",data.data )
	global x_current, y_current, z_current
	#print(z_current,"before")
	x_current, y_current, z_current = data.pose.position.x, data.pose.position.y, data.pose.position.z
	#print(z_current,"after")

def listener ():
	sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped ,callback )
	#rospy.spin()

import numpy.linalg as LA
import numpy as np
from math import sqrt


def positionTarget(goal):
	pub = rospy.Publisher("/mavros/setpoint_position/local" , PoseStamped , queue_size=10)
	rate = rospy.Rate(10)
	current = (x_current, y_current, z_current)
	print(x_current, y_current, z_current)
	pos = (goal[0],goal[1],goal[2])
	distance = LA.norm((pos) - np.array(current))
	print(distance)
	while distance > 0.1:
		listener()
		current = (x_current, y_current, z_current)
		#print(distance)
		#print(goal)
		#print(current)
		ps_copy = PoseStamped()
		ps_copy.pose.position.x = goal[0]
		ps_copy.pose.position.y = goal[1]
		ps_copy.pose.position.z = goal[2]
		
		distance = LA.norm((pos) - np.array(current))

		rospy.loginfo(ps_copy)
		pub.publish(ps_copy)
		rate.sleep()
	else :
		return
		


#coordinate system right hand system green --> -x, red --> y , blue -->z

# setpoint from the waypoint astar script
#move toward setpoint , when within specific region of waypoint, move to next waypoint
	
	

def globalPositionCallback(globalPositionCallback):
	global latitude
	global longitude
	latitude = globalPositionCallback.latitude
	longitude = globalPositionCallback.longitude
	#print ("longitude: %.7f" %longitude)
	#print ("latitude: %.7f" %latitude)






#############################################    WAYPOINT GENERATOR
from enum import Enum
from queue import PriorityQueue
import numpy as np

def hello():
	print("hello")

def waypoint_gen():

	sd = 11
	map = np.zeros((100,100,100))
	for i in range(30 - sd,30 + sd):
		for j in range(50 -sd,50 + sd):
			for k in range(0,100):
				map[i][j][k] = 1
	for i in range(20 - sd,20 + sd):
		for j in range(20 - sd,20 + sd):
			for k in range(0,100):
				map[i][j][k] = 1
	for i in range(50 - sd,50 + sd):
		for j in range(0,0+ sd):
			for k in range(0,100):
				map[i][j][k] = 1
	for i in range(60 - sd,60 + sd):
		for j in range(70 - sd , 70 + sd):
			for k in range(0,100):
				map[i][j][k] = 1
	for i in range(50 - sd,50 + sd):
		for j in range(30 - sd,30 + sd):
			for k in range(0,100):
				map[i][j][k] = 1


	hello()
	listener()
	start = (x_current, y_current, 10)
	goal = (90,90,10)

	
	def heuristic_func(position, goal_position):
		# TODO: write a heuristic!
		dx = abs(goal_position[1] - position[1])*1
		dy = abs(goal_position[0] - position[0])*1

		return ( np.sqrt(dx**2 + dy**2))


	

	# Quadroter assume all actions cost the same.
	class Action(Enum):
		"""
		An action is represented by a 3 element tuple.

		The first 2 values are the delta of the action relative
		to the current grid position. The third and final value
		is the cost of performing the action.
		"""

		WEST = (0, -1, 1 , 0)
		EAST = (0, 1, 1 , 0)
		NORTH = (-1, 0, 1 , 0)
		SOUTH = (1, 0, 1 , 0)
		NORTH_WEST = (-1, -1, np.sqrt(2) , 0)
		NORTH_EAST = (-1, 1, np.sqrt(2) , 0)
		SOUTH_WEST = (1, -1, np.sqrt(2) , 0)
		SOUTH_EAST = (1, 1, np.sqrt(2) , 0)
		UP = (0, 0, 1, 1)
		DOWN = (0, 0, 1, -1)

		@property
		def cost(self):
			return self.value[2]

		@property
		def delta(self):
			return (self.value[0], self.value[1], self.value[3])


	def valid_actions(voxmap, current_node):
		"""
		Returns a list of valid actions given a grid and current node.
		"""
		valid_actions = list(Action)
		n, m, l = voxmap.shape[0] - 1, voxmap.shape[1] - 1, voxmap.shape[2] - 1
		x, y, z = current_node

		# check if the node is off the grid or
		# it's an obstacle

		if x - 1 < 0 or voxmap[x - 1, y, z] :
			valid_actions.remove(Action.NORTH)
		if x + 1 > n or voxmap[x + 1, y, z] :
			valid_actions.remove(Action.SOUTH)
		if y - 1 < 0 or voxmap[x, y - 1, z] :
			valid_actions.remove(Action.WEST)
		if y + 1 > m or voxmap[x, y + 1, z] :
			valid_actions.remove(Action.EAST)

		if (x - 1 < 0 or y - 1 < 0) or voxmap[x - 1, y - 1, z] :
			valid_actions.remove(Action.NORTH_WEST)
		if (x - 1 < 0 or y + 1 > m) or voxmap[x - 1, y + 1, z] :
			valid_actions.remove(Action.NORTH_EAST)
		if (x + 1 > n or y - 1 < 0) or voxmap[x + 1, y - 1, z] :
			valid_actions.remove(Action.SOUTH_WEST)
		if (x + 1 > n or y + 1 > m) or voxmap[x + 1, y + 1, z] :
			valid_actions.remove(Action.SOUTH_EAST)

		if (z + 1 > l or voxmap[x, y, z + 1]) :
			valid_actions.remove(Action.UP)
		if (z - 1 < 0 or voxmap[x, y, z - 1]) :
			valid_actions.remove(Action.DOWN)
		
		return valid_actions


	def a_star3D(voxmap, heuristic_func, start, goal):
		"""
		Given a grid and heuristic function returns
		the lowest cost path from start to goal.
		"""

		path = []
		path_cost = 0
		queue = PriorityQueue()
		queue.put((0, start))
		visited = set(start)

		branch = {}
		found = False

		while not queue.empty():
			item = queue.get()
			current_cost = item[0]
			current_node = item[1]

			if current_node == goal:
				print('Found a path.')
				found = True
				break
			else:
				# Get the new vertexes connected to the current vertex
				for a in valid_actions(voxmap , current_node):
					next_node = (current_node[0] + a.delta[0], current_node[1] + a.delta[1], current_node[2] + a.delta[2])
					new_cost = current_cost + a.cost + heuristic_func(next_node, goal)

					if next_node not in visited:
						visited.add(next_node)
						queue.put((new_cost, next_node))

						branch[next_node] = (new_cost, current_node, a)

		if found:
			# retrace steps
			n = goal
			path_cost = branch[n][0]
			path.append(goal)
			while branch[n][1] != start:
				path.append(branch[n][1])
				n = branch[n][1]
			path.append(branch[n][1])

		return path[::-1], path_cost

	path = a_star3D(map, heuristic_func, start, goal)
	print (path)
	print("this is path")

	def collinearity_3D(p1, p2, p3, epsilon=1e-6): 
		collinear = False
		# TODO: Create the matrix out of three points
		matrix =  np.array([p1, p2, p3])
		# TODO: Calculate the determinant of the matrix.
		det = np.linalg.det(matrix) 
		# TODO: Set collinear to True if the determinant is less than epsilon
		collinear = (det <= epsilon)

		return collinear
	tmp = 1



########################################################################################       only 8 directions here, in graph apply an epsilon difference 


	def prune_path2(path):
		pruned_path = []
		pruned_path.append(path[0])
		slopes = np.zeros((len(path)-1)) 
		check = np.zeros((len(path)-2),dtype='bool')
		for i in range(0, len(path)-1):
			if((path[i+1][0] - path[i][0]) != 0):
				slopes[i] = (path[i+1][1] - path[i][1])/(path[i+1][0] - path[i][0])
			else:
				slopes[i] = 0
		for i in range(0, len(path)-2):
			if (slopes[i] == slopes[i+1]):
				continue
			else:
				pruned_path.append(path[i+1]) 
		pruned_path.append(path[-1])
		pruned_path2 = np.zeros((len(pruned_path),3))
		for i in range(0,len(pruned_path)) :
			print(pruned_path[i])
			pruned_path2[i][0] = pruned_path[i][0]/10.0
			pruned_path2[i][1] = pruned_path[i][1]/10.0
			pruned_path2[i][2] = pruned_path[i][2]/10.0
		return pruned_path2


	def prune_path(path):
		pruned_path = path
		# TODO: prune the path!
		for i in range(1,len(pruned_path)-1):
			print(i)
			if(collinearity_3D((pruned_path[i-1]) , (pruned_path[i]) , (pruned_path[i+1]) )):
				pruned_path.pop(i)

			if (pruned_path[i+1] == goal):
				break
			
		print (pruned_path)
		print("this is pruned path")
		pruned_path2 =  np.zeros((len(pruned_path),3))
		for i in range(0,len(pruned_path)) :
			print(pruned_path[i])
			pruned_path2[i][0] = pruned_path[i][0]/10.0
			pruned_path2[i][1] = pruned_path[i][1]/10.0
			pruned_path2[i][2] = pruned_path[i][2]/10.0
		return pruned_path2

	pruned_path = prune_path2(path[0])
	print(pruned_path)
	print(len(pruned_path))

	'''

	while True :
		tmp_path = prune_path(pruned_path)
		if(len(pruned_path) == len(tmp_path)):
			break
		else:
			pruned_path = tmp_path
	'''

	def man() :
		hello()
		for i in pruned_path:
			positionTarget(i)
	
	man()

##################################################


# for urdf
# +x - red
# +y - green
# +z - blue


#################################################### MAIN LOOP


def menu():
	print ("Press")
	print ("1: to set mode to GUIDED")
	print ("2: to set mode to STABILIZE")
	print ("3: to set mode to ARM the drone")
	print ("4: to set mode to DISARM the drone")
	print ("5: to set mode to TAKEOFF")
	print ("6: to set mode to LAND")
	print ("7: print GPS coordinates")
	print ("8: to set mode to guided-> arming-> takeoff")
	print ("9: to maneuver to goal after takeoff")
	print ("10: MASTER : guided -> arm -> takeoff -> waypoint -> reach goal -> land")

class Point():
	x = 0
	y = 0
	z = 0 
	
def myLoop():
	x='1'
	height = 10
	
	#goal_ps = [-4,-4,4]
	#print(goal)

	while ((not rospy.is_shutdown())and (x in ['1','2','3','4','5','6','7','8','9','10'])):
		menu()
		x = input("Enter your input: ")
		x = str(x)
		if (x=='1'):
			setGuidedMode()
		elif(x=='2'):
			setStabilizeMode()
		elif(x=='3'):
			setArm()
		elif(x=='4'):
			setDisarm()
		elif(x=='5'):
			setTakeoffMode(height)
		elif(x=='6'):
			setLandMode()
		elif(x=='7'):
			global latitude
			global longitude
			print ("longitude: %.7f" %longitude)
			print ("latitude: %.7f" %latitude)
		elif(x=='8'):
			setGui_Arm_Tkf(height)
		elif(x=='9'):
			#listener()
			#positionTarget(goal)
			waypoint_gen()
			#waypoint_gen().man()
		elif(x=='10'):
			setGui_Arm_Tkf(height)
			waypoint_gen()
			setLandMode()
			break

		else: 
			print ("Exit")
		
		# x, y, z
	

if __name__ == '__main__':
	rospy.init_node('dronemap_node', anonymous=True)
	rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
	
	# spin() simply keeps python from exiting until this node is stopped
	
	#listener()
	myLoop()
	#rospy.spin()
