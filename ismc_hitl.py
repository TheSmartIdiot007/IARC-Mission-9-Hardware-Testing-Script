#!/usr/bin/env python2.7
import numpy as np
import rospy
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool
from mavros_msgs.msg import AttitudeTarget, PositionTarget
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist, TwistStamped, Wrench, Vector3
from time import sleep
import pickle


## IMPORTANT!! -> This code doesn't take into account of the transformation of angles and body rates yet. 
## 			   -> So, only use the linear acceleration commands for now.


def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[0], r[1], r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def quaternion_to_euler(q):
	w, x, y, z = q[0],q[1], q[2], q[3]
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + y * y)
	phi = np.arctan2(t0, t1)

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	theta = np.arcsin(t2)

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	psi = np.arctan2(t3, t4)

	return np.array([phi, theta, psi]).transpose()

class FLIGHT_CONTROLLER:

	def __init__(self):
		self.pt = Point()

		#NODE
		rospy.init_node('iris_drone', anonymous = True)

		#SUBSCRIBERS																	
		rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
		self.get_linear_vel = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.get_vel)
		self.get_imu_data_acc = rospy.Subscriber('/mavros/imu/data', Imu, self.get_acc)
		self.get_imu_attitude = rospy.Subscriber('/mavros/imu/data', Imu, self.get_attitude)

		#PUBLISHERS
		self.publish_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,queue_size=1)
		self.publish_pos_tar = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget,queue_size=1)
		self.publish_att_tar = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

		#SERVICES
		self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
		self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
		self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

		rospy.loginfo('INIT')

	
	def get_rotmat(self):
		psi_i = self.att[2]
		return np.array([[np.cos(psi_i), -np.sin(psi_i), 0], [np.sin(psi_i), np.cos(psi_i), 0], [0, 0, 1]])

	def toggle_arm(self, arm_bool):
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			self.arm_service(arm_bool)
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)

	def takeoff(self, t_alt):
		rospy.wait_for_service('/mavros/cmd/takeoff')
		try:
			self.takeoff_service(0,0,0,0,t_alt)
			rospy.loginfo('TAKEOFF')
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)
	
	
	def land(self, l_alt):
		rospy.wait_for_service('/mavros/cmd/land')
		try:
			self.land_service(0.0, 0.0, 0, 0, l_alt)
			rospy.loginfo("LANDING")
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)


	def set_mode(self,md):
			rospy.wait_for_service('/mavros/set_mode')
			try:
				self.flight_mode_service(0, md)
				rospy.loginfo("Mode changed")	
			except rospy.ServiceException as e:
				rospy.loginfo("Mode could not be set: " %e)

	def set_Guided_mode(self):
		rate=rospy.Rate(20)
		PS = PoseStamped()
		PS.pose.position.x = self.pt.x
		PS.pose.position.y = self.pt.y
		PS.pose.position.z = self.pt.z
		for i in range(10):
			self.publish_pose.publish(PS)	
			rate.sleep()
		print('done')
		self.set_mode("GUIDED")

	def get_pose(self, location_data):
		self.pt.x = location_data.pose.position.x
		self.pt.y = location_data.pose.position.y
		self.pt.z = location_data.pose.position.z

	def get_vel(self, vel_data):
		vel = vel_data.twist.linear
		self.vel = np.array([vel.x, vel.y, vel.z]).transpose()

	def get_acc(self, imu_data_acc):
		acc = imu_data_acc.linear_acceleration
		self.acc = np.array([acc.x, acc.y, acc.z - 9.80665]).transpose()

	def get_attitude(self, imu_attitude):
		att_q = np.array([imu_attitude.orientation.w, imu_attitude.orientation.x, imu_attitude.orientation.y, imu_attitude.orientation.z])
		self.att = quaternion_to_euler(att_q)


	#PUBLISHERS
	def gotopose(self,x,y,z):
		rate = rospy.Rate(20)
		sp = PoseStamped()
		sp.pose.position.x = x
		sp.pose.position.y = y
		sp.pose.position.z = z
		sp.pose.orientation.x = 0.0
		sp.pose.orientation.y = 0.0
		sp.pose.orientation.z = 0.0
		sp.pose.orientation.w = 1.0
		dist = np.sqrt(((self.pt.x-x)**2) + ((self.pt.y-y)**2) + ((self.pt.z-z)**2))
		while(dist > 0.2):
			self.publish_pose.publish(sp)
			dist = np.sqrt(((self.pt.x-x)**2) + ((self.pt.y-y)**2) + ((self.pt.z-z)**2))
			rate.sleep()
		#print('Reached ',x,y,z)


	def set_pos(self, a):
		sp = PositionTarget()
		sp.coordinate_frame = 1
		sp.type_mask = 3135
		sp.acceleration_or_force.x = a[0]
		sp.acceleration_or_force.y = a[1]
		sp.acceleration_or_force.z = a[2]
		self.publish_pos_tar.publish(sp)
	
	def set_att(self, br, thrust):
		sa = AttitudeTarget()
		sa.type_mask = 128
		# sa.orientation.x = q[0]
		# sa.orientation.y = q[1]
		# sa.orientation.z = q[2]
		# sa.orientation.w = q[3]
		sa.body_rate.x = br[0]
		sa.body_rate.y = br[1]
		sa.body_rate.z = br[2]
		sa.thrust = thrust
		self.publish_att_tar.publish(sa)


class smc:
	
	def __init__(self, m, alpha_1, alpha_2, beta, d_p):
		
		self.g = np.array([0 , 0 , -9.8]).transpose()	        #gravity
		self.m = m										        #mass
		self.alpha_1 = alpha_1
		self.alpha_2 = alpha_2
		self.beta = beta
		self.d_p = d_p

	def controller(self, p, p_dot, p_d, p_dot_d, p_ddot_d, s_int):

		# Numerical Integration
		p_e_u = p_d - p
		v_e_u = p_dot_d - p_dot		
		s_0_u = self.alpha_1*s_int + self.alpha_2*p_e_u + v_e_u

		self.p_e_u = p_e_u
		self.v_e_u = v_e_u
		self.s_0_u = s_0_u

		E = self.alpha_1*p_e_u + self.alpha_2*v_e_u - self.g + p_ddot_d + np.dot(self.d_p,p_dot)/self.m

		sat_s = np.array([0,0,0]).transpose()

		if((s_0_u[0] > -0.05) and (s_0_u[0] < 0.05)):
			sat_s[0] = 0.05*s_0_u[0]/0.05
		elif((s_0_u[0] > -0.3) and (s_0_u[0] < 0.3)):
			sat_s[0] = 0.7*s_0_u[0]/0.3
		else:
			sat_s[0] = 0.85*np.sign(s_0_u[0])

		if((s_0_u[1] > -0.3) and (s_0_u[1] < 0.3)):
			sat_s[1] = 0.7*s_0_u[1]/0.3
		elif((s_0_u[1] > -0.8) and (s_0_u[1] < 0.)):
			sat_s[1] = 1.1*s_0_u[1]/0.8
		else:
			sat_s[1] = 1.5*np.sign(s_0_u[1])
			
		if((s_0_u[2] > -0.01) and (s_0_u[2] < 0.01)):
			sat_s[2] = 0.1*s_0_u[2]/0.01
		elif((s_0_u[2] > -0.08) and (s_0_u[2] < 0.08)):
			sat_s[2] = 0.75*s_0_u[2]/0.08
		else:
			sat_s[2] = 0.95*np.sign(s_0_u[2])


		E_hat = E + sat_s
		self.p_ddot_c = (E_hat + self.g - np.dot(self.d_p,p_dot)/self.m)

if __name__ == '__main__':

	#unpickle the trajectory from its file

	pickle_off = open("1D.txt", "rb")#("Documents/IARC/3D.txt", "rb")			#modify the first input in open() to be the location of the trajectory file, and default is home
	gen = pickle.load(pickle_off)

	[x_path, x_dot_path, x_ddot_path, y_path, y_dot_path, y_ddot_path, z_path, z_dot_path, z_ddot_path, psi_path] = gen


	mav = FLIGHT_CONTROLLER()
	time.sleep(1)
	transform = mav.get_rotmat()
	transform_inv = transform.T
	mav.toggle_arm(1)
	time.sleep(1)
	mav.set_Guided_mode()
	mav.takeoff(7)
	time.sleep(3)
	initpos = np.dot(transform, np.array([x_path[0],y_path[0],z_path[0]]).T)
	mav.gotopose(initpos[0], initpos[1], initpos[2])
	
	m = 1.5												    # mass of the quadrotor
	alpha_1 = 0.25											#0.3
	alpha_2 = 0.9											#1.0
	beta = np.array([0.1,0.5,2,7])
	J_p = np.diag(np.array([4.9e-3, 4.9e-3, 8.8e-3]))		# coefficients of the rotary inertia
	Tau_n = np.array([1, 1, 1]).transpose()					# moments in the body-fixed frame
	d_p = np.diag(np.array([0.00,0.00,0.00]))				# air drag
	d_eta = np.diag(np.array([6e-5,6e-5,6e-5]))				# aerodynamic drag coefficients

	Smc = smc(m, alpha_1, alpha_2, beta, d_p)

	p = np.dot(transform_inv, np.array([mav.pt.x, mav.pt.y, mav.pt.z]).transpose())
	p_dot = np.dot(transform_inv, mav.vel)
	p_ddot = np.dot(transform_inv, mav.acc)

	p_e = np.zeros((3,len(z_path)+1))
	v_e = np.zeros((3,len(z_path)+1))
	s_0 = np.zeros((3,len(z_path)+1))
		
	p_e[:,0] = np.array([x_path[0], y_path[0], z_path[0]]).transpose() - p
	v_e[:,0] = np.array([x_dot_path[0], y_dot_path[0], z_dot_path[0]]).transpose() - p_dot
	s_0[:,0] = alpha_2*p_e[:,0] + v_e[:,0]
	s_int = 0

	p_prev = p

	rate = rospy.Rate(10)
	irate = rospy.Rate(30)

	for iter in range(len(z_path)):
		
		p_d = np.array([x_path[iter], y_path[iter], z_path[iter]]).transpose()
		p_dot_d = np.array([x_dot_path[iter], y_dot_path[iter], z_dot_path[iter]]).transpose()
		p_ddot_d = np.array([x_ddot_path[iter], y_ddot_path[iter], z_ddot_path[iter]]).transpose()
		psi_d = np.arctan2(p_dot_d[1],p_dot_d[0])

		Smc.controller(p, p_dot, p_d, p_dot_d, p_ddot_d, s_int)
		
		for i in range (3):
			mav.set_pos(np.dot(transform, Smc.p_ddot_c))
			p_inst = np.dot(transform_inv, np.array([mav.pt.x, mav.pt.y, mav.pt.z]).transpose())			#instantaneous position
			s_int += (2*p_d - p_inst - p_prev)/60									                        #integral of position error

			p_prev = p_inst
			irate.sleep()
		
		rate.sleep()

		p = np.dot(transform_inv, np.array([mav.pt.x, mav.pt.y, mav.pt.z]).transpose())
		p_dot = np.dot(transform_inv, mav.vel)
		p_ddot = np.dot(transform_inv, mav.acc)

		p_e[:,iter+1] = np.copy(Smc.p_e_u)
		v_e[:,iter+1] = np.copy(Smc.v_e_u)
		s_0[:,iter+1] = np.copy(Smc.s_0_u)
