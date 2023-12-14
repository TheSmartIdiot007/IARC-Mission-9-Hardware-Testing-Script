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
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
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

    return np.array([phi, theta, psi]).T


class FLIGHT_CONTROLLER:

	def __init__(self):
		
		#NODE
		rospy.init_node('iris_drone', anonymous = True)

		#SUBSCRIBERS																	
		self.get_position = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
		self.get_linear_vel = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.get_vel)
		self.get_imu_data_acc = rospy.Subscriber('/mavros/imu/data', Imu, self.get_acc)
		self.get_imu_attitude = rospy.Subscriber('/mavros/imu/data', Imu, self.get_attitude)

		#PUBLISHERS
		self.publish_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,queue_size=1)
		self.publish_pos_tar = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget,queue_size=1)
		self.publish_acc = rospy.Publisher('/mavros/setpoint_accel/accel', Vector3, queue_size=1)

		#SERVICES
		self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
		self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
		self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

		rospy.loginfo('INIT')


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
		rate = rospy.Rate(20)
		PS = PoseStamped()
		PS.pose.position.x = self.pose[0]
		PS.pose.position.y = self.pose[1]
		PS.pose.position.z = self.pose[2]
		for i in range(10):
			self.publish_pose.publish(PS)	
			rate.sleep()
		print('done')
		self.set_mode("GUIDED")

	def get_pose(self, location_data):
		pose = location_data.pose.position
		self.pose = np.array([pose.x, pose.y, pose.z]).T

	def get_vel(self, vel_data):
		vel = vel_data.twist.linear
		self.vel = np.array([vel.x, vel.y, vel.z]).T

	def get_acc(self, imu_data_acc):
		acc = imu_data_acc.linear_acceleration
		self.acc = np.array([acc.x, acc.y, acc.z - 9.80665])

	def get_attitude(self, imu_attitude):
		att_q = np.array([imu_attitude.orientation.w, imu_attitude.orientation.x, imu_attitude.orientation.y, imu_attitude.orientation.z])
		self.att = quaternion_to_euler(att_q)


	#PUBLISHERS
	
	def gotopose(self, pose_tar):
		
		rate = rospy.Rate(20)
		sp = PoseStamped()

		sp.pose.position.x = pose_tar[0]
		sp.pose.position.y = pose_tar[1]
		sp.pose.position.z = pose_tar[2]

		sp.pose.orientation.x = 0.0
		sp.pose.orientation.y = 0.0
		sp.pose.orientation.z = 0.0
		sp.pose.orientation.w = 1.0

		dist = np.sqrt(((self.pose[0] - pose_tar[0])**2) + ((self.pose[1] - pose_tar[1])**2) + ((self.pose[2] - pose_tar[2])**2))

		while(dist > 0.2):
			self.publish_pose.publish(sp)
			dist = np.sqrt(((self.pose[0] - pose_tar[0])**2) + ((self.pose[1] - pose_tar[1])**2) + ((self.pose[2] - pose_tar[2])**2))
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

	def set_acc(self, a):

		sp = Vector3()
		sp.x = a[0]
		sp.y = a[1]
		sp.z = a[2]

		self.publish_acc.publish(sp)



if __name__ == '__main__':

    mav = FLIGHT_CONTROLLER()
    time.sleep(3)
    mav.set_mode('STABILIZE')
    mav.toggle_arm(1)
    time.sleep(2)
    mav.set_Guided_mode()
    mav.takeoff(5)
    time.sleep(5)
    #mav.gotopose(np.array([0,5,5]))
    #print('reached position')
    

    rate = rospy.Rate(20)
    for i in range(100):
        print('in loop')
        mav.gotopose(np.array([0.0,2.0,5.0]))
        rate.sleep()
        #print(mav.pose)

    print('exited')
    time.sleep(2)
    mav.land(5)
	
