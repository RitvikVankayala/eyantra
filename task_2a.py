'''
*****************************************************************************************
*
*        =================================================
*             Pharma Bot Theme (eYRC 2022-23)
*        =================================================
*                                                         
*  This script is intended for implementation of Task 2A   
*  of Pharma Bot (PB) Theme (eYRC 2022-23).
*
*  Filename:			task_2a.py
*  Created:				
*  Last Modified:		8/10/2022
*  Author:				e-Yantra Team
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			task_2a.py
# Functions:		control_logic, detect_distance_sensor_1, detect_distance_sensor_2
# 					[ Comma separated list of functions in this file ]
# Global variables:	
# 					[ List of global variables defined in this file ]

####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
##############################################################
import  sys
import traceback
import time
import os
import math
from zmqRemoteApi import RemoteAPIClient
import zmq

##############################################################


def left_turn(start_time,end_time,sim,la,ra):
	while(True):
		current_time=sim.getSimulationTime()
		if(start_time<=current_time<=end_time):
			sim.setJointTargetVelocity(la,-2)
			sim.setJointTargetVelocity(ra,2)
		else:
			sim.setJointTargetVelocity(la,2)
			sim.setJointTargetVelocity(ra,2)
			break

def right_turn(start_time,end_time,sim,la,ra):
	while(True):
		current_time=sim.getSimulationTime()
		if(start_time<=current_time<=end_time):
			sim.setJointTargetVelocity(la,2)
			sim.setJointTargetVelocity(ra,-2)
		else:
			sim.setJointTargetVelocity(la,2)
			sim.setJointTargetVelocity(ra,2)
			break

def tilt_right(sim,la,ra,d2):
	while(1):
		if(d2>0.23 and d2<0.19):
			sim.setJointTargetVelocity(la,2)
			sim.setJointTargetVelocity(ra,1.8)

		else:
			break
def tilt_left(sim,la,ra,d2):
	while(1):
		if(d2<0.15 and d2>0.19):
			sim.setJointTargetVelocity(la,1.5)
			sim.setJointTargetVelocity(ra,2)

		else:
			break


def left_turn_check(start_time,end_time,sim,la,ra):
	while(True):
		current_time=sim.getSimulationTime()
		if(start_time<=current_time<=end_time):
			sim.setJointTargetVelocity(la,-2)
			sim.setJointTargetVelocity(ra,2)
		else:
			sim.setJointTargetVelocity(la,0)
			sim.setJointTargetVelocity(ra,0)
			break

def stop_condition(f1,f2,la,ra,sim,turn_duration):
	flag=1
	while(1):
		if(f1==1 and f2==1):
			s=sim.getSimulationTime()
			e=s+turn_duration
			left_turn_check(s,e,sim,la,ra)
			sim.setJointTargetVelocity(la,0)
			sim.setJointTargetVelocity(ra,0)
			if(f1==1 and f2==1):
				flag=0
				break
	return flag



def control_logic(sim):
	"""
	Purpose:
	---
	This function should implement the control logic for the given problem statement
	You are required to actuate the rotary joints of the robot in this function, such that
	it traverses the points in given order

	Input Arguments:
	---
	`sim`    :   [ object ]
		ZeroMQ RemoteAPI object

	Returns:
	---
	None

	Example call:
	---
	control_logic(sim)
	"""
	##############  ADD YOUR CODE HERE  ##############
	
	la=sim.getObjectHandle("/Diff_Drive_Bot/left_joint")
	ra=sim.getObjectHandle("/Diff_Drive_Bot/right_joint")
	sensor1=sim.getObjectHandle("/distance_sensor_1")	
	sensor2=sim.getObjectHandle("/distance_sensor_2")
	turn_duration=1
	
	distance_btw_wheels=0.18
	radius=0.0355
	angular_velocity=2
	linear_velocity=radius*angular_velocity
	angle=math.pi/2
	rate=(2*linear_velocity)/distance_btw_wheels
	turn_duration=angle/rate
	count=0

	while(True):

		if(count==10):
			sim.setJointTargetVelocity(la,0)
			sim.setJointTargetVelocity(ra,0)
			break
		sim.setJointTargetVelocity(la,2)
		sim.setJointTargetVelocity(ra,2)
		f1,d1,_,_,_=sim.readProximitySensor(sensor1)
		f2,d2,_,_,_=sim.readProximitySensor(sensor2)

		

		if(f1==1 and f2==1 and d1<0.225):
			count=count+1
			# sim.setJointTargetVelocity(la,0)
			sim.setJointTargetVelocity(ra,0)
			start_time=sim.getSimulationTime()
			end_time=start_time+turn_duration
			left_turn(start_time,end_time+0.32,sim,la,ra)

		elif(f1==1 and f2==0 and d1<0.225):
			count=count+1
			sim.setJointTargetVelocity(la,0)
			sim.setJointTargetVelocity(ra,0)
			start_time=sim.getSimulationTime()
			end_time=start_time+turn_duration
			right_turn(start_time,end_time+0.2,sim,la,ra)


		if(d2>0.23 and f1==0 and f2 ==1):
			# sim.setJointTargetVelocity(la,0)
			# sim.setJointTargetVelocity(ra,0)
			tilt_right(sim,la,ra,d2)
			sim.setJointTargetVelocity(la,2)
			sim.setJointTargetVelocity(ra,2)
		if(d2<0.12 and f1==0 and f2==1):
			# sim.setJointTargetVelocity(la,0)
			# sim.setJointTargetVelocity(ra,0)
			tilt_left(sim,la,ra,d2)
			sim.setJointTargetVelocity(la,2)
			sim.setJointTargetVelocity(ra,2)
			
		
	##################################################

def detect_distance_sensor_1(sim):
	"""
	Purpose:
	---
	Returns the distance of obstacle detected by proximity sensor named 'distance_sensor_1'

	Input Arguments:
	---
	`sim`    :   [ object ]
		ZeroMQ RemoteAPI object

	Returns:
	---
	distance  :  [ float ]
	    distance of obstacle from sensor

	Example call:
	---
	distance_1 = detect_distance_sensor_1(sim)
	"""
	
	distance = None
	##############  ADD YOUR CODE HERE  ##############
	sensor=sim.getObjectHandle("/distance_sensor_1")	
	_,distance,_,_,_=sim.readProximitySensor(sensor)
	##################################################
	return distance




def detect_distance_sensor_2(sim):
	"""
	Purpose:
	---
	Returns the distance of obstacle detected by proximity sensor named 'distance_sensor_2'

	Input Arguments:
	---
	`sim`    :   [ object ]
		ZeroMQ RemoteAPI object

	Returns:
	---
	distance  :  [ float ]
	    distance of obstacle from sensor

	Example call:
	---
	distance_2 = detect_distance_sensor_2(sim)
	"""
	distance = None
	##############  ADD YOUR CODE HERE  ##############
	sensor=sim.getObjectHandle("/distance_sensor_2")	
	_,distance,_,_,_=sim.readProximitySensor(sensor)	
	##################################################
	return distance

######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THE MAIN CODE BELOW #########

if __name__ == "__main__":
	client = RemoteAPIClient()
	sim = client.getObject('sim')

	try:

		## Start the simulation using ZeroMQ RemoteAPI
		try:
			return_code = sim.startSimulation()
			if sim.getSimulationState() != sim.simulation_stopped:
				print('\nSimulation started correctly in CoppeliaSim.')
			else:
				print('\nSimulation could not be started correctly in CoppeliaSim.')
				sys.exit()

		except Exception:
			print('\n[ERROR] Simulation could not be started !!')
			traceback.print_exc(file=sys.stdout)
			sys.exit()

		## Runs the robot navigation logic written by participants
		try:
			control_logic(sim)
			time.sleep(5)

		except Exception:
			print('\n[ERROR] Your control_logic function throwed an Exception, kindly debug your code!')
			print('Stop the CoppeliaSim simulation manually if required.\n')
			traceback.print_exc(file=sys.stdout)
			print()
			sys.exit()

		
		## Stop the simulation using ZeroMQ RemoteAPI
		try:
			return_code = sim.stopSimulation()
			time.sleep(0.5)
			if sim.getSimulationState() == sim.simulation_stopped:
				print('\nSimulation stopped correctly in CoppeliaSim.')
			else:
				print('\nSimulation could not be stopped correctly in CoppeliaSim.')
				sys.exit()

		except Exception:
			print('\n[ERROR] Simulation could not be stopped !!')
			traceback.print_exc(file=sys.stdout)
			sys.exit()

	except KeyboardInterrupt:
		## Stop the simulation using ZeroMQ RemoteAPI
		return_code = sim.stopSimulation()
		time.sleep(0.5)
		if sim.getSimulationState() == sim.simulation_stopped:
			print('\nSimulation interrupted by user in CoppeliaSim.')
		else:
			print('\nSimulation could not be interrupted. Stop the simulation manually .')
			sys.exit()