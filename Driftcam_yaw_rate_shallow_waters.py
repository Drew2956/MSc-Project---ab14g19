# version 1.0
# Author: Subhra subhra_k.das@soton.ac.uk
# Date created: 27/08/2019
# Description: code calculates the steady state yaw rate that the driftcam will experience
# during shallow water applications due to higher torque from thrusters running all the time


import math
import numpy as np
import matplotlib.pyplot as plt



# driftcam yaw rate calculations
#--------------------------------------------------------------------------------
simulation_time = 0.25
ts = 0.01
density = 1024 #kg/m3

moi_system = 0.5598 # kg-m2
torque_thruster = 0.169 # N-m

# obtained from spreadsheet
area_float = 0.2356 # m2
area_battery = 0.1159 # m2
area_camera = 0.0659 # m2
area_thruster = 0.01 # m2

Cd = 1.2 #drag coefficient

yaw_rate = 0 # rad/sec

i = 0

yaw_rate_list = []

while i < simulation_time:
	yaw_rate_list.append (yaw_rate * 180 / math.pi)
	
	drag_torque_float = yaw_rate * yaw_rate * area_float * density * Cd * 0.5
	drag_torque_battery = yaw_rate * yaw_rate * area_battery * density * Cd * 0.5
	drag_torque_camera = yaw_rate * yaw_rate * area_camera * density * Cd * 0.5
	drag_torque_thruster = yaw_rate * yaw_rate * area_thruster * density * Cd * 0.5

	net_drag = drag_torque_float + drag_torque_battery + drag_torque_camera + drag_torque_thruster
	
	net_torque = torque_thruster - net_drag

	yaw_accl = net_torque / moi_system

	yaw_rate = yaw_rate + yaw_accl * ts # difference equation

	i = i + ts

print('Steady state yaw rate is: ', yaw_rate_list[len(yaw_rate_list)-1])
print('deg/sec')
plt.plot(yaw_rate_list)

plt.show()









