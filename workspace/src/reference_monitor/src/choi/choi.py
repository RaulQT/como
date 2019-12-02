#!/usr/bin/env python
import time
import rospy
import roslib
import numpy as np
from decimal import Decimal
from std_msgs.msg import String
from como_image_processing.msg import LineData
from std_msgs.msg import Float64
from barc.msg import Encoder

class StrSub:
	'''
	Subscribes to the steering angle topic
	'''
	def __init__(self):
		self.str_cmd = 0.0
		self.str_sub = rospy.Subscriber("/ecu/line_follower/servo", Float64, self.callback, queue_size =1)
		
	def callback(self, data):
		self.str_cmd = data.data
		
	def get_str(self):
		return self.str_cmd

class VelEstFetcher:
	def __init__(self):
		self.data_sub = rospy.Subscriber("/vel_est", Encoder, self.data_callback, queue_size =1)
		self.encoder_vel_data = []
		
	def data_callback(self, data):
		self.encoder_vel_data = data
		return
		
	def get_fetched_data(self):
		return self.encoder_vel_data
		
class CarVelEst:
	'''
	Estimates the velocity of the car using the vel_est data
	'''
	def __init__(self):
		self.data_curr = []
		self.data_prev = []
		self.car_speed_est = []
	
	def update_data(self, data):
		if data == []:
			return
		self.data_prev = self.data_curr
		self.data_curr = data
		return
		
	def calc_vel (self):
		if ((self.data_curr == []) | (self.data_prev == [])):
			return 0.0
		vel_sum = 0.0
		num_elts = 0.0
		if self.data_curr.FL >= 0.00001:
			vel_sum += self.data_curr.FL
			num_elts += 1.
		if self.data_curr.FR >= 0.00001:
			vel_sum += self.data_curr.FR
			num_elts += 1.
		if self.data_curr.BL >= 0.00001:
			vel_sum += self.data_curr.BL
			num_elts += 1.
		if self.data_curr.BR >= 0.00001:
			vel_sum += self.data_curr.BR
			num_elts += 1.
		if self.data_prev.FL >= 0.00001:
			vel_sum += self.data_prev.FL
			num_elts += 1.
		if self.data_prev.FR >= 0.00001:
			vel_sum += self.data_prev.FR
			num_elts += 1.
		if self.data_prev.BL >= 0.00001:
			vel_sum += self.data_prev.BL
			num_elts += 1.
		if self.data_prev.BR >= 0.00001:
			vel_sum += self.data_prev.BR
			num_elts += 1.
		if num_elts > 0:
			vel = vel_sum/num_elts
			return vel
		else:
			return 0.0
	
	 

class LineDataFetcher:
	def __init__(self):
		self.data_sub = rospy.Subscriber("/line_data", LineData, self.data_callback, queue_size = 1)
		self.line_data = []
	def data_callback(self, linedata):
		self.line_data = linedata
		return
		
	def get_linedata(self):
		return self.line_data
		
def main():
	f = open("choi_09.txt", "w+")
	f.write("time error xlin_pos xlin_angle sensor_pos sensor_angle str_cmd est_vel\n")
	rospy.init_node("choi", anonymous=True)
	line_fetcher = LineDataFetcher()
	vel_est_fetcher = VelEstFetcher()
	car_vel_est = CarVelEst()
	str_sub = StrSub()

	rate = rospy.Rate(30)
	
	counter_window = 0
	t_window = Decimal('{0:f}'.format(time.time()))
	monitor_window = 3
	taulin = 0.06
	A = np.matrix([[-23.2533, -36.5024],
               [4.0271, 6.0330]])

	B = np.matrix([[-1.9326, -1.439],
               [0.3231, 0.2243]])

	L = np.matrix([[0.0034, -0.0006],
               [-0.0006, 0.0001]])

	xlin = np.matrix([[0],
                  [0]])

	sumlin = np.matrix([[0],
                    [0]])
	
	
	
	dt = 0.033333
	

	
	while not rospy.is_shutdown():
		counter_window = counter_window + 1
		startTime = Decimal('{0:f}'.format(time.time()*1000000))
		line_data = line_fetcher.get_linedata()
		vel_est_data = vel_est_fetcher.get_fetched_data
		vel_est_encoders = vel_est_fetcher.get_fetched_data()
		str_cmd = str_sub.get_str()*np.pi/180 #get steering command
		
		if line_data == []: #or vel_est_data ==[] or vel_est_encoders ==[]:
			continue

		line_pos_x   = line_data.line_pos_x
		angle_radian = line_data.angle_radian
		car_vel_est.update_data(vel_est_encoders)
		car_vel = car_vel_est.calc_vel()
		
		#line data position and angle
		sensors = np.matrix([[line_pos_x],
                     		     [angle_radian]])

		U =  np.matrix([[str_cmd],
                		[car_vel]])

		dxlin = np.add(np.add(np.matmul(A,xlin),np.matmul(B,U)),np.matmul(L,np.subtract(sensors,xlin)))
    		xlin = np.add(xlin, dt*dxlin)

	    	sumlin = np.add(sumlin, np.square(np.subtract(sensors,xlin)))

	    	error = sumlin/counter_window

	    	enlapsedTime = Decimal('{0:f}'.format(time.time()))


	    	if enlapsedTime - t_window > monitor_window:
			#print '3 seconds have passed'
			t_window = enlapsedTime
			counter_window = 0
			sumlin = 0


	    	if error.item(0, 0) > taulin:
			t_window = enlapsedTime
			sumlin = 0
			error[0][0]=0
			error[1][0]=0
			counter_window = 0
			
		
		
		f.write("%f %f %f %f %f %f %f %F\n" %(enlapsedTime,error.item(0, 0),xlin.item(0,0), xlin.item(1,0),line_pos_x,angle_radian,str_cmd,car_vel  ))
		
		

		rate.sleep()
		
		
		

if __name__=='__main__':
	main()



          

