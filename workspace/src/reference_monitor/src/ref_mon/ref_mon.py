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
from reference_monitor.msg import Residual
from reference_monitor.msg import Attack_flag
from reference_monitor.msg import MITM
class Cusum:
    def __init__(self):
        self.cusum = [0, 0]
        self.attack = False
        self.b_pos = 0.03
        self.b_angle = 0.08
        self.threshold_pos = 0.5
        self.threshold_angle = 0.5

    def calculate_residual(self, residual):
        if self.cusum[0] + residual[0] - self.b_pos < 0 or self.cusum[0] + residual[0] - self.b_pos > self.threshold_pos:
            self.cusum[0] = 0
        else:
            self.cusum[0] = self.cusum[0] + residual[0] - self.b_pos

        if self.cusum[1] + residual[1] - self.b_angle < 0 or  self.cusum[1] + residual[1] - self.b_angle > self.threshold_angle:
            self.cusum[1] = 0
        else:
            self.cusum[1] = self.cusum[1] + residual[1] - self.b_angle
        return

    def get_attack_flag(self):
        if self.cusum[0] > threshold_pos or self.cusum[1] > threshold_angle:
            self.attack = True
        return self.attack

    def get_cusum(self):
        return self.cusum

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
	f = open("saturation.txt", "w+")
	f.write("time y_Position y_angle x_hat_pos x_hat_angle est_vel steering residual_pos residual_angle cusum_pos cusum_angle\n")
	rospy.init_node("ref_mon", anonymous=True)
	line_fetcher = LineDataFetcher()
	vel_est_fetcher = VelEstFetcher()
	car_vel_est = CarVelEst()
	str_sub = StrSub()
	cusum_algorithm = Cusum()
	pub = rospy.Publisher('/rm/residual', Residual, queue_size=1)
	pub_attack = rospy.Publisher('/attack_flag', Attack_flag, queue_size=1)
	pub_cusum = rospy.Publisher('/cusum',MITM, queue_size=1)
	rate = rospy.Rate(30)
	dt = 0.033333
	
	x_hat = np.matrix([[0],
		           [0]])		
		
	Q = np.matrix([[2, 0],
		       [ 0, 2]])

	R = np.matrix([[.02, 0],
		       [ 0, .02]])

	Hk = np.matrix([[1, 0],
			[0, 1]])

	P = np.matrix([[0, 0],
		       [0, 0]])
	
	while not rospy.is_shutdown():
		startTime = Decimal('{0:f}'.format(time.time()*1000000))
		line_data = line_fetcher.get_linedata()
		vel_est_data = vel_est_fetcher.get_fetched_data
		vel_est_encoders = vel_est_fetcher.get_fetched_data()
		str_cmd = str_sub.get_str() #get steering command
		
		if line_data == []: #or vel_est_data ==[] or vel_est_encoders ==[]:
			continue

		line_pos_x   = line_data.line_pos_x
		angle_radian = line_data.angle_radian
		car_vel_est.update_data(vel_est_encoders)
		car_vel = car_vel_est.calc_vel()
		
		#print("Angle Rotation: ", angle_radian)
		#print("Line Position: ", line_pos_x)
		#print("Car Vel: ", car_vel)	
		#print("Steering Angle: ", str_cmd)
		#print()
		
		
		
		
		


	
		     
		gain = 0.2
		steering = gain * (str_cmd * (np.pi/180.0) - np.pi/2)               #data: 132.012803853 convert to radiasn
		velocity = car_vel                   				    #/vel_est FL: 0.668301343918
		pos = line_pos_x                     			           #line_pos_x: -0.106172680855
		angle = gain * (angle_radian - np.pi/2)         			#angle_radian: 2.42298460007

		lr = 0.15
		lf = 0.17
		                                   

		# needed matrices
		u = np.matrix([steering, velocity])

		y = np.matrix([[pos],
			       [angle]])


		beta = np.arctan(lr / (lr + lf) * np.tan(u.item((0, 0))))
		dpos = u.item((0, 1)) * np.sin(x_hat.item((1, 0)) + beta)
		dangle = (u.item((0, 1)) / lr) * np.sin(beta)
		dx = np.matrix([[dpos],
			   [dangle]])

		Fk = np.matrix([[0, velocity * np.cos(x_hat.item((1, 0)) + beta)],
			    [0, 0]])

		K = np.matmul((np.matmul(P, Hk.transpose())), np.linalg.inv(R))

		x_hat = np.add(x_hat, np.multiply(dt, np.add(dx, np.matmul(K, np.subtract(y, np.matmul(Hk, x_hat))))))

		Pdot = np.add(np.add(np.matmul(Fk, P), np.matmul(P, Fk.transpose())), np.subtract(Q, np.matmul(np.matmul(K, Hk), P)))
		P = np.add(P, np.multiply(dt, Pdot))
		
		residual = np.subtract(x_hat, y)
		
		#print("residual: ",residual)

		endTime = Decimal('{0:f}'.format(time.time()*1000000))
		#f.write("%s\n" % str(endTime-startTime))
		#f.write("%s\n" % str(endTime-startTime))

		
		
		res = [abs(residual[0]), abs(residual[1])]
		
		cusum_algorithm.calculate_residual(res)
		output = cusum_algorithm.get_cusum()
		#pub.publish(output[0],output[1])
		pub.publish(abs(residual[0]), abs(residual[1]))
		pub_attack.publish(False)
		pub_cusum.publish(output[0], output[1], x_hat[0], x_hat[1])
		
		
		f.write("%f %f %f %f %f %f %f %f %f %f %f\n" %(endTime, y[0], y[1], x_hat[0], x_hat[1], velocity, steering, abs(residual[0]),abs(residual[1]), output[0], output[1]))
		#print (endTime-startTime)
		rate.sleep()


if __name__=='__main__':
	main()



          

