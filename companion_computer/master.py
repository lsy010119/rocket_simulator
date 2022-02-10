import numpy as np
import rospy
import time

from std_msgs.msg import Float32MultiArray
from tvc_controller import TVC


class Master:

    def __init__(self):
        rospy.init_node('sensor_receiver')
        rospy.Subscriber('sensor_data',Float32MultiArray,self.status_updater)
        self.commander = rospy.Publisher("tvc_angle", Float32MultiArray,queue_size=1)
        
        self.pos = 0
        self.vel = 0
        self.att = 0
        self.angvel = 0
        self.angacc = 0
        self.is_burning = 0
        self.time = 0

        self.tvc = TVC()


    def status_updater(self,sensor_datas):
        
        self.pos = sensor_datas.data[:3]
        self.vel = sensor_datas.data[3:6]
        self.att = sensor_datas.data[6:9]
        self.angvel = sensor_datas.data[9:12]
        self.angacc = sensor_datas.data[12:15]
        self.is_burning = sensor_datas.data[15]
        self.time = sensor_datas.data[16]
        

    def run(self):

        rate = rospy.Rate(20)
        
        pos_hist = np.array([0,0,0])
        vel_hist = np.array([0,0,0])
        att_hist = np.array([0,0,0])
        angvel_hist = np.array([0,0,0])
        angacc_hist = np.array([0,0,0])
        i = 0
        
        while not rospy.is_shutdown():

            pos_hist = np.vstack((pos_hist,self.pos)) 
            vel_hist = np.vstack((vel_hist,self.vel)) 
            att_hist = np.vstack((att_hist,self.att)) 
            angvel_hist = np.vstack((angvel_hist,self.angvel)) 
            angacc_hist = np.vstack((angacc_hist,self.angacc)) 
            
            if self.is_burning == 1:

                self.control_tvc()


            rate.sleep()


if __name__ == "__main__":

    M = Master()
    M.run()