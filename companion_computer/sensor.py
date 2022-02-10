import numpy as np
import asyncio
import rospy

from std_msgs.msg import Float32MultiArray


class Sensor:

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

    def status_updater(self,sensor_datas):
        
        self.pos = sensor_datas.data[:3]
        self.vel = sensor_datas.data[3:6]
        self.att = sensor_datas.data[6:9]
        self.angvel = sensor_datas.data[9:12]
        self.angacc = sensor_datas.data[12:15]
        self.is_burning = sensor_datas.data[15]

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":

    s = Sensor()
    s.run()