import numpy as np
import asyncio
import rospy

from std_msgs.msg import Float32MultiArray


class Sensor:

    def __init__(self):
        rospy.init_node('sensor_receiver')
        rospy.Subscriber('sensor_data',Float32MultiArray,self.status_updater)
        self.pos = 0
        self.vel = 0
        self.att = 0
        self.angvel = 0
        self.angacc = 0

    def status_updater(self,sensor_datas):
        print("received")
        print(sensor_datas.data)

