import numpy as np
import rospy
import time 

from sensor import Sensor

class Master:
    
    def __init__(self):

        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.acc = np.zeros(3)
        self.att = np.zeros(3)
        self.angvel = np.zeros(3)
        self.andacc = np.zeros(3)
        self.tvcang = np.zeros(2)
        self.time = 0


    def main(self):

        s = Sensor()

        while True:

            print("running...",end="\r")
            
            time.sleep(0.01)








if __name__ == "__main__":

    M = Master()
    M.main()