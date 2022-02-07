import numpy as np
import matplotlib.pyplot as plt

from transform import Transformer

class Visualizer:
    
    def __init__(self,sim):
        self.L = 10
        self.tf = Transformer()

    def run(self,pos,att):
        
        center_pos = pos[1:,:]
        top_pos = np.zeros((len(pos),3))
        bot_pos = np.zeros((len(pos),3))

        for i in range(len(pos)):
            T = self.tf.body_to_earth(att[i,:])
            top_pos[i] = pos[i,:] + T @ np.array([0.5 * self.L , 0 , 0])
            bot_pos[i] = pos[i,:] + T @ np.array([-0.5 * self.L, 0 , 0])
            
        plt.subplot(1,1,1,projection='3d')
        plt.gca().set_xlim(-500,500)
        plt.gca().set_ylim(-500,500)
        plt.gca().set_zlim(0,1000)
        plt.quiver(0,0,0,1,0,0,length=2,arrow_length_ratio=0,color='red')
        plt.quiver(0,0,0,0,1,0,length=2,arrow_length_ratio=0,color='green')
        plt.quiver(0,0,0,0,0,1,length=2,arrow_length_ratio=0,color='blue')


        for i in range(len(pos)):
            plt.cla()
            plt.gca().set_xlim(0,800)
            plt.gca().set_ylim(0,800)
            plt.gca().set_zlim(0,800)
            
            plt.quiver(bot_pos[i,0],bot_pos[i,1],bot_pos[i,2],
                       top_pos[i,0]-bot_pos[i,0],
                       top_pos[i,1]-bot_pos[i,1],
                       top_pos[i,2]-bot_pos[i,2],
                       length=5,linewidth=3,arrow_length_ratio=0,color='black')

            plt.plot(top_pos[:i,0],top_pos[:i,1],top_pos[:i,2],'r-')
            plt.plot(center_pos[:i,0],center_pos[:i,1],center_pos[:i,2],'g-')
            plt.plot(bot_pos[:i,0],bot_pos[:i,1],bot_pos[:i,2],'b-')

            plt.pause(0.01)
        
        plt.show()