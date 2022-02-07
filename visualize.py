from matplotlib.axes import Axes
import numpy as np
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import axes3d, Axes3D
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
            

        fig = plt.figure()

        ax = Axes3D(fig)

        ax.set_xlim(-500,500)
        ax.set_ylim(-500,500)
        ax.set_zlim(0,1000)
        ax.quiver(0,0,0,1,0,0,length=2,arrow_length_ratio=0,color='red')
        ax.quiver(0,0,0,0,1,0,length=2,arrow_length_ratio=0,color='green')
        ax.quiver(0,0,0,0,0,1,length=2,arrow_length_ratio=0,color='blue')


        for i in range(len(pos)):
            ax.cla()
            ax.set_xlim(0,800)
            ax.set_ylim(0,800)
            ax.set_zlim(0,800)
            
            ax.quiver(bot_pos[i,0],bot_pos[i,1],bot_pos[i,2],
                       top_pos[i,0]-bot_pos[i,0],
                       top_pos[i,1]-bot_pos[i,1],
                       top_pos[i,2]-bot_pos[i,2],
                       length=5,linewidth=3,arrow_length_ratio=0,color='black')

            ax.plot(top_pos[:i,0],top_pos[:i,1],top_pos[:i,2],'r-')
            ax.plot(center_pos[:i,0],center_pos[:i,1],center_pos[:i,2],'g-')
            ax.plot(bot_pos[:i,0],bot_pos[:i,1],bot_pos[:i,2],'b-')

            plt.pause(0.01)
        
        plt.show()