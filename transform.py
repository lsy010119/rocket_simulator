import numpy as np 
import matplotlib.pyplot as plt


class Transformer():
    def __init__(self):
        pass

    def body_to_earth(self,attitude): 
        
        psi = attitude[0]
        theta = attitude[1]
        phi = attitude[2]
        
        R = np.array([[1,      0,           0     ],
                      [0, np.cos(psi), -np.sin(psi)],
                      [0, np.sin(psi), np.cos(psi)]])
        
        P = np.array([[np.cos(theta), 0, np.sin(theta)],
                      [0,             1,       0      ],
                      [-np.sin(theta),0, np.cos(theta)]])
        
        Y = np.array([[np.cos(phi),  -np.sin(phi), 0],
                      [np.sin(phi),   np.cos(phi), 0],
                      [0,            0,           1]])


        transformation_mtx = Y@P@R
        
        return transformation_mtx




'''
if __name__ == "__main__":
    
    t = Transformer()

    plt.subplot(1,1,1, projection ="3d") 
    plt.gca().set_xlim(-15,15)
    plt.gca().set_ylim(-15,15)
    plt.gca().set_zlim(-10,10)
    
    plt.quiver(0,0,0,1,0,0,length=5,color='black')
    plt.quiver(0,0,0,0,1,0,length=5,color='black')
    plt.quiver(0,0,0,0,0,1,length=5,color='black')

    psi = 0
    theta = 30
    phi = 30

    for i in range(0,40):
        
        w_r = 0
        w_p = 9
        w_y = 0

        psi = psi + w_r * i
        theta = theta + w_p * i
        phi = phi + w_y * i

        T = t.body_to_earth([psi,theta,phi])

        w = T@np.array([w_r,w_p,w_y])

        xt_body = T@np.array([1,0,0])
        xb_body = T@np.array([-1,0,0])
        y_body = T@np.array([0,1,0])
        z_body = T@np.array([0,0,1])

        # plt.cla()
        plt.gca().set_xlim(-15,15)
        plt.gca().set_ylim(-15,15)
        plt.gca().set_zlim(-10,10)

        plt.quiver(0,0,0,1,0,0,length=5,color='red')
        plt.quiver(0,0,0,0,1,0,length=5,color='blue')
        plt.quiver(0,0,0,0,0,1,length=5,color='green')
        
        plt.quiver(0,0,0,xt_body[0],xt_body[1],xt_body[2],length=2.5,linewidth=2,arrow_length_ratio=0,color='black',)
        plt.quiver(0,0,0,xb_body[0],xb_body[1],xb_body[2],length=2.5,linewidth=2,arrow_length_ratio=0,color='black',)
        plt.quiver(0,0,0,w[0],w[1],w[2],length=1,color='blue')
        # plt.quiver(0,0,0,y_body[0],y_body[1],y_body[2],length=5,color='blue')
        # plt.quiver(0,0,0,z_body[0],z_body[1],z_body[2],length=5,color='green')



        plt.pause(0.1)

plt.show()
'''