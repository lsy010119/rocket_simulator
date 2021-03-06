from struct import pack
import numpy as np
import matplotlib.pyplot as plt
import rospy
import time

from mpl_toolkits.mplot3d import axes3d, Axes3D
from std_msgs.msg import Float32MultiArray
from transform import Transformer
from visualize import Visualizer



class Simulator:
    def __init__(self,
                    dt,
                    m0,mf,
                    burn_time,
                    l_cog0,l_cogf,L,D,
                    thrust,
                    k,
                    init_pos,
                    init_vel,
                    init_acc,
                    init_att,
                    init_angvel,
                    init_angacc,
                    init_tvcang,
                    is_burning,
                    sensor_rate):

        # init datas
        self.dt = dt
        self.time = 0
        self.m0,self.mf = m0,mf
        self.t_b = burn_time
        self.l_cog0,self.l_cogf = l_cog0,l_cogf
        self.L,self.D = L,D
        self.thrust = thrust
        self.k = k
        self.pos = init_pos
        self.vel = init_vel
        self.acc = init_acc
        self.att = init_att
        self.angvel = init_angvel
        self.angacc = init_angacc
        self.tvcang = init_tvcang
        self.is_burning = 0
        self.sensor_rate = sensor_rate
        
        # load the modules for transformation and visualizing
        self.tf = Transformer()
        self.vz = Visualizer(self)

        # init publisher node as a sensor
        rospy.init_node("sensor")
        self.sensors = rospy.Publisher("sensor_data",Float32MultiArray, queue_size=1) 
        self.rate = rospy.Rate(self.sensor_rate)

    def mass(self,t):
        if self.time <= self.t_b:
            return self.m0 - ((self.m0 - self.mf)/self.t_b)*t
        else:
            return self.mf


    def cog(self,t):
        if self.time <= self.t_b:
            
            L_cog = self.l_cog0 +  self.time * (self.l_cogf - self.l_cog0)/self.t_b

            L1 = L_cog
            L2 = self.L - L1

            return L1, L2
        
        else:
            
            L1 = self.l_cogf

            L2 = self.L - L1

            return L1, L2


    def run(self):

        # INITIALIZE FIGURE =================================================
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.set_xlim(-500,500)
        ax.set_ylim(-500,500)
        ax.set_zlim(0,1000)
        ax.quiver(0,0,0,1,0,0,length=2,arrow_length_ratio=0,color='red')
        ax.quiver(0,0,0,0,1,0,length=2,arrow_length_ratio=0,color='green')
        ax.quiver(0,0,0,0,0,1,length=2,arrow_length_ratio=0,color='blue')

        odometry_top = self.pos
        odometry_cog = self.pos
        odometry_bot = self.pos


        # Ignite
        self.is_burning = 1
        i = 0

        while not rospy.is_shutdown():

            # update time and iterations
            self.time += self.dt
            i += 1

            T = self.tf.body_to_earth(self.att) # Initinalize Transformation matrix
            
            """ Considering Thrust ====================================================="""

            if self.time > self.t_b: # No thrust after burn out
                self.is_burning = 0
                Thrust_body = np.array([0,0,0])    

            else:  
                # Calculate Thrust vector in Body-Frame
                Thrust_body = np.array([np.cos(self.tvcang[0])*np.cos(self.tvcang[1]),
                                    np.sin(self.tvcang[0]),
                                    -np.cos(self.tvcang[0])*np.sin(self.tvcang[1])])\
                                    * self.thrust[i-1]
                

            # update the Moment of Inertia
            I = self.mass(self.time)*(self.L**2) / 12
            
            # update the Center of Gravity
            L1,L2 = self.cog(self.time)
            
            # update vector cog_to_top / cog_to_bottom
            c2t = T @ np.array([L2,0,0])
            c2b = T @ np.array([-L1,0,0])


            # Force by Thrust
            Thrust_xyz = T @ Thrust_body
            
            # Torque by Thrust
            Torque_by_thrust = np.array([[0,  0,  0],
                                         [0, L1,  0],
                                         [0,  0,-L2]]) @ Thrust_body
            
            """ Considering Drag ====================================================="""

            if np.linalg.norm(self.vel) == 0: # exception for preveting "divided by zero error"

                Drag = np.zeros(3)
                Torque_by_drag = np.zeros(3)    

            else:
                
                # Force by Drag ======================================================

                # effective area is the area of the projection to the plane perpandicular to v vector
                # calculate the angle between v and r
                ang_v_r = np.arccos((self.vel @ c2t)/(np.linalg.norm(self.vel)*(np.linalg.norm(c2t))))

                A_effect = abs(( self.L * self.D ) * np.sin(ang_v_r)) + abs(( np.pi * self.D**2 / 4 ) * np.cos(ang_v_r))

                # calculate the drag applied on the center of gravity
                Drag = - self.k * np.linalg.norm(self.vel) * A_effect * self.vel 

                # Torque by Drag =====================================================

                # Drag is Distributed Force
                # but we assumes that the Drag is applied on the top, center, bottom of the rocket.
                # then, the torque by the Drag can be simplified to
                # the torque by two forces applied on the top and bottom
                '''
                ?? = r X F
            
                F = - k^2 A |v|^2 v
            
                v = ?? X r + v_cog
                '''
                v_top = self.vel + np.cross(self.angvel , c2t)
                v_bot = self.vel + np.cross(self.angvel , c2b)

                drag_top = - self.k* np.linalg.norm(v_top) * (A_effect/3) * v_top 

                drag_bot = - self.k* np.linalg.norm(v_bot) * (A_effect/3) * v_bot 
 
                Torque_by_drag = np.cross(c2t, drag_top) + np.cross(c2b, drag_bot)

            # Updating Dynamic status
            self.acc = ( Thrust_xyz + Drag ) / self.mass(self.time) - np.array([0, 0, 9.8])
            self.pos = self.pos + self.vel * self.dt + 0.5 * self.acc * (self.dt**2)
            self.vel = self.vel + self.acc * self.dt
            
            self.angacc = ( Torque_by_thrust + Torque_by_drag ) / I
            self.att = self.att + self.angvel * self.dt + 0.5 * self.angacc * (self.dt**2)
            self.angvel = self.angvel + self.angacc * self.dt

            if self.pos[2] <= 0: # stop the rocket if it collides with ground
                self.acc = np.zeros(3)
                self.vel = np.zeros(3)
                self.angvel = np.zeros(3)
                self.angacc = np.zeros(3)

            # packing sensor datas to a single 1-dim array
            packed_data = np.hstack((self.pos,self.vel,self.att,self.angvel,self.angacc,self.is_burning,self.time))
            # publishing sensor datas in every interations
            sensor_datas = Float32MultiArray()
            sensor_datas.data = packed_data
            self.sensors.publish(sensor_datas)

  
            # VISUALIZING ================================================
            top_pos = self.pos + T @ np.array([0.5 * 5 , 0 , 0])
            bot_pos = self.pos + T @ np.array([-0.5 * 5, 0 , 0])

            ax.cla()
            ax.set_xlim(0,200)
            ax.set_ylim(0,200)
            ax.set_zlim(0,200)
            
            ax.quiver(bot_pos[0],bot_pos[1],bot_pos[2],
                       top_pos[0]-bot_pos[0],
                       top_pos[1]-bot_pos[1],
                       top_pos[2]-bot_pos[2],
                       length=2,linewidth=2,arrow_length_ratio=0,color='black')
            if self.is_burning == 1:
                ax.quiver(bot_pos[0],bot_pos[1],bot_pos[2],
                        - 5*Thrust_xyz[0]/self.thrust[i-1],
                        - 5*Thrust_xyz[1]/self.thrust[i-1],
                        - 5*Thrust_xyz[2]/self.thrust[i-1],
                       length=3,linewidth=1,arrow_length_ratio=0,color='red')


            odometry_top = np.vstack((odometry_top,top_pos))
            odometry_cog = np.vstack((odometry_cog,self.pos))
            odometry_bot = np.vstack((odometry_bot,bot_pos))


            ax.plot(odometry_top[:,0],odometry_top[:,1],odometry_top[:,2],'r-',linewidth=0.5)
            ax.plot(odometry_cog[:,0],odometry_cog[:,1],odometry_cog[:,2],'g-',linewidth=0.5)
            ax.plot(odometry_bot[:,0],odometry_bot[:,1],odometry_bot[:,2],'b-',linewidth=0.5)

            plt.pause(0.01)
            self.rate.sleep()
        
        plt.show()





if __name__ == "__main__":
    dt = 0.1
    m0 = 6 + 0.150
    mf = 6
    l_cog0 = 1
    l_cogf = 1.05
    L = 1.2
    D = 0.11
    thrust = np.ones(20)*250
    t_b = len(thrust) * dt
    k = 0.15 # rho = 1.125 , Cd = 0.25
    init_pos = np.array([0,0,0])
    init_vel = np.array([0,0,0])
    init_acc = np.array([0,0,0])
    init_att = np.array([0,-np.pi/2,0])
    init_angvel = np.array([0,0,0])
    init_angacc = np.array([0,0,0])
    init_tvcang = np.array([0,0])
    is_burning = 0
    sensor_rate = 30

    S = Simulator(dt,
                  m0,
                  mf,
                  t_b,
                  l_cog0,
                  l_cogf,
                  L,
                  D,
                  thrust,
                  k,
                  init_pos,
                  init_vel,
                  init_acc,
                  init_att,
                  init_angvel,
                  init_angacc,
                  init_tvcang,
                  is_burning,
                  sensor_rate)
    S.run()