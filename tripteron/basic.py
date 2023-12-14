import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from util.GeneratorNode           import GeneratorNode
from util.TransformHelpers        import *
from util.TrajectoryUtils         import *
from tripteron.KinematicChain     import KinematicChain
from tripteron.Kinematic          import kinematic




class Trajectory():
    def __init__(self, node):
        # TODO: Rename the tips in the URDF to a better name
        self.FRchain = KinematicChain(node, 'base_link', 'railAttach_one', self.jointnames("frontright"))
        self.FLchain = KinematicChain(node, 'base_link', 'railAttach_two', self.jointnames("frontleft"))
        self.BRchain = KinematicChain(node, 'base_link', 'railAttach_three', self.jointnames("backright"))
        self.BLchain = KinematicChain(node, 'base_link', 'railAttach_four', self.jointnames("backleft"))
        # self.kin = kinematic()
        self.platformP = pxyz(0.4,0.4,0.15)
        
        self.platformP0 = self.platformP

        # self.q = np.radians(np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]).reshape((-1,1)))
        self.q = np.radians(np.array([-pi/2, -pi/4, 0, -pi/2, -3*pi/4, 0, pi/3, -2*pi/3, 0, -pi/3, 2*pi/3, 0]).reshape((-1,1)))

    # Declare the joint names.
    def jointnames(self, arm):
        # ask gunter about wether the last joint should be fixed
        if arm == 'frontright':
            return ['theta1', 'theta2', 'theta3'] # , 'theta3'
        elif arm == 'frontleft':
            return ['theta4', 'theta5', 'theta6'] # , 'theta6'
        elif arm == 'backright':
            return ['theta7', 'theta8', 'theta9'] # , 'theta9'
        elif arm == 'backleft':
            return ['theta10', 'theta11', 'theta12']# , 'theta12'
        elif arm == "all":
            return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 
                'theta7', 'theta8', 'theta9', 'theta10', 'theta11', 'theta12']

    def bodyTrajectory(self, t):
        Pplatform = self.platformP 
        Rplatform = Rotz(0)
        Tplatform = T_from_Rp(Rplatform, Pplatform)

        return Transform_from_T(Tplatform)
       
    def debug (self):
        # self.kin.legOne(self.pd_FR)
        return self.pd_FR
        
    def calc_L(self, platform):
        # P_xyz = platform xyz
        T = np.array([[1, 0 , -1],
                     [1, -1 , 1],
                     [1, 1 , 1]])
        
        return T @ platform
    def Jac(self, theta1, theta2):
        
        l = 0.1524
        #print(theta1, theta2)
        J = np.array([[-l*sin(theta1)/sqrt(2), -l*sin(theta2)/sqrt(2)],
                     [(l**2 * sin(theta1+theta2))/(2*sqrt(l**2-l**2 * cos(pi - (theta1+theta2)))), (l**2 * sin(theta1+theta2))/(2*sqrt(l**2-l**2 * cos(pi - (theta1+theta2))))]])
        return J

    def T_mat(self):
        T_from_Rp(R_from_RPY(-pi/2, -pi/4,0), pxyz(0, 0, 0.1524))
        return None
    

    def evaluate(self, t, dt):
        l1, l2, l3 = self.calc_L(self.platformP).flatten().tolist()
        l2 -= 0.55
        l3 -=0.1
        x,y,z = self.platformP.flatten().tolist()
        #same target position
        legTargetOne = pxyz(-(y - l1), 0.550,0)
        legTargetTwo = pxyz(-(y - l1),-0.1, 0)

        legTargetThree = pxyz( -l2 + y,0.1, 0)
        legTargetFour = pxyz( -l3+  y,-0.1, 0)
        
        legTargets = [legTargetOne, legTargetTwo, legTargetThree, legTargetFour]
        
        # Compute the joints.
        # t1 = (t) % 2.0
        if t < 10:
            pd = self.platformP0 + pxyz(0.1*sin(t*5), 0.25*sin(t*7.5)*exp(-t/3), t/50)
            vd = pxyz( 0.5*cos(5*t),exp(-t) * (7.5*cos(7.5*t) - sin(7.5*t)), 1)
        else:
            pd = self.platformP0 + pxyz(sin(10)*exp(-10), 0, 10/50)
            vd = pxyz(0, 0, 0)
        T = np.matrix([
           [0, 1, -1],
           [-1, 1, 1],
           [1, 1, 1]
        ])
        
        qupdate = []
        qupdatedot = []
        #print(self.q)
        for i, target in enumerate(legTargets):
            q1, q2 = self.q.flatten().tolist()[3*i: 3*(i) + 2]
            J = self.Jac(q1, q2)
            q12dot = np.linalg.inv(J) @ vd[:2]
            q12 = np.array([q1, q2]).reshape(-1, 1) +  q12dot * dt
            q3 = target.flatten()[0] # slider pos solved with the T matrix
            q3dot = vd.flatten()[0] # estimating y vel as qslider dot
            
            q12 = q12.flatten()
            #qupdate.append(q12[0])
            #qupdate.append(q12[1])
            qupdate.append(q1)
            qupdate.append(q2)
            qupdate.append(q3)
            
            q12dot = q12dot.flatten()
            #qupdatedot.append(q12dot[0])
            #qupdatedot.append(q12dot[1])
            qupdatedot.append(0)
            qupdatedot.append(0)
            qupdatedot.append(q3dot)
        #print(qupdate)
            
        
        self.q = np.array(qupdate).reshape((-1, 1))
        qdot = np.array(qupdatedot).reshape((-1, 1))
        
        self.platformP = pd


        return (self.q.flatten().tolist(), qdot.flatten().tolist())


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    # generator = GeneratorNode('generator', 100, Trajectory)
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

