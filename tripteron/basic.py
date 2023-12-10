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
        self.FRchain = KinematicChain(node, 'platform', 'railAttach_one', self.jointnames("frontright"))
        self.FLchain = KinematicChain(node, 'platform', 'railAttach_two', self.jointnames("frontleft"))
        self.BRchain = KinematicChain(node, 'platform', 'railAttach_three', self.jointnames("backright"))
        self.BLchain = KinematicChain(node, 'platform', 'railAttach_four', self.jointnames("backleft"))
        # self.kin = kinematic()
        self.platformP = pxyz(0.4,0.4,0.15)
        # self.target = pxyz(0.4,0.0, 0.15) 
        d0 = self.calc_L(self.platformP)
        print(d0)
        # self.q = np.radians(np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]).reshape((-1,1)))
        self.q = np.radians(np.array([0, 0, 0, 0, 0, 0, 0, 0]).reshape((-1,1)))
        
        
        (self.pd_FR, self.Rd_FR, _, _) = self.FRchain.fkin(self.q[0:2])

        # (self.pd_FL, self.Rd_FL, _, _) = self.FLchain.fkin(self.q_FL)
        # (self.pd_BR, self.Rd_BR, _, _) = self.BRchain.fkin(self.q_BR)
        # (self.pd_BL, self.Rd_BL, _, _) = self.BLchain.fkin(self.q_BL)
        self.lam = 20.0
        self.p0_FR = self.pd_FR
        # pxyz(self.platformP[0][0]-d0[0][0], 0.25, -0.127)
        self.p0_FL = pxyz(d0[0][0], -0.1, 0)

        self.p0_BR = pxyz(d0[1][0], 0.1, 0)
        self.p0_BL = pxyz(d0[2][0], -0.1, 0)
        # self.pd_FR = self.p0_FR
        # self.node = node
        # self.q0 = np.radians(np.array([0, 0, 0, 0, 0, 0]).reshape((-1,1)))

    # Declare the joint names.
    def jointnames(self, arm):
        # ask gunter about wether the last joint should be fixed
        if arm == 'frontright':
            return ['theta1', 'theta2'] # , 'theta3'
        elif arm == 'frontleft':
            return ['theta4', 'theta5'] # , 'theta6'
        elif arm == 'backright':
            return ['theta7', 'theta8'] # , 'theta9'
        elif arm == 'backleft':
            return ['theta10', 'theta11']# , 'theta12'
        elif arm == "all":
            return ['theta1', 'theta2','theta4', 'theta5', 
                'theta7', 'theta8', 'theta10', 'theta11']
        # if arm == 'frontright':
        #     return ['theta1', 'theta2', 'theta3'] # , 'theta3'
        # elif arm == 'frontleft':
        #     return ['theta4', 'theta5', 'theta6'] # , 'theta6'
        # elif arm == 'backright':
        #     return ['theta7', 'theta8', 'theta9'] # , 'theta9'
        # elif arm == 'backleft':
        #     return ['theta10', 'theta11', 'theta12']# , 'theta12'
        # elif arm == "all":
        #     return ['theta1', 'theta2', 'theta3','theta4', 'theta5', 'theta6', 
        #         'theta7', 'theta8', 'theta9', 'theta10', 'theta11', 'theta12']

    def bodyTrajectory(self, t):
        # Compute position/orientation of the pelvis (w.r.t. world).
        # go in circle relative to x y plane
        # Pplatform = pxyz(0.4+0.05*sin(t),0.4+ 0.05*cos(t), 0.15) 
        Pplatform = self.platformP 
        Rplatform = Rotz(0)
        Tplatform = T_from_Rp(Rplatform, Pplatform)

        return Transform_from_T(Tplatform)
       
    def debug (self):
        # self.kin.legOne(self.pd_FR)
        return self.pd_FR
        
    def calc_L(self, platform):
        # P_xyz = platform xyz
        T = np.array([[0, 1 , -1],
                     [-1, 1 , 1],
                     [1, 1 , 1]])
        
        return T @ platform
    def Jac(self, q):
        theta1 = q[0][0]
        theta2 = q[1][0]
        l = 0.1524
        J = np.array([[-l*sin(theta1)/sqrt(2), -l*sin(theta2)/sqrt(2)],
                     [(l**2 * sin(theta1+theta2))/(2*sqrt(l**2-l**2 * cos(theta1+theta2))), (l**2 * sin(theta1+theta2))/(2*sqrt(l**2-l**2 * cos(theta1+theta2)))]])
        return J

    def T_mat(self):
        T_from_Rp(R_from_RPY(-pi/2, -pi/4,0), pxyz(0, 0, 0.1524))
        return None
    

    def evaluate(self, t, dt):
        dist = self.calc_L(self.platformP)
        #same target position
        legTargetOne = pxyz(dist[0][0], 0.550, 0)
        legTargetTwo = pxyz(dist[0][0], -0.1, 0)

        legTargetThree = pxyz(dist[1][0], 0.1, 0)
        legTargetFour = pxyz(dist[2][0], -0.1, 0)
        
        # Compute the joints.
        # t1 = (t) % 2.0
        if t < 3:
            pd, vd = goto(t, 3, self.p0_FR, legTargetOne)
            # pP, Vd = goto(t1, 2, )
            # self.platformP = pxyz(0.4,0.4,0.15)
        else:
            return None
            # pd, vd = goto(t1, 2, legTargetOne, self.p0_FR)
        # pd, pv = goto(t, 2, self.p0_FL, legTargetTwo)
        # pd, pv = goto(t, 2, self.p0_BR, legTargetThree)
        # pd, pv = goto(t, 2, self.p0_BL, legTargetFour)
        
        
        # print(R)
        # lam = self.lam
        # qlast = self.q_FR
        # xdlast = self.xdFR

        q_FR = self.q[0:2] 
        pd_FR = self.pd_FR
        (p, R, Jv, Jw) = self.FRchain.fkin(q_FR)
        Pplatform = self.platformP 
        Rplatform = Rotz(pi)
        Tplatform = T_from_Rp(Rplatform, Pplatform)@T_from_Rp(R, p)
        p = p_from_T(Tplatform)
        print(p_from_T(Tplatform))
        # print(self.p0_FR)
        # Compute the inverse kinematics
        vr   = vd + self.lam * (pd_FR - p)
        J = np.vstack((Jv, Jw))
        # print(J)
        qdot = np.linalg.pinv(Jv) @ vr

        # Integrate the joint position.
        q = q_FR + dt * qdot
        # qdot = np.radians(np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]).reshape((-1,1)))
        # Save the joint value and desired position for the next cycle.
        # qdot= np.vstack(qdot, np.array([0, 0, 0, 0, 0, 0, 0, 0]).reshape((-1,1)))
        
        # print(q)
        self.q[0:2]  = q
        self.pd_FR = pd


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

