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
        self.kin = kinematic()
        self.platformP0 = pxyz(0,0,0)
        self.target = pxyz(0.4,0.4, 0.15) 

        self.q_FR = np.zeros((3,1))
        (self.pd_FR, self.Rd_FR, _, _) = self.FRchain.fkin(self.q_FR)
        self.p0_FR = self.pd_FR

        self.q_FL = np.zeros((3,1))
        (self.pd_FL, self.Rd_FL, _, _) = self.FLchain.fkin(self.q_FL)

        self.q_BR = np.zeros((3,1))
        (self.pd_BR, self.Rd_BR, _, _) = self.BRchain.fkin(self.q_BR)

        self.q_BL = np.zeros((3,1))
        (self.pd_BL, self.Rd_BL, _, _) = self.BLchain.fkin(self.q_BL)

        self.vd = np.zeros((3,1))
        self.q = np.zeros((12,1))
        self.qdot= np.zeros((12,1))
        self.lam = 20.0
        # self.node = node
        # self.q0 = np.radians(np.array([0, 0, 0, 0, 0, 0]).reshape((-1,1)))

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
            return ['theta1', 'theta2', 'theta3','theta4', 'theta5', 'theta6', 
                'theta7', 'theta8', 'theta9', 'theta10', 'theta11', 'theta12']

    def bodyTrajectory(self, t):
        # Compute position/orientation of the pelvis (w.r.t. world).
        # go in circle relative to x y plane
        # Pplatform = pxyz(0.4+0.05*sin(t),0.4+ 0.05*cos(t), 0.15) 
        Pplatform = self.platformP0 
        Rplatform = Rotz(0)
        Tplatform = T_from_Rp(Rplatform, Pplatform)

        return Transform_from_T(Tplatform)




        # vr = vd


        return None
    # def get_pd (self):
    #     (p1, _, _, _) = self.FRchain.fkin(np.zeros((3,1)))
    #     (p2, _, _, _) = self.FLchain.fkin(np.zeros((3,1)))
    #     (p3, _, _, _) = self.BRchain.fkin(np.zeros((3,1)))
    #     (p4, _, _, _) = self.BLchain.fkin(np.zeros((3,1)))
    #     return np.vstack((p1+self.offset,p2+self.offset,p3+self.offset, p4+self.offset))
        
    # def joints (self):
    #     self.pd_FR
    #     P_legone = pxyz(self.p0_FR[0][0],self.p0_FR[1][0], self.p0_FR[2][0])
    #     R_legone = Rotz(0)
    #     T_legone = T_from_Rp(R_legone, P_legone)
    #     return Transform_from_T(T_legone) 
       
    def debug (self):
        # FLchainTest = KinematicChain(node, 'world', 'railAttach_two', self.jointnames("frontleft"))
        # (ptip, _, _, _) = self.FLchain.fkin(self.q[:3, 0])
        # (p1, _, _, _) = self.FRchain.fkin(np.array([-np.pi/2, -3*np.pi/4, -3*np.pi/4, ]).reshape(3,1))
        # (p1, _, _, _) = self.FRchain.fkin(np.zeros((3,1)))
        # (p2, _, _, _) = self.FLchain.fkin(np.zeros((3,1)))
        # (p3, _, _, _) = self.BRchain.fkin(np.zeros((3,1)))
        # (p4, _, _, _) = self.BLchain.fkin(np.zeros((3,1)))
        # ptip = np.vstack((p1+self.offset,p2,p3, p4))
        return self.kin.legOne(self.pd_FR)       

    def newtonRaphson(self, pd):
        # Collect the distance to goal and change in q every step!
        # xdistance = []
        # qstepsize = []
        max_iter = 20

        # Set the initial joint value guess.
        # q = np.array([0.0, np.pi/2, -np.pi/2]).reshape(3,1)
        for i in range(max_iter):
            (p, R, Jv, Jw) = self.FRchain.fkin(self.q_FR)
            e = pe(p,pd)
            dist = np.linalg.norm(e)
            # xdistance.append(dist)
            if dist < 1e-12:
                break 
    
            joint_step = self.q_FR + np.dot(np.linalg.pinv(Jv), e)
            # qstepsize.append(np.linalg.norm(joint_step - q))
            self.q_FR = joint_step

            # Jinv = np.linalg.pinv(Jv)

            # qdot = Jinv @ (v + self.lam * e)
            # self.q += qdot * dt

    def evaluate(self, t, dt):
        
        # Compute the joints.
        # pd, pv = goto(t, 2, self.p0, self.target)
        # lam = self.lam
        # qlast = self.q_FR
        # xdlast = self.xdFR

        # (p, R, Jv, Jw) = self.FRchain.fkin(self.q_FR)

        # x_error = xp - x_current
        # J = Jac(self.q) 
        # J_inv = np.linalg.pinv(J)
        # qdot = J_inv @ (Jv + (lam * x_error))
        # # qdot = np.linalg.solve(J, v_desired + lambda_value * x_error)
        # self.q += qdot * dt
        # self.newtonRaphson(pd)
        # current postion
        
        # dist

    
        return (self.q.flatten().tolist(), self.qdot.flatten().tolist())

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

