'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
import numpy as np

from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
                       }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''

        # final rotation matrix R = Rz Ry Rx
        # Rz = [[c,-s,0],[s,c,0],[0,0,1]]
        # Ry = [[c,0,s],[0,1,0],[-s,0,c]]
        # Rx = [[1,0,0],[0,c,-s],[0,s,c]]

        # axis-angle parameters
        # cos(theta) = (tr(R) -1) / 2
        # k = 1/(2sin(theta)) . [[r32 -r23][r13 -r31][r21 -r12]]

        # quaternion parameters
        # eta = cos(theta/2)
        # epsiolon = k.sin(theta/2)

        T = identity(4)
        # YOUR CODE HERE
        sin_angle = np.sin(joint_angle)
        cos_angle = np.cos(joint_angle)

        Rz = [[cos_angle, -sin_angle, 0, 0],
              [sin_angle, cos_angle, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]]

        Ry = [[cos_angle, 0, sin_angle, 0],
              [0, 1, 0, 0],
              [-sin_angle, 0, cos_angle, 0],
              [0, 0, 0, 1]]

        Rx = [[1, 0, 0, 0],
              [0, cos_angle, -sin_angle, 0],
              [0, sin_angle, cos_angle, 0],
              [0, 0, 0, 1]]


        if(joint_name.endswith('YawPitch')):
            T = np.dot(Rz, Ry)
        elif(joint_name.endswith('Yaw')):
            T = Rz
        elif(joint_name.endswith('Pitch')):
            T = Ry
        else:
            T = Rx


        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = np.dot(T, Tl)

                self.transforms[joint] = T


if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()

