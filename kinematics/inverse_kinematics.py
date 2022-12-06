'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
from numpy import random, linalg, arctan, identity
import numpy as np



    def from_trans(matrix):
        # return x,y,z
        x, y, z = matrix[3, 0], matrix[3, 1], matrix[3, 2]

        angle_x, angle_y, angle_z = 0, 0, 0

        if matrix[0, 0] == 1:
            angle_x = arctan(matrix[2, 1] / matrix[1, 1])
        elif matrix[1, 1] == 1:
            angle_y = arctan(matrix[0, 2] / matrix[0, 0])
        elif matrix[2, 2] == 1:
            angle_z = arctan(matrix[1, 0] / matrix[0, 0])

        return [x, y, z, angle_x, angle_y, angle_z]


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        
        
        lambda_ = 1
        max_step = 0.1

        joint_angles = np.random.random(len(self.chains[effector_name]))
        target = np.matrix([self.from_trans(transform)]).T

        for i in range(1000):
            self.forward_kinematics(joint_angles)

            T = [0] * len(self.chains[effector_name])
            for i, name in enumerate(self.chains[effector_name]):
                T[i] = self.transforms[name]

            Te = np.matrix([self.from_trans(T[-1])]).T
            
            e = target - Te
            e[e > max_step] = max_step
            e[e < -max_step] = -max_step
            T = np.matrix([self.from_trans(j) for j in T[0:-1]]).T
            J = Te - T
            d = Te - T
            
            J[0, :] = d[2, :]
            J[1, :] = d[1, :]
            J[2, :] = d[0, :]
            J[-1, :] = 1
            
            d_theta = lambda_ * np.linalg.pinv(J) * e

            joint_angles[name] += np.asarray(d_theta.T)[0]

            if np.linalg.norm(d_theta) < 1e-4:
                break
        
        
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        
        joint_angles = self.inverse_kinematics(effector_name, transform)

        names = self.chains[effector_name]
        times = [[0, 5]] * len(names)
        keys = []
        for i, name in enumerate(names):
            keys.insert(i, [[self.perception.joint[name], [3, 0, 0]], [joint_angles[name], [3, 0, 0]]])

        self.keyframes = ([names], [times], [keys])  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
