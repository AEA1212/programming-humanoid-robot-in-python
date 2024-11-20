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
import numpy as np


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE

        def extract_transform_features(transform):
            # return: [x, y, z, yaw]
            # print(transform[:3,3])
            position = transform[:3, 3]
            orientation_yaw = np.atan2(transform[2, 1], transform[2, 2])  # Calculate yaw angle
            return np.append(position, orientation_yaw)

        # Hyperparameters
        step_size = 0.1
        max_iterations = 1000
        tolerance = 1e-3
        max_adjustment = 0.5

        joint_angles = {joint: self.perception.joint[joint] for joint in self.chains[effector_name]}
        target_state = extract_transform_features(transform)
        for joint in self.joint_names:
            if joint not in joint_angles:
                joint_angles[joint] = 0

        for iteration in range(max_iterations):
            self.forward_kinematics(joint_angles)

            current_transform = [x for x in self.transforms.values()]
            current_state = extract_transform_features(current_transform)

            error_vector = target_state - current_state

            if np.norm(error_vector) < tolerance:
                break

            jacobian = np.zeros((4, len(self.chains[effector_name])))
            for i, joint in enumerate(self.chains[effector_name]):
                original_angle = joint_angles[joint]
                joint_angles[joint] += 1e-5
                self.forward_kinematics(joint_angles)
                perturbed_state = extract_transform_features(self.transforms[effector_name])
                jacobian[:, i] = (perturbed_state - current_state) / 1e-5
                joint_angles[joint] = original_angle

            angle_adjustments = step_size * np.pinv(jacobian).dot(error_vector)
            for i, joint in enumerate(self.chains[effector_name]):
                joint_angles[joint] += np.clip(angle_adjustments[i], -max_adjustment, max_adjustment)

        return [joint_angles[joint] for joint in self.chains[effector_name]]

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name, transform)
        effector_chain = self.chains[effector_name]
        print(joint_angles)

        keys = [
            [
                [self.perception.joint[joint], [3, 0, 0], [3, 0, 0]],
                [joint_angles[i], [3, 0, 0], [3, 0, 0]],
            ]
            for i, joint in enumerate(effector_chain)
        ]

        time_intervals = [[2.0, 6.0]] * len(effector_chain)
        self.keyframes = (effector_chain, time_intervals, keys)
        print("Generated Keyframes:", self.keyframes)


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = np.identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
