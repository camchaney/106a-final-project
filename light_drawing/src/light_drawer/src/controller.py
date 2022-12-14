#!/usr/bin/env python
"""
Controller Class for Lab 8
Author: Valmik Prabhu, Chris Correa
"""

import rospy
import sys
import numpy as np
import itertools
from collections import deque

import baxter_interface
import intera_interface

from moveit_msgs.msg import RobotTrajectory

from voxelizer import Voxelizer
import example_forward_kinematics as efk



class Controller(object):
    """
    A controller object

    Fields:
    _Kp: 7x' ndarray of proportional constants
    _Ki: 7x' ndarray of integral constants
    _Kd: 7x' ndarray of derivative constants
    _Kw: 7x' ndarray of antiwindup constants
    _LastError: 7x' ndarray of previous position errors
    _LastTime: Time from start at which LastError was updated (in sec)
    _IntError: 7x' ndarray of integrated error values
    _path: a moveit_msgs/RobotTrajectory message
    _curIndex: the current index in the path
    _maxIndex: maximum index in the path
    _limb: baxter_interface.Limb or intera_interface.Limb

    _times: For Plotting
    _actual_positions: For Plotting
    _actual_velocities: For Plotting
    _target_positions: For Plotting
    _target_velocities: For Plotting

    Methods:
    __init__(self, Kp, Ki, Kd, Kw): constructor

    """

    def __init__(self, Kp, Ki, Kd, Kw, limb):
        """
        Constructor:

        Inputs:
        Kp: 7x' ndarray of proportional constants
        Ki: 7x' ndarray of integral constants
        Kd: 7x' ndarray of derivative constants
        Kw: 7x' ndarray of antiwindup constants
        limb: sawyer_interface.Limb
        """

        # If the node is shutdown, call this function
        rospy.on_shutdown(self.shutdown)

        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd
        self._Kw = Kw

        self._LastError = np.zeros(len(Kd))
        self._LastTime = 0;
        self._IntError = np.zeros(len(Ki))
        self._ring_buff_capacity = 3
        self._ring_buff = deque([], self._ring_buff_capacity)

        self._path = RobotTrajectory()
        self._curIndex = 0;
        self._maxIndex = 0;

        self._limb = limb

        # For Plotting:
        self._times = list()
        self._actual_positions = list()
        self._actual_velocities = list()
        self._target_positions = list()
        self._target_velocities = list()

        # For voxelization of 3d space
        self.voxelizer = Voxelizer()
    
    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety
        """
        rospy.loginfo("Stopping Controller")

        # Set velocities to zero
        dic_vel = {name:np.zeros(len(self._limb.joint_names())) for name in self._limb.joint_names()}

        self._limb.set_joint_velocities(dic_vel)
        rospy.sleep(0.1)

    def execute_plan(self, path, log=True):
        """
        Execute a given path

        Inputs:
        path: a moveit_msgs/RobotTrajectory message
        timeout: max time the controller will run
        log: should the controller display a plot
        
        """

        self._path = path

        self._curIndex = 0
        self._maxIndex = len(self._path.joint_trajectory.points)-1

        startTime = rospy.Time.now()

        # Set the last error as zero for t = 0
        self._LastError = np.zeros(len(self._Kd))
        self._LastTime = 0.0

        # Set the integral of positions to zero
        self._IntError = np.zeros(len(self._Ki))

        # Set your ring buffer to zero
        self._ring_buff = deque([], self._ring_buff_capacity)

        # Reset plot values
        self._times = list()
        self._actual_positions = list()
        self._actual_velocities = list()
        self._target_positions = list()
        self._target_velocities = list()
        
        r = rospy.Rate(200)

        toggled = set()

        while not rospy.is_shutdown():
            # Find the time from start
            t = (rospy.Time.now() - startTime).to_sec()

            # If the controller has timed out, stop moving and return false
            # if timeout is not None and t >= timeout:
            #     # Set velocities to zero
            #     dic_vel = {name:np.zeros(len(self._limb.joint_names())) for name in self._limb.joint_names()}
            #     self._limb.set_joint_velocities(dic_vel)
            #     return False

            # Get the input for this time
            u = self.step_control(t)

            # Set the joint velocities
            dic_vel = {}
            for i in range(len(self._limb.joint_names())):
                # print(f"joint: {self._limb.joint_names()[i]}, input: {u[i]}, type: {type(u[i])}")
                dic_vel[self._limb.joint_names()[i]] = float(u[i])
            self._limb.set_joint_velocities(dic_vel)
            # Sleep for a defined time (to let the robot move)
            r.sleep()

            # Once the end of the path has been reached, stop moving and break
            if self._curIndex >= self._maxIndex:
                break

        # if log:
        #     import matplotlib.pyplot as plt

        #     times = np.array(self._times)
        #     actual_positions = np.array(self._actual_positions)
        #     actual_velocities = np.array(self._actual_velocities)
        #     target_positions = np.array(self._target_positions)
        #     target_velocities = np.array(self._target_velocities)
        #     plt.figure()
        #     joint_num = len(self._path.joint_trajectory.joint_names)
        #     for joint in range(joint_num):
        #         plt.subplot(joint_num,2,2*joint+1)
        #         plt.plot(times, actual_positions[:,joint], label='Actual')
        #         plt.plot(times, target_positions[:,joint], label='Desired')
        #         plt.xlabel("Time (t)")
        #         if(joint == 0):
        #             plt.ylabel(self._path.joint_trajectory.joint_names[joint] + " Position Error")
        #         else:
        #             plt.ylabel(self._path.joint_trajectory.joint_names[joint])
        #         plt.legend()

        #         plt.subplot(joint_num,2,2*joint+2)
        #         plt.plot(times, actual_velocities[:,joint], label='Actual')
        #         plt.plot(times, target_velocities[:,joint], label='Desired')
        #         plt.xlabel("Time (t)")
        #         if(joint == 0):
        #             plt.ylabel(self._path.joint_trajectory.joint_names[joint] + " Velocity Error")
        #         else:
        #             plt.ylabel(self._path.joint_trajectory.joint_names[joint])
        #         plt.legend()

        #     print("Close the plot window to continue")
        #     plt.show()

        return True


    def permute(self, nums):
        # helper
        def recursive(nums, perm=[], res=[]):
            if not nums: # -- NOTE [1] 
                res.append(perm[::]) #  -- NOTE [2] - append a copy of the perm at the leaf before we start popping/backtracking

            for i in range(len(nums)): # [1,2,3]
                newNums = nums[:i] + nums[i+1:]
                perm.append(nums[i])
                recursive(newNums, perm, res) # - recursive call will make sure I reach the leaf
                perm.pop() # -- NOTE [3] 
            return res

        return recursive(nums)


    def step_control(self, t):
        """
        Return the control input given the current controller state at time t and returns boolean to toggle light
 
        Inputs:
        t: time from start in seconds
        toggle_indices: indices to toggle light

        Output:
        u: 7x' ndarray of velocity commands
        toggle: True or False on whether to toggle light or not
        """
        # Make sure you're using the latest time
        while (not rospy.is_shutdown() and self._curIndex < self._maxIndex and self._path.joint_trajectory.points[self._curIndex+1].time_from_start.to_sec() < t+0.001):
            self._curIndex = self._curIndex+1

        
        # print(f"Current index: {self._curIndex}") 

        # Get current state
        current_position = np.array([self._limb.joint_angles()[joint_name] for joint_name in self._path.joint_trajectory.joint_names])
        current_velocity = np.array([self._limb.joint_velocities()[joint_name] for joint_name in self._path.joint_trajectory.joint_names])


        # permuted = np.array(self.permute(self._path.joint_trajectory.points[self._curIndex].positions))
        # print(f"NEW for {current_position}")
        # for i in range(permuted.shape[0]):
        #     joints = permuted[i]
        #     print(permuted.shape)
        #     print(joints, i)
        #     # target_so3 = efk.baxter_forward_kinematics_from_angles(np.array(self._path.joint_trajectory.points[self._curIndex].positions))
        #     target_so3 = efk.baxter_forward_kinematics_from_angles(joints)
        #     target_end = target_so3[:3, 3].flatten()
        #     print(target_end)
        # # print(self.voxelizer.convert(tuple(target_end)))
        # # print(self._path.joint_trajectory.joint_names)

        # if self.voxelizer.convert(tuple(target_end)) in on_indices:
        #     turn_on = True

        if self._curIndex < self._maxIndex:
            time_low = self._path.joint_trajectory.points[self._curIndex].time_from_start.to_sec()
            time_high = self._path.joint_trajectory.points[self._curIndex+1].time_from_start.to_sec()

            target_position_low = np.array(self._path.joint_trajectory.points[self._curIndex].positions)
            target_velocity_low = np.array(self._path.joint_trajectory.points[self._curIndex].velocities)

            target_position_high = np.array(self._path.joint_trajectory.points[self._curIndex+1].positions)
            target_velocity_high = np.array(self._path.joint_trajectory.points[self._curIndex+1].velocities)

            target_position = target_position_low + (t - time_low)/(time_high - time_low)*(target_position_high - target_position_low)
            target_velocity = target_velocity_low + (t - time_low)/(time_high - time_low)*(target_velocity_high - target_velocity_low)

        else:
            target_position = np.array(self._path.joint_trajectory.points[self._curIndex].positions)
            target_velocity = np.array(self._path.joint_trajectory.points[self._curIndex].velocities)

        # For Plotting
        self._times.append(t)
        self._actual_positions.append(current_position)
        self._actual_velocities.append(current_velocity)
        self._target_positions.append(target_position)
        self._target_velocities.append(target_velocity)


        # Feed Forward Term
        u_ff = target_velocity

        # Error Term
        error = target_position - current_position

        # Integral Term
        self._IntError = self._Kw * self._IntError + error
        
        # Derivative Term
        dt = t - self._LastTime
        # We implement a moving average filter to smooth the derivative
        curr_derivative = (error - self._LastError) / dt
        self._ring_buff.append(curr_derivative)
        ed = np.mean(self._ring_buff)

        # Save terms for the next run
        self._LastError = error
        self._LastTime = t

        ###################### YOUR CODE HERE #########################

        # Note, you should load the Kp, Ki, Kd, and Kw constants with
        # self._Kp
        # and so on. This is better practice than hard-coding

        u = u_ff + self._Kp * error + self._Kd * ed  + self._Ki * self._IntError

        ###################### YOUR CODE END ##########################

        return u


if __name__ == '__main__': 
    pass



    





