#!/usr/bin/env python3

import numpy as np
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive

class PathPlanner(object):

    def __init__(self, group_name="right_arm"):
        """
        Constructor.
        
        Inputs: group_name: the name of the move_group.
        """

        rospy.on_shutdown(self.shutdown)

        moveit_commander.roscpp_initialize(sys.argv)

        self._robot = moveit_commander.RobotCommander()

        self._scene = moveit_commander.PlanningSceneInterface()

        self._planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

        # #NEED TO FIND SOME SORT OF BOOLEAN MESSAGE TYPE
        # self._light_control_publisher = rospy.Publisher('/light_toggle', None, queue_size=10)

        self._group = moveit_commander.MoveGroupCommander(group_name)

        self._group.set_workspace([-2, -2, -2, 2, 2, 2])

        self.orien_const = OrientationConstraint()
        self.orien_const.link_name = "right_gripper"
        self.orien_const.header.frame_id = "base"
        self.orien_const.orientation.y = -1.0
        self.orien_const.absolute_x_axis_tolerance = 0.1
        self.orien_const.absolute_y_axis_tolerance = 0.1
        self.orien_const.absolute_z_axis_tolerance = 0.1
        self.orien_const.weight = 1.0

        rospy.sleep(0.5)

    def shutdown(self):
        """
        Shutdown procedure"""
        self._group = None
        rospy.loginfo("Stopping Path Planner")

    def distance(self, x, y):
        """
        Calculates Euclidean distance between two 2D points
        """
        return abs(((x[0] - y[0])**2 + (x[1] - y[1])**2)**.5)

    def sample(self, contours, rate):
        new_contour_list = []
        for i, contour in enumerate(contours):
            if i % rate == 0:
                new_contour_list.append(contour)
        return np.array(new_contour_list)

    def connect_contours(self, contours):
        """
        Connects all the contours in the 'contours' list and adds filler points between contours
        """
        new_contour_list = []
        last_end = None
        for contour in contours:
            contour = np.squeeze(contour, axis=1)
            if last_end is not None:
                beginning = contour[0]
                slope = (beginning[1] - last_end[1]) / (beginning[0] - last_end[0])
                for i in range(5 * int(self.distance(last_end, beginning))):
                    np.append(new_contour_list, np.array(last_end[0] + i * .2, last_end[1] + i * .2 * slope))
            for point in contour:
                new_contour_list.append(point)
            last_end = contour[-1]
        np_contours = np.array(new_contour_list)
        print(np_contours.shape)
        return np_contours
        
    def transform_contours(self, contours):
        """
        contours: a single level list of all connected contours in image frame coordinates
        returns: a single leve list of all the contour coordinates in world frame coordinates 
        """
        #TODO
        contours = contours.T
        transformation = np.array([
            [0, 0, .7],
            [.8 / 500, 0, -.4],
            [0, -.8 / 500, .8],
        ])
        dim = contours.shape[1]
        unos = np.ones((1, dim)).reshape((1, dim))
        homo_coords = np.vstack((contours, unos))
        return transformation @ homo_coords

    def ndarray_to_pos(self, arr):
        """
        Creates an array of pose messages from an ndarray
        """
        poses = []
        for i in range(arr.shape[1]):
            pos = Pose()
            pos.position.x = arr[0][i]
            pos.position.y = arr[1][i]
            pos.position.z = arr[2][i]
            pos.orientation.x = 1
            pos.orientation.y = 0
            pos.orientation.z = 0
            pos.orientation.w = 0
            poses.append(pos)
        print(f"Number of poses: {len(poses)}")
        return poses

    def plan_along_path(self, contours, orientation_constraints=None):
        """
        Plans a path for the robot given contours to draw and the end effector orientation constraints
        """
        if orientation_constraints is not None:                
            slope = (beginning[1] - last_end[1]) / (beginning[0] - last_end[0])
            constraints = Constraints()
            constraints.orientation_constraints = orientation_constraints
            self._group.set_path_constraints(constraints)
        connected_contours = self.connect_contours(contours)
        for p in connected_contours:
            print(p)
        # print(connected_contours)
        connected_contours = self.transform_contours(connected_contours)
        connected_contours = self.ndarray_to_pos(connected_contours)
        connected_contours = self.sample(connected_contours, 3)
        return self._group.compute_cartesian_path(connected_contours, .01, 0)

    def execute_plan(self, plan):
        """
        Executes the planned path
        """
        return self._group.execute(plan, True)

    

