#!/usr/bin/env python3

import numpy as np
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
from voxelizer import Voxelizer

EPS = 1e-4

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
        self.voxelizer = Voxelizer()

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
        list_of_lists = []
        for contour in contours:
            new_contour_list = []
            for i, point in enumerate(contour):
                if i % rate == 0:
                    new_contour_list.append(point)
            new_contour_list = np.array(new_contour_list)
            list_of_lists.append(new_contour_list)
        return list_of_lists

    def connect_contours(self, contours):
        """
        Connects all the contours in the 'contours' list and adds filler points between contours
        """
        new_contour_list = [contours[0].squeeze(1)]
        connectors = []
        for i in range(len(contours) - 1):
            connecting = np.array((contours[i][0][-1], contours[i + 1][0][0]))
            connectors.append(connecting)
            new_contour_list.append(contours[i + 1].squeeze(1))
        # print(np_contours.shape)
        return tuple(new_contour_list), tuple(connectors)
        
    def transform_contours(self, contours):
        """
        contours: a single level list of all connected contours in image frame coordinates
        returns:
         - a single leve list of all the contour coordinates in world frame coordinates 
         - on indices for the light
        """
        transformed = []
        for contour in contours:
            # print(contour.shape)
            contour = contour.T
            transformation = np.array([
                [0, 0, .65],
                [.5 / 500, 0, -.25],
                [0, -.5 / 500, .6],
            ])
            dim = contour.shape[1]
            unos = np.ones((1, dim)).reshape((1, dim))
            homo_coords = np.vstack((contour, unos))
            temp = transformation @ homo_coords
            transformed.append(temp)
        return transformed

    def ndarray_to_pos(self, contours):
        """
        Creates an array of pose messages from an ndarray
        """
        list_of_poses = []
        for arr in contours:
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
            # print(f"Number of poses: {len(poses)}")
            list_of_poses.append(poses)
        return list_of_poses

    def toggle_index_scaler(self, path, connected_contours, toggle_indices):
        scale = float(len(path.joint_trajectory.points)) / float(len(connected_contours))
        new_toggle_indices = set()
        for i in toggle_indices:
            new_toggle_indices.add(int(i//scale))
        return new_toggle_indices

    def make_paths_from_poses(self, poses, connector=False):
        if connector:
            scaling = .2
        else:
            scaling = .3
        paths = []
        for pose_list in poses:
            # print(pose_list)
            path, _ = self._group.compute_cartesian_path(pose_list, .01, 0)
            path = self._group.retime_trajectory(self._robot.get_current_state(), path, scaling)
            paths.append(path)
        return paths

    def plan_along_path(self, contours, orientation_constraints=None):
        """
        Plans a path for the robot given contours to draw and the end effector orientation constraints
        """
        if orientation_constraints is not None:                
            constraints = Constraints()
            constraints.orientation_constraints = orientation_constraints
            self._group.set_path_constraints(constraints)
        contours, connectors = self.connect_contours(contours)
        print(f"contours[0]: {contours[0].shape}, connectors[0]: {connectors[0].shape}")
        # for p in connected_contours:
        #     print(p)
        # print(connected_contours)
        contours = self.transform_contours(contours)
        connectors = self.transform_contours(connectors)
        #HEREEE
        contours = self.ndarray_to_pos(contours)
        connectors = self.ndarray_to_pos(connectors)
        ##HERREEEE
        contours = self.sample(contours, 20)
        #HEREEE
        # contour_paths = self.make_paths_from_poses(contours)
        # connector_paths = self.make_paths_from_poses(connectors, True)
        # print(on_indices)
        # print(len(contour_paths))
        # print(len(connectors))
        return contours, connectors

    def execute_plan(self, plan):
        """
        Executes the planned path
        """
        return self._group.execute(plan, True)

    

