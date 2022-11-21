import numpy as np
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from geometry_msgs.msg import PoseStamped
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

        #NEED TO FIND SOME SORT OF BOOLEAN MESSAGE TYPE
        self._light_control_publisher = rospy.Publisher('/light_toggle', None, queue_size=10)

        self._group = moveit_commander.MoveGroupCommander(group_name)

        self._group.set_workspace([-2, -2, -2, 2, 2, 2])

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

    def connect_contours(self, contours):
        """
        Connects all the contours in the 'contours' list and adds filler points between contours
        """
        new_contour_list = np.array([])
        last_end = None
        for contour in contours:
            if last_end is not None:
                beginning = contour[0]
                slope = (beginning[1] - last_end[1]) / (beginning[0] - last_end[0])
                for i in range(5 * int(self.distance(last_end, beginning))):
                    np.append(new_contour_list, np.array(last_end[0] + i * .2, last_end[1] + i * .2 * slope))
            np.append(new_contour_list, contour)
            last_end = contour[-1]
        
    def transform_contours(self, contours):
        """
        contours: a single level list of all connected contours in image frame coordinates
        returns: a single leve list of all the contour coordinates in world frame coordinates 
        """
        #TODO
        return contours

    def plan_along_path(self, contours, orientation_constraints):
        """
        Plans a path for the robot given contours to draw and the end effector orientation constraints
        """
        constraints = Constraints()
        constraints.orientation_constraints = orientation_constraints
        self._group.set_path_constraints(constraints)
        connected_contours = self.connect_contours(contours)
        return self._group.compute_cartesian_path(connected_contours, .01, 0)

    def execute_plan(self, plan):
        """
        Executes the planned path
        """
        return self._group.execute(plan, True)