#!/usr/bin/env python3

import os
import cv2
import numpy as np
import rospy
from intera_interface import Limb
from pixel_extractor import PixelExtractor
from path_planner import PathPlanner
from controller import Controller
from light_controller import LightController
from moveit_msgs.msg import OrientationConstraint


if __name__=="__main__":
    rospy.init_node('moveit_node')

    # Select image
    cwd = os.path.realpath(os.path.dirname(__file__))
    files = [filename for filename in os.listdir(cwd + "/images") if ".jpg" in filename]
    print("\n\nInput integer of image you want: \n")
    for i in range(len(files)):
        print(f"{i}: {files[i]}")
    selection = int(input("\n..."))
    selection = int(selection)
    img = cv2.imread(cwd + "/images/" + files[selection])

    # Normalize image
    pixel_extractor = PixelExtractor()
    img = pixel_extractor.resize_img(img)
    pixel_extractor.draw_image(img, "original")
    rows = img.shape[0]      # not width of image
    columns = img.shape[1]
    #print(x)

    # Physical properties
    width_mm = 500         # (mm)
    num_leds = 26
    pixel_spacing = 6.48
    width_pixels = int(width_mm / pixel_spacing) + 1
    scale = width_pixels / img.shape[1]
    width = int(img.shape[1] * scale)
    height = int(img.shape[0] * scale)
    dim = (width, height)
    img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)      # 1 pixel = 1 led unit
    pixel_extractor.draw_image(img, "scaled")

    

    # Create contours
    contour_img = pixel_extractor.create_empty_img(rows, columns)
    contours, hierarchy = pixel_extractor.extract_contour(img)
    contours = pixel_extractor.filter_contours_by_len(contours)
    # for contour in contours:
    #     print(contour.squeeze(axis=1).shape)
    cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 1)
    pixel_extractor.draw_image(pixel_extractor.invert_black_white(contour_img), "contours")

    # Plan paths
    path_planner = PathPlanner()
    # input_coords = np.arange(0, 2 * np.pi, .1)
    # simple_circle = .1 * np.array((np.cos(input_coords), 0.2 + np.sin(input_coords)))
    # t1 = np.ones(simple_circle[0].shape).T.reshape((1, simple_circle.shape[1])) * .75
    # t2 = simple_circle
    # simple_circle = np.vstack((t1, t2))
    # plan, _ = path_planner.plan_along_path(simple_circle)
    orien_const = OrientationConstraint()           # using an orientation constraint
    orien_const.link_name = "right_hand";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;               # keep end effector pointing down
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;
    contour_paths, connector_paths = path_planner.plan_along_path(contours, [orien_const])
    # print(connected_contours.shape)
    input("check rviz bruh")
    Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 1, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    light_controller = LightController()
    
    # Do motion control
    # controller = Controller(Kp,Kd,Ki,Kw, Limb("right"))           # lab 7 controller
    controller = path_planner           # moveit controller
    for i in range(len(contour_paths)):
        # controller = Controller(Kp,Kd,Ki,Kw, Limb("right"))
        # controller.execute_plan(connector_paths[i])
        light_controller.off()
        if i < len(connector_paths):
            path = path_planner.make_paths_from_poses([connector_paths[i]])[0]
            controller.execute_plan(path)
            
        print(i)
        path = path_planner.make_paths_from_poses([contour_paths[i]])
        light_controller.on()
        controller.execute_plan(path[0])
        # controller = Controller(Kp,Kd,Ki,Kw, Limb("right"))
    
        
    light_controller.off()