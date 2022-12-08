#!/usr/bin/env python3

import os
import cv2
import numpy as np
import rospy
from intera_interface import Limb
from feature_extractor import FeatureExtractor
from scanner import Scanner
from path_planner import PathPlanner
from controller import Controller
from light_controller import LightController
from moveit_msgs.msg import OrientationConstraint
from skimage.filters import threshold_local
import imutils



if __name__=="__main__":
	rospy.init_node('moveit_node')
	scan = Scanner()
	cwd = os.path.realpath(os.path.dirname(__file__))
	files = [filename for filename in os.listdir(cwd + "/images") if ".jpg" in filename]
	print("\n\nInput integer of image you want: \n")
	print("-1: Scan Webcam")
	for i in range(len(files)):
		print(f"{i}: {files[i]}")
	selection = int(input("\n..."))
	selection = int(selection)
	img = None
	threshold_sample_size = 201
	if selection == -1:
		img_name = ""
		#Choose Webcam
		cv2.namedWindow("preview")
		vc = cv2.VideoCapture(0)
		if vc.isOpened(): # try to get the first frame
			rval, frame = vc.read()
		else:
			rval = False
		while rval:
			cv2.imshow("preview", frame)
			rval, frame = vc.read()
			key = cv2.waitKey(1)
			if key == 27: # exit on ESC
				break
			if key == 13:
				# Enter pressed
				img_name = cwd + "/images/webcam.jpg"
				cv2.imwrite(img_name, frame)
				print("Image taken!")
				break
		vc.release()
		cv2.destroyWindow("preview")

		image = cv2.imread(img_name)
		ratio = image.shape[0] / 500.0
		orig = image.copy()
		image = imutils.resize(image, height = 500)
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray, (5, 5), 0)
		edged = cv2.Canny(gray, 75, 200)
		# find the contours in the edged image, keeping only the
		# largest ones, and initialize the screen contour
		cnts, _ = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

		print("STEP 1: Edge Detection")
		cv2.imshow("Image", image)
		cv2.imshow("Edged", edged)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

		screenCnt = scan.find_screen(image, cnts)
		print(screenCnt is None)
		print("STEP 2: Find contours of paper")
		cv2.drawContours(image, [screenCnt], -1, (0, 255, 0), 2)
		cv2.imshow("Outline", image)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

		warped = scan.transform_image(orig, screenCnt)
		# warped = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
		# T = threshold_local(warped, 11, offset = 10, method = "gaussian")
		# warped = (warped > T).astype("uint8") * 25

		print("STEP 3: Apply perspective transform")
		cv2.imshow("Original", imutils.resize(orig, height = 650))
		cv2.imshow("Scanned", imutils.resize(warped, height = 650))
		cv2.waitKey(0)
		cv2.imwrite(cwd + "/images/webcam.jpg", warped)
		cv2.destroyAllWindows()
		img = cv2.imread(cwd + "/images/webcam.jpg")
		threshold_sample_size = 11
	else:
		img = cv2.imread(cwd + "/images/" + files[selection])


	feature_extractor = FeatureExtractor(threshold_sample_size=threshold_sample_size)
	img = imutils.resize(img, height = 600)
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (5, 5), 0)
	edged = cv2.Canny(gray, 75, 200)

	print("STEP 1: Edge Detection")
	cv2.imshow("Image", img)
	cv2.waitKey(0)
	cv2.imshow("Edged", edged)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	contours, _ = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	# countors = imutils.grab_contours(contours)
	contours = feature_extractor.filter_contours_by_len(contours)

	# img = feature_extractor.resize_img(img)
	# feature_extractor.draw_image(img, "original")
	# x = img.shape[0]
	# y = img.shape[1]
	# contour_img = feature_extractor.create_empty_img(x, y)
	# contours, hierarchy = feature_extractor.extract_contour(img)
	# contours = feature_extractor.filter_contours_by_len(contours)


	colors = feature_extractor.extract_color(img, contours)

	colored_image = np.zeros(img.shape)
	for i, contour in enumerate(contours):
		cv2.drawContours(colored_image, [contour], -1, (colors[i][0], colors[i][1], colors[i][2]), 2)
	cv2.imwrite(cwd + "/images/color.jpg", colored_image)

	# for contour in contours:
	#     print(contour.squeeze(axis=1).shape)
	# cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 1)
	# feature_extractor.draw_image(feature_extractor.invert_black_white(contour_img), "contours")

	cv2.imshow("Color", cv2.imread(cwd + "/images/color.jpg"))
	cv2.waitKey(0)
	cv2.destroyAllWindows()


	path_planner = PathPlanner()
	# input_coords = np.arange(0, 2 * np.pi, .1)
	# simple_circle = .1 * np.array((np.cos(input_coords), 0.2 + np.sin(input_coords)))
	# t1 = np.ones(simple_circle[0].shape).T.reshape((1, simple_circle.shape[1])) * .75
	# t2 = simple_circle
	# simple_circle = np.vstack((t1, t2))
	# plan, _ = path_planner.plan_along_path(simple_circle)
	orien_const = OrientationConstraint()
	orien_const.link_name = "right_hand";
	orien_const.header.frame_id = "base";
	orien_const.orientation.y = -1.0;
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
	
	controller = Controller(Kp,Kd,Ki,Kw, Limb("right"))
	controller = path_planner
	for i in range(len(contour_paths)):
		# controller = Controller(Kp,Kd,Ki,Kw, Limb("right"))
		# controller.execute_plan(connector_paths[i])
		light_controller.off()
		if i < len(connector_paths):
			path = path_planner.make_paths_from_poses([connector_paths[i]])[0]
			controller.execute_plan(path)
			
		print(i)
		path = path_planner.make_paths_from_poses([contour_paths[i]])
		b, g, r = colors[i]
		print(r, g, b)
		light_controller.on_color(b, g, r, 0)
		controller.execute_plan(path[0])
		# controller = Controller(Kp,Kd,Ki,Kw, Limb("right"))
	
		
	light_controller.off()

	