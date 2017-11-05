# camera_pose_estimate.py
# Main logic to reconstruct the 3D model.



import math
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import cv2				#OpenCV version 2.4.11

import utilities as ut
from utilities import Arrow3D
import QRCode
from mpl_toolkits.mplot3d import Axes3D

# Import all global constants and coefficients
from settings import *
# Import global model figure
import global_figure as gf



def camPoseEstimate(image_path):
	'''
	Get estimated camera position and orientation in 3D world coordinates.

	Input:
		image_path: Input image path - string
	Output:
		Coordinates of camera in 3D world coordinates and its orientation matrix - numpy.array, numpy.array
	'''
	image = cv2.imread(image_path)
	gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	# OpenCV on Mac OSX has some issue on image size swapping while reading
	# Mac user might need this line to rotate the image by 90 degree clockwise
	image = cv2.flip(cv2.transpose(image), 1)

	size = image.shape

	# Pattern points in 2D image coordinates
	pattern_points = np.array(QRCode.detectQRCode(image), dtype='double')
	#print pattern_points
	p1 = pattern_points.astype(int)
	print p1

	# Pattern points in 3D world coordinates.
	model_points = np.array([	(-QRCodeSide / 2, QRCodeSide / 2, 0.0), 
								(QRCodeSide / 2, QRCodeSide / 2, 0.0), 
								(QRCodeSide / 2, -QRCodeSide / 2, 0.0), 
								(-QRCodeSide / 2, -QRCodeSide / 2, 0.0), 
							])

	focal_length = size[1]
	camera_center = (size[1] / 2, size[0] / 2)

	# Initialize approximate camera intrinsic matrix
	camera_intrinsic_matrix = np.array([[focal_length, 0, camera_center[0]],
                         				[0, focal_length, camera_center[1]],
                         				[0, 0, 1]
                         				], dtype = "double")

	# Assume there is no lens distortion
	dist_coeffs = np.zeros((4, 1))

	# Get camera extrinsic matrix - R and T
	flag, rotation_vector, translation_vector = cv2.solvePnP(model_points,pattern_points, camera_intrinsic_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE	)

	# Convert 3x1 rotation vector to rotation matrix for further computation
	rotation_matrix, jacobian = cv2.Rodrigues(rotation_vector)
	C = np.matmul(-rotation_matrix.transpose(), translation_vector)
	# Orientation vector
	O = np.matmul(rotation_matrix.T, np.array([0, 0, 1]).T)
	axisImgPoints, jac = cv2.projectPoints(model_points, rotation_vector, translation_vector, camera_intrinsic_matrix, dist_coeffs)
	#draw(image_path,pattern_points,axisImgPoints,C.squeeze(),O)
	return C.squeeze(), O, p1






def visualize3D(image_path):
	img = cv2.imread(image_path)
	# cv2.resize(img,(640,480))
	'''
	Visualize 3D model with Matplotlib 3D.
	Input:
		image_path: Input image path - string
	Output:
		None - 
	'''
	# Get camera 3D position coordinates in world system
	camera_pose, camera_orientation, corners = camPoseEstimate(image_path)

	# Equal the unit scale, some embellishment
	max_unit_length = max(30, max(camera_pose[:2])) + 10
	gf.ax.set_xlim3d(-max_unit_length, max_unit_length)
	gf.ax.set_ylim3d(-max_unit_length, max_unit_length)
	gf.ax.set_zlim3d(-1, 100)

	# Decompose the camera coordinate
	arrow_length = camera_pose[2] * 0.8
	xs = [camera_pose[0], camera_pose[0] + camera_orientation[0] * arrow_length]
	ys = [camera_pose[1], camera_pose[1] + camera_orientation[1] * arrow_length]
	zs = [camera_pose[2], camera_pose[2] + camera_orientation[2] * arrow_length]
	#corner = tuple(corners[0])
	#cv2.line(img, corner, (int(zs[0]), int(zs[1])), (0, 0, 255), 5)
	#cv2.imshow("cool",img)

	# Plot camera location
	gf.ax.scatter([camera_pose[0]], [camera_pose[1]], [camera_pose[2]])
	label = '%s (%d, %d, %d)' % (ut.getImageName(image_path), camera_pose[0], camera_pose[1], camera_pose[2])
	gf.ax.text(camera_pose[0], camera_pose[1], camera_pose[2], label)
	arrow = Arrow3D(xs, ys, zs, mutation_scale=5, lw=2, arrowstyle="-|>", color="k")
	gf.ax.add_artist(arrow)
	
	# Prepare pattern image
	# To cater to the settings of matplotlib, 
	# we need the second line here to rotate the image 90 degree counterclockwise
	pattern_image = cv2.imread(pattern_file_path)
	pattern_image = cv2.flip(cv2.transpose(pattern_image), 0)

	# Plot pattern
	val = QRCodeSide * pattern_image.shape[0] / PatternSide
	xx, yy = np.meshgrid(np.linspace(-val, val, pattern_image.shape[0]), 
		np.linspace(-val, val, pattern_image.shape[0]))
	X = xx
	Y = yy
	Z = xx * 0 + yy * 0
	#Z = 0
	#Z = np.asarray(Z,(2,2))

	gf.ax.plot_surface(X, Y, Z, rstride=10, cstride=10, facecolors=pattern_image / 255., shade=False)



