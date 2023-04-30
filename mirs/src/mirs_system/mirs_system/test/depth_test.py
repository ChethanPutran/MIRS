import os
import cv2
import numpy as np
import matplotlib.image as im
import matplotlib.pyplot as plt
from mirs_system.ai.vision.estimator.depth_estimator import CoordinateEstimator
from mirs_system.ai.vision.estimator import depth_estimator as de
from mirs_system.ai.vision.calibration.raspberrypi import Raspberrypi
# import open3d as o3d


def ShowDisparity(imgLeft,imgRight,numDisparities = 16,bSize=5):
    # Initialize the stereo block matching object 
    stereo = cv2.StereoBM_create(numDisparities=numDisparities, blockSize=bSize)

    # Compute the disparity image
    disparity = stereo.compute(imgLeft, imgRight)

    # Normalize the image for representation
    min = disparity.min()
    max = disparity.max()
    disparity = np.uint8(255 * (disparity - min) / (max - min))
    
    # Plot the result
    return disparity


get_new_images = False

files  = [os.path.join(os.path.dirname(__file__),"left_sample_image.jpg"),
                        os.path.join(os.path.dirname(__file__),"right_sample_image.jpg")]
if get_new_images:
    
    rp = Raspberrypi()
    data,err = rp.get_sample_image(os.path.dirname(__file__))
    rp.close()

    if err:
        print(err)
        print("Using default images...")
    else:
        files = data
        print("Using new images...")

img_left_pth = files[0]
img_right_pth = files[1]

left_img = cv2.imread(img_left_pth,0)
right_img = cv2.imread(img_right_pth,0)

disparity = ShowDisparity(left_img,right_img, numDisparities=32,bSize=11)


# FX_DEPTH = 2.6
# FY_DEPTH = 2.6
# CX_DEPTH = 2
# CY_DEPTH = 1

# FX_RGB = 2.6
# FY_RGB = 2.6
# CX_RGB = 2
# CY_RGB = 1

# colors = []
# pcd = []

# R,T = de.R_LR,de.T_LR
# height, width = disparity.shape

# for i in range(height):
#     for j in range(width):
#         """
#             Convert the pixel from depth coordinate system
#             to depth sensor 3D coordinate system
#         """
#         z = disparity[i][j]
#         x = (j - CX_DEPTH) * z / FX_DEPTH
#         y = (i - CY_DEPTH) * z / FY_DEPTH

#         """
#             Convert the point from depth sensor 3D coordinate system
#             to rgb camera coordinate system:
#         """
#         [x_RGB, y_RGB, z_RGB] = np.linalg.inv(R).dot([x, y, z]) - np.linalg.inv(R).dot(T)

#         """
#             Convert from rgb camera coordinates system
#             to rgb image coordinates system:
#         """
#         j_rgb = int((x_RGB * FX_RGB) / z_RGB + CX_RGB + width / 2)
#         i_rgb = int((y_RGB * FY_RGB) / z_RGB + CY_RGB)

#         # Add point to point cloud:
#         pcd.append([x, y, z])

#         # Add the color of the pixel if it exists:
#         if 0 <= j_rgb < width and 0 <= i_rgb < height:
#             colors.append(imgL[i_rgb][j_rgb] / 255)
#         else:
#             colors.append([0., 0., 0.])
            
# # Convert to Open3D.PointCLoud:
# pcd_o3d = o3d.geometry.PointCloud()  # create a point cloud object
# pcd_o3d.points = o3d.utility.Vector3dVector(pcd)
# pcd_o3d.colors = o3d.utility.Vector3dVector(colors)
# # Visualize:
# o3d.visualization.draw_geometries([pcd_o3d])

# def nothing(x):
#     pass
 
# cv2.namedWindow('disp',cv2.WINDOW_NORMAL)
# cv2.resizeWindow('disp',600,600)
 
# cv2.createTrackbar('numDisparities','disp',1,17,nothing)
# cv2.createTrackbar('blockSize','disp',5,50,nothing)
# cv2.createTrackbar('preFilterType','disp',1,1,nothing)
# cv2.createTrackbar('preFilterSize','disp',2,25,nothing)
# cv2.createTrackbar('preFilterCap','disp',5,62,nothing)
# cv2.createTrackbar('textureThreshold','disp',10,100,nothing)
# cv2.createTrackbar('uniquenessRatio','disp',15,100,nothing)
# cv2.createTrackbar('speckleRange','disp',0,100,nothing)
# cv2.createTrackbar('speckleWindowSize','disp',3,25,nothing)
# cv2.createTrackbar('disp12MaxDiff','disp',5,25,nothing)
# cv2.createTrackbar('minDisparity','disp',5,25,nothing)
 
# # Creating an object of StereoBM algorithm
# stereo = cv2.StereoBM_create()
 

# imgR_gray = cv2.cvtColor(right_img,cv2.COLOR_BGR2GRAY)
# imgL_gray = cv2.cvtColor(left_img,cv2.COLOR_BGR2GRAY)



# # Updating the parameters based on the trackbar positions
# numDisparities = cv2.getTrackbarPos('numDisparities','disp')*16
# blockSize = cv2.getTrackbarPos('blockSize','disp')*2 + 5
# preFilterType = cv2.getTrackbarPos('preFilterType','disp')
# preFilterSize = cv2.getTrackbarPos('preFilterSize','disp')*2 + 5
# preFilterCap = cv2.getTrackbarPos('preFilterCap','disp')
# textureThreshold = cv2.getTrackbarPos('textureThreshold','disp')
# uniquenessRatio = cv2.getTrackbarPos('uniquenessRatio','disp')
# speckleRange = cv2.getTrackbarPos('speckleRange','disp')
# speckleWindowSize = cv2.getTrackbarPos('speckleWindowSize','disp')*2
# disp12MaxDiff = cv2.getTrackbarPos('disp12MaxDiff','disp')
# minDisparity = cv2.getTrackbarPos('minDisparity','disp')
    
# # Setting the updated parameters before computing disparity map
# stereo.setNumDisparities(numDisparities)
# stereo.setBlockSize(blockSize)
# stereo.setPreFilterType(preFilterType)
# stereo.setPreFilterSize(preFilterSize)
# stereo.setPreFilterCap(preFilterCap)
# stereo.setTextureThreshold(textureThreshold)
# stereo.setUniquenessRatio(uniquenessRatio)
# stereo.setSpeckleRange(speckleRange)
# stereo.setSpeckleWindowSize(speckleWindowSize)
# stereo.setDisp12MaxDiff(disp12MaxDiff)
# stereo.setMinDisparity(minDisparity)

# # Calculating disparity using the StereoBM algorithm
# disparity = stereo.compute(imgL_gray,imgR_gray)
# disparity = disparity.astype(np.float32)
# disparity = (disparity/16.0 - minDisparity)/numDisparities

# # Displaying the disparity map
# plt.imshow(disparity)
# plt.show()


# ce = CoordinateEstimator(160,11,window_size=5,BM=True)

# ce.display_image(left_img,right_img)
# disp_map = ce.compute_disparity_map(left_img,right_img)
# ce.display_disparity_map(disp_map)
# dm = ce.compute_depth_map(disp_map,use_default=True)
# ce.display_depth_map(dm)


# point = (250,600)
# print(ce.get_point_depth(left_img,right_img,point))
# print(ce.depth_map)
