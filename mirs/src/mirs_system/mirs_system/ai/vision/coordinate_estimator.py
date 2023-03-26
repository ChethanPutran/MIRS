import numpy as np
import cv2
from matplotlib import pyplot as plt
from matplotlib import patches


DISTANCE = 10
WIDTH = 2

# classes = ["mobile", "mouse"]

# yolo_net = cv2.dnn.readNet("yolov4-tiny.weights", "yolov4-tiny.cfg")

# model = cv2.dnn_DetectionModel(yolo_net)
# model.setInputParams(size=(416, 416), scale=1/255, swapRB=True)

# cap = cv2.VideoCapture(3)

# while True:
#     status, frame = cap.read()

#     img = frame

PROJECTION_MAT_LEFT = [[640.,     0.,   640.,  2176.],
                       [0.,     480.,   480.,   552.],
                       [0.,       0.,     1.,    1.4]]
PROJECTION_MAT_RIGHT = [[640.,     0.,   640.,  2176.],
                        [0.,     480.,   480.,   792.],
                        [0.,       0.,     1.,    1.4]]


class CoordinateEstimator:
    def __init__(self, num_disparities=6*16,
                 block_size=11,
                 min_disparity=0,
                 window_size=6,
                 projection_mat_left=PROJECTION_MAT_LEFT,
                 projection_mat_right=PROJECTION_MAT_RIGHT,
                 ):
        self.projection_mat_left = projection_mat_left

        self.projection_mat_right = projection_mat_right

        self.num_disparities = num_disparities
        self.block_size = block_size
        self.min_disparity = min_disparity
        self.window_size = window_size
        self.depth_map = []

        # Stereo BM matcher
        # self.matcher = cv2.StereoBM_create( numDisparities = self.num_disparities, blockSize = self.block_size)

        # Stereo SGBM matcher
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=self.min_disparity,
            numDisparities=self.num_disparities,
            blockSize=self.block_size,
            P1=8 * 3 * self.window_size ** 2,
            P2=32 * 3 * self.window_size ** 2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

    def compute_depth(self,img_l,img_r):
        # Step 1. Estimating disparity map
        disparity_map = self.compute_disparity_map(img_l, img_r)

        # Step 2. Decompose projection matrix
        K_left, R_left, t_left = self.decompose_projection_mat(
            self.projection_mat_left)
        *_, t_right = self.decompose_projection_mat(
            self.projection_mat_right)

        # Step 3. Calculate depth map
        self.depth_map = self.compute_depth_map(
            disparity_map, K_left, t_left, t_right)
        


    def compute_disparity_map(self, img_left, img_right, left=True, right=False):
        """ 
        **** To determine the disparity between the two images ****
        """

        img_left_g = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        img_right_g = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)

        disparity_map = np.ones(img_left.shape[:2], np.single)
        if left:
            # Compute the left disparity map
            disparity_map[:] = (self.matcher.compute(
                img_left_g, img_right_g).astype(np.float32)/16)[:]
        else:
            disparity_map[:] = (self.matcher.compute(
                img_left_g, img_right_g).astype(np.float32)/16)[:]

        return disparity_map

    def compute_depth_map(self, disparity_map, K, t_left, t_right):
        """
        1. Get the focal length  𝑓 from the  𝐾 matrix
        2. Compute the baseline  𝑏 using corresponding values from the translation vectors  𝑡
        3. Compute depth map of the image

        Z = (fb)/(x_l - x_r)   (x_l - x_r = d)
        Z = fb/d

        """
        # Get the focal length from the K matrix
        f = K[0, 0]

        # Get the distance between the cameras from the t matrices (baseline)
        b = t_left[1] - t_right[1]

        # Replace all instances of 0 and -1 disparity with a small minimum value (to avoid div by 0 or negatives)
        disparity_map[disparity_map == 0] = 0.1
        disparity_map[disparity_map == -1] = 0.1

        # Initialize the depth map to match the size of the disparity map
        depth_map = np.ones(disparity_map.shape, np.single)

        # Calculate the depths
        depth_map[:] = f * b / disparity_map[:]

        return depth_map

    def decompose_projection_mat(self, projection_mat):
        """
        **** Decompose the projection matrices into the camera intrinsic matrix  𝐾, and extrinsics  𝑅,  𝑡s ****
        """

        # R, K = np.linalg.qr(projection_mat[:, :3])
        # t = -np.matmul(np.linalg.inv(K), projection_mat[:, 3].reshape(3, -1))
        # t = t/t[3]

        K, R, t, *_ = cv2.decomposeProjectionMatrix(projection_mat)
        t = t / t[3]

        return K, R, t

    def calculate_nearest_point(self, depth_map, object_location, object_image):

        # Gather the relative parameters of the object box
        object_height, object_width, _ = object_image.shape
        object_min_x_pos = object_location[0]
        object_max_x_pos = object_location[0] + object_width
        object_min_y_pos = object_location[1]
        object_max_y_pos = object_location[1] + object_height

        # Get the depth of the pixels within the bounds of the object image
        object_depth = depth_map[object_min_y_pos:object_max_y_pos,
                                 object_min_x_pos:object_max_x_pos]

        # Find the closest point in this rectangle
        closest_point_depth = object_depth.min()

        # Create the object bounding box
        object_bbox = patches.Rectangle((object_min_x_pos, object_min_y_pos), object_width, object_height,
                                        linewidth=1, edgecolor='r', facecolor='none')

        return closest_point_depth, object_bbox

    def locate_object(self, image, object_image):
        # Run the template matching from OpenCV
        cross_corr_map = cv2.matchTemplate(image,
                                           object_image,
                                           method=cv2.TM_CCOEFF)

        # Locate the position of the object using the minMaxLoc function from OpenCV
        *_, object_location = cv2.minMaxLoc(cross_corr_map)
        return cross_corr_map, object_location

    def display_image(self, img1, img2=[], patch=None):
        if (len(img2) > 0):
            fig, ax = plt.subplots(1, 2, figsize=(10, 10))
            ax[0].imshow(img1)
            ax[1].imshow(img2)

            if patch:
                ax[0].add_patch(patch)
        else:
            plt.imshow(img1)
            if patch:
                plt.axes().add_patch(patch)

        plt.show()

    def estimate(self, img_left, img_right, object_image):
        """
        ***** Depth Estimation Steps *****
        1. Determine the disparity between the two images.
        2. Decompose the projection matrices into the camera intrinsic matrix  𝐾, and extrinsics  𝑅,  𝑡s.
        3. Estimate depth using what we've gathered in the two prior steps.
        """
        # Display images
        self.display_image(img_left, img_right)

        # Step 1. Estimating disparity map
        disparity_map = self.compute_disparity_map(img_left, img_right)

        # Display disparity map
        self.display_disparity_map(disparity_map)

        # Step 2. Decompose projection matrix
        K_left, R_left, t_left = self.decompose_projection_mat(
            self.projection_mat_left)
        K_right, R_right, t_right = self.decompose_projection_mat(
            self.projection_mat_right)

        # Step 3. Calculate depth map
        depth_map = self.compute_depth_map(
            disparity_map, K_left, t_left, t_right)

        # Display depth map
        self.display_depth_map(depth_map)

        # Step 4. Finding the distance to pick
        cross_corr_map, object_location = self.locate_object(
            img_left, object_image)
        closest_point_depth, object_bbox = self.calculate_nearest_point(
            depth_map, object_location, object_image)

        # Display cross correlation map
        self.display_image(img_left, cross_corr_map)

        print(
            "\nObstacle Location (left-top corner coordinates):\n {0}".format(list(object_location)))

        print("\nClosest point depth (meters):\n {0}".format(
            closest_point_depth))

    def display_depth_map(self, depth_map):
        # Display the depth map
        plt.figure(figsize=(8, 8), dpi=100)
        plt.imshow(depth_map, cmap='flag')
        plt.show()

    def display_disparity_map(self, disparity_map):
        # Show the left disparity map
        plt.figure(figsize=(10, 10))
        plt.imshow(disparity_map)
        plt.show()

    def get_way_points_depth(self, image_l, image_r, way_points):
       
        depths = []
        for waypoint in way_points:
            depths.append(self.depth_map[waypoint])

        return depths
    def get_point_depth(self,image_1,image_r,point):
        return self.depth_map[point]
