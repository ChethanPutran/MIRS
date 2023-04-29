import os
import cv2
import numpy as np
import glob
import matplotlib.pyplot as plt
from .raspberrypi import Raspberrypi
import time

CALIBRATION_IMAGES = "stereo"  
CALIBRATION_RESULTS = os.path.join("stereo","results")
dir_path = os.path.dirname(__file__)



def get_cube_surface_points(x=9,y=9,z=9,num_points=5):
    cube_points = []

    # Z = 0 (Z-Plane)
    k = 0
    for i in np.linspace(0,x,num_points,endpoint=True):
        for j in np.linspace(0,y,num_points,endpoint=True):
            cube_points.append((i,j,k))

    # X = 0 (X-Plane)
    i = 0
    for k in np.linspace(2.25,z,num_points-1,endpoint=True):
        for j in np.linspace(0,y,5,endpoint=True):
            cube_points.append((i,j,k))


    # Y = 0 (Y-Plane)
    j = 0
    for k in np.linspace(2.25,z,num_points-1,endpoint=True):
        for x in np.linspace(2.25,x,num_points-1,endpoint=True):
            cube_points.append((i,j,k))

    
    return  np.array(cube_points).astype(np.float32)

def ResizeWithAspectRatio(image, width=None, height=None, inter=cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]

    if width is None and height is None:
        return image,1
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))

    return cv2.resize(image, dim, interpolation=inter),r

def get_2D_image_points(image):
    points = []
    def click_event(event, x, y, flags, params):
        
        # checking for left mouse clicks
        if event == cv2.EVENT_LBUTTONDOWN:

            # displaying the coordinates
            # on the Shell
            points.append((x/aspect_ratio, y/aspect_ratio))

            # displaying the coordinates
            # on the image window
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.circle(resized_img,(x,y),3,(0, 255, 0),5)
            plt.imshow(resized_img)

        # checking for right mouse clicks	
        if event==cv2.EVENT_RBUTTONDOWN:

            # displaying the coordinates
            # on the Shell
            print(x, ' ', y)

            # displaying the coordinates
            # on the image window
            font = cv2.FONT_HERSHEY_SIMPLEX
            b = resized_img[y, x, 0]
            g = resized_img[y, x, 1]
            r = resized_img[y, x, 2]
            cv2.putText(resized_img, str(b) + ',' +
                        str(g) + ',' + str(r),
                        (x,y), font, 1,
                        (255, 255, 0), 2)
            plt.imshow(resized_img)

    resize_width = 820
    resized_img,aspect_ratio = ResizeWithAspectRatio(image, width=resize_width) # Resize by width 

    plt.imshow( resized_img)

    # setting mouse handler for the image
    # and calling the click_event() function
    cv2.setMouseCallback('image', click_event)

    # wait for a key to be pressed to exit
    cv2.waitKey(0)

    # close the window
    cv2.destroyAllWindows()

    return points

class Calibration:
    WIDTH = 640
    HEIGHT = 480
    focal_length = 2.6  # in mm
    baseline = 60   # in mm
    u0 = 2
    v0 = 1
    FOV_H = 73
    FOV_V = 50
    FOV_d = 83
    aperture = 2.4
    resolution = ( 3280 , 2464 )
    camera_left = 0
    camera_right = 1

    camera_mat_left = os.path.join(dir_path,"camera_parameters","camera_left.npz")
    camera_mat_right = os.path.join(dir_path,"camera_parameters","camera_right.npz")

    def __init__(self):
        self.chess_board_size = (5,5)
        

        # stop the iteration when specified
        # accuracy, epsilon, is reached or
        # specified number of iterations are completed.
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,30,0.001)

        self.camera_matrix = []
        self.distorsion_params = []
        self.rotation_matrix = []
        self.translational_vector = []
        self.n_points = 61
        
    def calibrate(self,images_path,fname):
        # Define the dimensions of checkerboard
        CHECKERBOARD = (7,7)

        # stop the iteration when specified
        # accuracy, epsilon, is reached or
        # specified number of iterations are completed.
        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Vector for 3D points
        threedpoints = []

        # Vector for 2D points
        twodpoints = []

        # 3D points real world coordinates
        objectp3d = np.zeros((1, CHECKERBOARD[0]
                            * CHECKERBOARD[1],
                            3), np.float32)
        objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0],
                                    0:CHECKERBOARD[1]].T.reshape(-1, 2)

        for filename in images_path:
            image = cv2.imread(filename)
            h, w = image.shape[:2]
            grayColor = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            # If desired number of corners are
            # found in the image then ret = true
            ret, corners = cv2.findChessboardCorners(
                            grayColor, CHECKERBOARD,
                            cv2.CALIB_CB_ADAPTIVE_THRESH
                            + cv2.CALIB_CB_FAST_CHECK +
                            cv2.CALIB_CB_NORMALIZE_IMAGE)

            # If desired number of corners can be detected then,
            # refine the pixel coordinates and display
            # them on the images of checker board
            if ret == True:
                threedpoints.append(objectp3d)

                # Refining pixel coordinates
                # for given 2d points.
                corners2 = cv2.cornerSubPix(
                    grayColor, corners, (11, 11), (-1, -1), criteria)

                twodpoints.append(corners2)

                # Draw and display the corners
                image = cv2.drawChessboardCorners(image,
                                                CHECKERBOARD,
                                                corners2, ret)
                cv2.imwrite(os.path.join(dir_path,CALIBRATION_RESULTS,f"{str(int(time.time()))}.jpg"),image)

            cv2.imshow('img', image)
            cv2.waitKey(0)

        cv2.destroyAllWindows()

        ret, camera_matrix, distortion, rot_matrix, t_vector = cv2.calibrateCamera(
            threedpoints, twodpoints, grayColor.shape[::-1], None, None)
        
        self.save_camera_params(fname,camera_matrix=camera_matrix,
                 distorsion_params=distortion,
                 rotation_matrix=rot_matrix,
                 translational_vector=t_vector
                 )
        return True

    def collect_points(self):
        object_points_3D = get_cube_surface_points(9,9,9)
        image_points = []

        # Loading images
        left_img_path = os.path.join(dir_path,"stereo\\")+"test-left-*.jpg"
        right_img_path = os.path.join(dir_path,"stereo\\")+"test-right-*.jpg"
      
        left_images = glob.glob(left_img_path)
        right_images = glob.glob(right_img_path)

        for idx,img_path in enumerate(left_images):
            img = cv2.imread(img_path)
            image_points = get_2D_image_points(img)
            np.savez(f"{dir_path}/points/points_data_left_camera_{idx}",x=object_points_3D,y=image_points)
            break
        
        for idx,img_path in enumerate(right_images):
            img = cv2.imread(img_path)
            image_points = get_2D_image_points(img)
            np.savez(f"{dir_path}\\points\\points_data_right_camera_{idx}",x=object_points_3D,y=image_points)
            break 

    @staticmethod
    def load_camera_params(right_cam=False):
        """ Default left camera params"""
        
        if not right_cam:
            data = np.load(Calibration.camera_mat_left)
            camera_matrix,distortion,rot_matrix,t_vector = data['camera_matrix'],data['distorsion_params'],data['rotation_matrix'],data['translational_vector']

        else:
            data = np.load(Calibration.camera_mat_left)
            camera_matrix,distortion,rot_matrix,t_vector = data['camera_matrix'],data['distorsion_params'],data['rotation_matrix'],data['translational_vector']

        return camera_matrix, distortion, rot_matrix, t_vector 
    
    def save_camera_params(self,fname,camera_matrix,distorsion_params,rotation_matrix,translational_vector):
        print("Saving camera params...")
        np.savez(fname,camera_matrix=camera_matrix,
                 distorsion_params=distorsion_params,
                 rotation_matrix=rotation_matrix,
                 translational_vector=translational_vector)
        print("Saved.")

    def stereo_calibrate(self,get_new_images = False):
        print("Starting stereo calibration...")

        if not get_new_images:
            print("Calibrating with default images...")
            left_imgs_path = glob.glob(os.path.join(dir_path,"stereo","left-*.jpg"))
            right_imgs_path = glob.glob(os.path.join(dir_path,"stereo","right-*.jpg"))
        else:
            print("Calibrating with new images...")
            rasp = Raspberrypi()
            files,err = rasp.get_calibration_images(Calibration.WIDTH,Calibration.HEIGHT)

            if not err:
                left_imgs_path =  [files[0],]
                right_imgs_path = [files[1],]

            else:
                print("ERROR : ",err)
                return self.stereo_calibrate(False)
            
        status_l = self.calibrate(left_imgs_path,Calibration.camera_mat_left)
        status_r = self.calibrate(right_imgs_path,Calibration.camera_mat_right)

        if status_l and status_r:
            print("Stereo calibration sucessful.")
        else:
            print("Stereo calibration failed!")
    
    def undistort(self,img):
        img = cv2.imread("/images/1.png")
        h,w = img.shape[:2]
        new_cam_mat,roi = cv2.getOptimalNewCameraMatrix(cameraMatrix=self.camera_matrix,
                                                        distCoeffs=self.distorsion_params,
                                                        )

if __name__ == "__main__":
    cc = Calibration()
    #cc.stereo_calibrate(False)
    print(cc.load_camera_params())
