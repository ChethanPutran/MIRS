import sys
import os
import cv2
import numpy as np
import glob
import matplotlib.pyplot as plt
from mirs_system.ai.vision.calibration.raspberrypi import Raspberrypi
import time
from scipy import linalg
import yaml



#calibration pattern settings
CHECK_BOARD_ROWS = 7
CHECK_BOARD_COLUMNS = 7
CHECK_BOARD_BOX_SIZE = 2.25 
CRITERIA =(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
WIDTH =  640
HEIGHT = 480
#CRITERIA = (cv2.TERM_CRITERIA_EPS +cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


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
    stereo_data_path = os.path.join(dir_path,"camera_parameters","stereo.npz")

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

        
        self.left_imgs = glob.glob(os.path.join(dir_path,"stereo","right-*.jpg"))
        self.right_imgs = glob.glob(os.path.join(dir_path,"stereo","left-*.jpg"))
        
    def calibrate(self,images_path,fname,show=True):

        #coordinates of squares in the checkerboard world space
        objp = np.zeros((CHECK_BOARD_ROWS*CHECK_BOARD_COLUMNS,3), np.float32)
        objp[:,:2] = np.mgrid[0:CHECK_BOARD_ROWS,0:CHECK_BOARD_COLUMNS].T.reshape(-1,2)
        objp = CHECK_BOARD_BOX_SIZE* objp


        #Pixel coordinates of checkerboards
        imgpoints = [] # 2d points in image plane.

        #coordinates of the checkerboard in checkerboard world space.
        objpoints = [] # 3d point in real world space
    

        for img_path in images_path:
            frame = cv2.imread(img_path)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            #find the checkerboard
            ret, corners = cv2.findChessboardCorners(gray, (CHECK_BOARD_ROWS, CHECK_BOARD_COLUMNS), cv2.CALIB_CB_ADAPTIVE_THRESH
                                + cv2.CALIB_CB_FAST_CHECK +
                                cv2.CALIB_CB_NORMALIZE_IMAGE)
            #ret, corners = cv2.findChessboardCorners(gray, (CHECK_BOARD_ROWS, CHECK_BOARD_COLUMNS), None)

            if ret == True:

                #Convolution size used to improve corner detection. Don't make this too large.
                conv_size = (11, 11)

                #opencv can attempt to improve the checkerboard coordinates
                corners = cv2.cornerSubPix(gray, corners, conv_size, (-1, -1), CRITERIA)

                # Draw and display the corners
                cv2.drawChessboardCorners(frame, (CHECK_BOARD_ROWS,CHECK_BOARD_COLUMNS), corners, ret)
                cv2.imwrite(os.path.join(dir_path,CALIBRATION_RESULTS,f"{str(int(time.time()))}.jpg"),frame)

                if show:
                    cv2.imshow('img', frame)

                    if cv2.waitKey(0) & 0xFF == ord('s'):
                        print('skipping')
                        continue

                objpoints.append(objp)
                imgpoints.append(corners)

        if show:   
            cv2.destroyAllWindows()

        ret, cmtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (WIDTH, HEIGHT), None, None)
     

    
        self.save_camera_params(fname,camera_matrix=cmtx,
                distorsion_params=dist,
                rotation_matrix=rvecs,
                translational_vector=tvecs
                )
        return cmtx, dist, rvecs, tvecs

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

    @staticmethod
    def load_stereo_params():
        data = np.load(Calibration.stereo_data_path)
        return data['camera_left_matrix'],\
            data['distorsion_left'],\
            data['camera_right_matrix'],\
            data['distorsion_right'],\
            data['rotation_matrix'],\
            data['translational_vector']

    def calibrate_stereo_camera(self,get_new_images = False,show=True):
        try:
            print("Starting stereo calibration...")

            if not get_new_images:
                print("Calibrating with default images...")
                left_imgs_path = self.left_imgs 
                right_imgs_path = self.right_imgs 
            else:
                print("Calibrating with new images...")
                rasp = Raspberrypi()
                files,err = rasp.get_calibration_images(Calibration.WIDTH,Calibration.HEIGHT)

                if not err:
                    left_imgs_path =  [files[0],]
                    right_imgs_path = [files[1],]

                else:
                    print("ERROR : ",err)
                    return self.calibrate_stereo_camera(False)
                
            cmtx_l,dist_l,rvec_l,tvec_l = self.calibrate(left_imgs_path,Calibration.camera_mat_left,show=show)
            cmtx_r,dist_r,rvec_r,tvec_r  = self.calibrate(right_imgs_path,Calibration.camera_mat_right,show=show)

            
            CM1,dist0,CM2,dist1,R, T = self.stereo_calibrate(cmtx_l, dist_l, cmtx_r, dist_r, left_imgs_path, right_imgs_path,show=show)

            print("Stereo calibration sucessful.")

            print("Saving stereo camera params...")
            np.savez(self.stereo_data_path,
                    camera_left_matrix=CM1,
                    camera_right_matrix=CM2,
                    distorsion_left=dist0,
                    distorsion_right=dist1,
                    rotation_matrix=R,
                    translational_vector=T)
            print("Saved.")

            return (CM1,dist0,CM2,dist1,R, T), None
        
        except Exception as e:

            print("Stereo calibration failed!")

            return None, e
    
    def undistort(self,img):
        img = cv2.imread("/images/1.png")
        h,w = img.shape[:2]
        new_cam_mat,roi = cv2.getOptimalNewCameraMatrix(cameraMatrix=self.camera_matrix,
                                                        distCoeffs=self.distorsion_params,
                                                        )
        
    #Given Projection matrices P1 and P2, and pixel coordinates point1 and point2, return triangulated 3D point.
    def DLT(self, P1, P2, point1, point2):

        A = [point1[1]*P1[2,:] - P1[1,:],
            P1[0,:] - point1[0]*P1[2,:],
            point2[1]*P2[2,:] - P2[1,:],
            P2[0,:] - point2[0]*P2[2,:]
            ]
        A = np.array(A).reshape((4,4))

        B = A.transpose() @ A
        U, s, Vh = linalg.svd(B, full_matrices = False)

        #print('Triangulated point: ')
        #print(Vh[3,0:3]/Vh[3,3])
        return Vh[3,0:3]/Vh[3,3]

    #open paired calibration frames and stereo calibrate for cam0 to cam1 coorindate transformations
    def stereo_calibrate(self,mtx0, dist0, mtx1, dist1, left_img_paths, right_img_paths,width=640,height=480,show=False):
        #coordinates of squares in the checkerboard world space
        objp = np.zeros((CHECK_BOARD_ROWS*CHECK_BOARD_COLUMNS,3), np.float32)
        objp[:,:2] = np.mgrid[0:CHECK_BOARD_ROWS,0:CHECK_BOARD_COLUMNS].T.reshape(-1,2)
        objp = CHECK_BOARD_BOX_SIZE* objp


        #Pixel coordinates of checkerboards
        imgpoints_left = [] # 2d points in image plane.
        imgpoints_right = []

        #coordinates of the checkerboard in checkerboard world space.
        objpoints = [] # 3d point in real world space

        for frame_l_pth,frame_r_pth in zip(left_img_paths,right_img_paths):
            frame_l = cv2.imread(frame_l_pth)
            frame_r = cv2.imread(frame_r_pth)

            gray1 = cv2.cvtColor(frame_l, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)

            c_ret1, corners1 = cv2.findChessboardCorners(gray1, (CHECK_BOARD_ROWS, CHECK_BOARD_COLUMNS), None)
            c_ret2, corners2 = cv2.findChessboardCorners(gray2, (CHECK_BOARD_ROWS, CHECK_BOARD_COLUMNS), None)

            if c_ret1 == True and c_ret2 == True:

                corners1 = cv2.cornerSubPix(gray1, corners1, (11, 11), (-1, -1), CRITERIA)
                corners2 = cv2.cornerSubPix(gray2, corners2, (11, 11), (-1, -1), CRITERIA)

                p0_c1 = corners1[0,0].astype(np.int32)
                p0_c2 = corners2[0,0].astype(np.int32)

                if show:

                    cv2.putText(frame_l, 'O', (p0_c1[0], p0_c1[1]), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 1)
                    cv2.drawChessboardCorners(frame_l, (CHECK_BOARD_ROWS,CHECK_BOARD_COLUMNS), corners1, c_ret1)
                    cv2.imshow('img_left', frame_l)

                    cv2.putText(frame_r, 'O', (p0_c2[0], p0_c2[1]), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 1)
                    cv2.drawChessboardCorners(frame_r, (CHECK_BOARD_ROWS,CHECK_BOARD_COLUMNS), corners2, c_ret2)
                    cv2.imshow('img_right', frame_r)
                    k = cv2.waitKey(0)

                    if k & 0xFF == ord('s'):
                        print('skipping')
                        continue

                objpoints.append(objp)
                imgpoints_left.append(corners1)
                imgpoints_right.append(corners2)

        stereocalibration_flags = cv2.CALIB_FIX_INTRINSIC
        ret, CM1, dist0, CM2, dist1, R, T, E, F = cv2.stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, mtx0, dist0,
                                                                    mtx1, dist1, (width, height), criteria = CRITERIA, flags = stereocalibration_flags)


        if show:
            cv2.destroyAllWindows()
            
        return CM1,dist0,CM2,dist1,R, T

    #Converts Rotation matrix R and Translation vector T into a homogeneous representation matrix
    def make_homogeneous_rep_matrix(self,R, t):
        P = np.zeros((4,4))
        P[:3,:3] = R
        P[:3, 3] = t.reshape(3)
        P[3,3] = 1
        return P
    
    # Turn camera calibration data into projection matrix
    def get_projection_matrix(self,cmtx, R, T):
        P = cmtx @ self.make_homogeneous_rep_matrix(R, T)[:3,:]
        return P

    # After calibrating, we can see shifted coordinate axes in the video feeds directly
    def check_calibration(self,camera_l_data, camera_r_data, _zshift = 50.):
        
        cmtx0 = np.array(camera_l_data[0])
        dist0 = np.array(camera_l_data[1])
        R0 = np.array(camera_l_data[2])
        T0 = np.array(camera_l_data[3])
        cmtx1 = np.array(camera_r_data[0])
        dist1 = np.array(camera_r_data[1])
        R1 = np.array(camera_r_data[2])
        T1 = np.array(camera_r_data[3])

        P0 = self.get_projection_matrix(cmtx0, R0, T0)
        P1 = self.get_projection_matrix(cmtx1, R1, T1)

        #define coordinate axes in 3D space. These are just the usual coorindate vectors
        coordinate_points = np.array([[0.,0.,0.],
                                    [1.,0.,0.],
                                    [0.,1.,0.],
                                    [0.,0.,1.]])
        z_shift = np.array([0.,0.,_zshift]).reshape((1, 3))
        #increase the size of the coorindate axes and shift in the z direction
        draw_axes_points = 5 * coordinate_points + z_shift

        #project 3D points to each camera view manually. This can also be done using cv2.projectPoints()
        #Note that this uses homogenous coordinate formulation
        pixel_points_camera0 = []
        pixel_points_camera1 = []
        for _p in draw_axes_points:
            X = np.array([_p[0], _p[1], _p[2], 1.])
            
            #project to camera0
            uv = P0 @ X
            uv = np.array([uv[0], uv[1]])/uv[2]
            pixel_points_camera0.append(uv)

            #project to camera1
            uv = P1 @ X
            uv = np.array([uv[0], uv[1]])/uv[2]
            pixel_points_camera1.append(uv)

        #these contain the pixel coorindates in each camera view as: (pxl_x, pxl_y)
        pixel_points_camera0 = np.array(pixel_points_camera0)
        pixel_points_camera1 = np.array(pixel_points_camera1)

        #open the video streams
        cap0 = cv2.VideoCapture(0)
        cap1 = cv2.VideoCapture(1)

        #set camera resolutions
        cap0.set(3, WIDTH)
        cap0.set(4, HEIGHT)
        cap1.set(3, WIDTH)
        cap1.set(4, HEIGHT)

        while True:

            ret0, frame0 = cap0.read()
            ret1, frame1 = cap1.read()

            if not ret0 or not ret1:
                print('Video stream not returning frame data')
                quit()

            #follow RGB colors to indicate XYZ axes respectively
            colors = [(0,0,255), (0,255,0), (255,0,0)]
            #draw projections to camera0
            origin = tuple(pixel_points_camera0[0].astype(np.int32))
            for col, _p in zip(colors, pixel_points_camera0[1:]):
                _p = tuple(_p.astype(np.int32))
                cv2.line(frame0, origin, _p, col, 2)
            
            #draw projections to camera1
            origin = tuple(pixel_points_camera1[0].astype(np.int32))
            for col, _p in zip(colors, pixel_points_camera1[1:]):
                _p = tuple(_p.astype(np.int32))
                cv2.line(frame1, origin, _p, col, 2)

            cv2.imshow('frame0', frame0)
            cv2.imshow('frame1', frame1)

            k = cv2.waitKey(1)
            if k == 27: break

        cv2.destroyAllWindows()

    def get_world_space_origin(self,cmtx, dist, img_path,show=True):

        frame = cv2.imread(img_path, 1)

        #calibration pattern settings

        #coordinates of squares in the checkerboard world space
        objp = np.zeros((CHECK_BOARD_ROWS*CHECK_BOARD_COLUMNS,3), np.float32)
        objp[:,:2] = np.mgrid[0:CHECK_BOARD_ROWS,0:CHECK_BOARD_COLUMNS].T.reshape(-1,2)
        objp = CHECK_BOARD_BOX_SIZE* objp

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (CHECK_BOARD_ROWS, CHECK_BOARD_COLUMNS), None)

        if show:
            cv2.drawChessboardCorners(frame, (CHECK_BOARD_ROWS,CHECK_BOARD_COLUMNS), corners, ret)
            cv2.imshow('img', frame)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        ret, rvec, tvec = cv2.solvePnP(objp, corners, cmtx, dist)
        R, _  = cv2.Rodrigues(rvec) #rvec is Rotation matrix in Rodrigues vector form

        return R, tvec

    def get_cam_left_to_world_transforms(self,cmtx0, dist0, R_WL, T_WL, 
                                    cmtx1, dist1, R_LR, T_LR,
                                    image_path0,
                                    image_path1,show=True):

        frame0 = cv2.imread(image_path0, 1)
        frame1 = cv2.imread(image_path1, 1)

        unitv_points = 5 * np.array([[0,0,0], [1,0,0], [0,1,0], [0,0,1]], dtype = 'float32').reshape((4,1,3))
        #axes colors are RGB format to indicate XYZ axes.
        colors = [(0,0,255), (0,255,0), (255,0,0)]

        #project origin points to frame 0
        points, _ = cv2.projectPoints(unitv_points, R_WL, T_WL, cmtx0, dist0)
        points = points.reshape((4,2)).astype(np.int32)
        origin = tuple(points[0])
        for col, _p in zip(colors, points[1:]):
            _p = tuple(_p.astype(np.int32))
            cv2.line(frame0, origin, _p, col, 2)

        #project origin points to frame1 (post multiply)
        R_WR= R_LR @ R_WL
        T_WR = R_LR @ T_WL + T_LR

        points, _ = cv2.projectPoints(unitv_points, R_WR, T_WR, cmtx1, dist1)

        points = points.reshape((4,2)).astype(np.int32)
        origin = tuple(points[0])
        for col, _p in zip(colors, points[1:]):
            _p = tuple(_p.astype(np.int32))
            cv2.line(frame1, origin, _p, col, 2)

        if show:
            cv2.imshow('frame0', frame0)
            cv2.imshow('frame1', frame1)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return R_WR, T_WR



if __name__ == "__main__":
    cc = Calibration()

    
    """Camera_left defines the world space origin."""
    # camera0_leftrotation and translation is identity matrix and zeros vector
    R0 = np.eye(3, dtype=np.float32)
    T0 = np.array([0., 0., 0.]).reshape((3, 1))


    "Use paired calibration pattern frames to obtain camera_left to camera_right rotation and translation"
    (data,error,) = cc.calibrate_stereo_camera(show=False)

    if not error:

        # cmtx_l,dist_l,*_ = cc.load_camera_params()
        # cmtx_r,dist_r,*_ = cc.load_camera_params()
       
        cmtx_l,dist_l,cmtx_r,dist_r,R_LR, T_LR = data

        camera_left_data = [cmtx_l, dist_l, R0, T0]
        camera_right_data = [cmtx_r, dist_r, R0, T0]

        #cc.check_calibration('camera_l', camera_left_data, 'camera1', camera_right_data, _zshift = 60.)

        """Define a different origin point (previous origin point was left camera)"""

        # Get the world to camera_left rotation and translation
        R_WL, T_WL = cc.get_world_space_origin(cmtx_l, dist_l,cc.left_imgs[0],show=False)


        # Get rotation and translation from world directly to camera_right
        R_WR, T_WR = cc.get_cam_left_to_world_transforms(cmtx_l, dist_l, R_WL, T_WL,
                                                cmtx_r, dist_r, R_LR, T_LR,
                                                cc.left_imgs[0],
                                                cc.right_imgs[0],)
        
        print("New transformations (with diff world origin point) :")
        print("R_WL : ",R_WL)
        print("T_WL : ",T_WL)
        print("R_WR : ",R_WR)
        print("T_WR : ",T_WR)
    
    else:
        print("ERROR :",error)