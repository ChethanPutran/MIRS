import cv2
import numpy as np
import glob
from mirs_system.ai.vision.calibration.calibration import Calibration

class PoseEstimator(object):
    def __init__(self):
            self.camera_mat1,self.dist_param1,self.camera_mat2,self.dist_param2,self.rot_mat,self.trans_vec = Calibration.load_stereo_params()
    def draw_line_for_corners(img,corners,img_points):
        corner = tuple(corners[0].ravel())
        img = cv2.line(img,corner,tuple(img_points[0].ravel()),(255,0,0),10)
        img = cv2.line(img,corner,tuple(img_points[1].ravel()),(0,255,0),10)
        img = cv2.line(img,corner,tuple(img_points[2].ravel()),(0,0,255),10)

        return img
    
    def draw_box(img,corners,img_points):
        img_points = np.int32(img_points).reshape(-1,2)

        # Draw ground floor
        img = cv2.drawContours(img,[img_points[:4]],-1,(0,255,0),-3)

        # Draw pillars
        for i,j in zip(range(4),range(4,8)):
            img = cv2.line(img,tuple(img_points[i]),tuple(img_points[j]),(255),3)
        
        # Draw top layer
        img = cv2.drawContours(img,[img_points[4:]],-1,(0,0,255),3)

        return img
    
    def estimate(self):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,30,0.001)
        objp = np.zeros((9*9,3),np.float32)
        objp[:,:2] = np.meshgrid[0:9,0:9].T.reshape(-1,2)
        axis = np.float32([[3,0,0],[0,3,0],[0,0,-3]]).reshape(-1,3)
        axis_boxes = np.float32([[0,0,0],[0,3,0],
                                [3,3,0],[3,0,0],
                                [0,0,-3],[0,3,-3],
                                [3,3,-3],[3,0,-3]])
        # Loading images
        images = glob.glob("./image/*.png")
        chessboard_size = (9,9)

        for image in images:
            img = cv2.imread(image)
            img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(img_gray,chessboard_size,None)

            if ret:
                corners2 = cv2.cornerSubPix(img_gray,corners,(11,11),(-1,-1),criteria)

                #Find rotation and translation vector
                ret,rot_mat,trans_vec = cv2.solvePnP(objp,corners2,self.camera_mat,self.dist_param)


                # For drawing box
                # Project 3D points to image plane
                img_points,jac = cv2.projectPoints(axis_boxes,
                                                   rot_mat,
                                                   trans_vec,
                                                   self.camera_mat,
                                                   self.dist_param)
                img = self.draw_box(img,corners2,img_points)


                # For drawing frame
                # # Project 3D points to image plane
                # img_points,jac = cv2.projectPoints(axis,
                #                                    rot_mat,
                #                                    trans_vec,
                #                                    self.camera_mat,
                #                                    self.dist_param)
                #img = self.draw_line_for_corners(img,corners2,img_points)

                cv2.imshow('img',img)

                if(cv2.waitKey(0) == ord('q')):
                    cv2.imwrite('pose',img)
        
        cv2.destroyAllWindows()
        pass


    def get_pose(self,object,position,img_left,img_right):
        [center_x, center_y, x, y, w, h] = position
        pass

