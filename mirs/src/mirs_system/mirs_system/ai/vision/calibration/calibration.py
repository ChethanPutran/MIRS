import cv2
import numpy as np
import glob


class Calibration:
    def __init__(self) -> None:
        self.chess_board_size = (9,9)
        self.frame_size = (640,640)
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,30,0.001)
        self.camera_matrix = []
        self.distorsion_params = []
        self.rotation_matrix = []
        self.translational_vector = []

    def calibrate(self):

        object_points = np.zeros((self.chess_board_size[0]*self.chess_board_size[1],3),dtype=np.float32)
        object_points[:,:2] = np.meshgrid[0:self.chess_board_size[0],0:self.chess_board_size[1]].T.reshape(-1,2)

        real_object_points = []
        image_points = []

        # Loading images
        images = glob.glob("./image/*.png")

        for image in images:
            img = cv2.imread(image)
            img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(img_gray,self.chess_board_size,None)

            if ret:
                real_object_points.append(object_points)
                corners2 = cv2.cornerSubPix(img_gray,corners,(11,11),(-1,-1),self.criteria)
                image_points.append(corners)

                cv2.drawChessboardCorners(img,self.chess_board_size,corners2,ret)
                cv2.imshow('img',img)
                cv2.waitKey(1000)
        
        cv2.destroyAllWindows()

        ret,camera_matrix,distorsion_params,rotation_matrix,translational_vector = cv2.calibrateCamera(real_object_points,
                                                                                    image_points,
                                                                                    self.frame_size,
                                                                                    None,
                                                                                    None
        )
        self.camera_matrix = camera_matrix
        self.distorsion_params = distorsion_params
        self.rotation_matrix = rotation_matrix
        self.translational_vector = translational_vector

        print("Camera calibration sucessful.\n")
        print("Camera Matrix : ",camera_matrix)
        print("Rotation Matrix : ",rotation_matrix)
        print("Translational Vector :",translational_vector)
        print("Distorsion Parameters : ",distorsion_params)

        print("Saving camera parameters...")
        np.savez("CameraParams",camera_matrix=camera_matrix,
                 distorsion_params=distorsion_params,
                 rotation_matrix=rotation_matrix,
                 translational_vector=translational_vector
                 )


    def undistort(self,img):
        img = cv2.imread("/images/1.png")
        h,w = img.shape[:2]
        new_cam_mat,roi = cv2.getOptimalNewCameraMatrix(cameraMatrix=self.camera_matrix,
                                                        distCoeffs=self.distorsion_params,
                                                        )



