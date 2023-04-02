import numpy as np
import cv2 

class Camera:
  def __init__(self,supervisor):
    self.supervisor = supervisor
    self.recognition_object = {
   'id':0, 
   'position' : [],
  'orientation': [4],
  'size' : [2],
  'position_on_image':[],
  'size_on_image':[],
  'number_of_colors':0,
  'colors':[],
  'model' : ''
    }
    self.camera = self.supervisor.getCamera()
    self.camera.enable(self.supervisor.timestep)


  def get_image_from_camera(self):
      img = self.camera.getImageArray()
      img = np.asarray(img, dtype=np.uint8)
      img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
      img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
      return cv2.flip(img, 1)
    
  def enable(self,sampling_period):
    pass

  def segment(self):
    img = self.get_image_from_camera()

    # Segment the image by color in HSV color space
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(img, np.array([50, 150, 0]), np.array([200, 230, 255]))

    # Find the largest segmented contour (red ball) and it's center
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    largest_contour = max(contours, key=cv2.contourArea)
    largest_contour_center = cv2.moments(largest_contour)
    center_x = int(largest_contour_center['m10'] / largest_contour_center['m00'])

    # Find error (ball distance from image center)
    error = self.camera.getWidth() / 2 - center_x


  def disable(self):
    pass
  def get_sampling_period(self):
    pass
  def get_image(self):
    pass
  def  get_width(self):
    pass
  def  get_height(self):
    pass
  def get_fov(self):
    pass
  def get_max_fov(self):
    pass
  def get_min_fov(self):
    pass
  def set_fov(self, fov):  #fov specified in rad
    pass
  def get_exposure(self):
    pass
  def  set_exposure(self, exposure):
    pass
  def get_focal_length(self):
    pass
  def get_focal_distance(self):
    pass
  def get_max_focal_distance(self):
    pass
  def get_min_focal_distance(self):
    pass
  def  set_focal_distance(self, focal_distance):
    pass
  def get_near(self):
    pass
  def save_image(self,filename, quality):
    pass
  def has_recognition(self):
    pass
  def  recognition_enable(self, sampling_period):
    pass
  def  recognition_disable(self):
    pass
  def  recognition_get_sampling_period(self):
    pass
  def  recognition_get_number_of_objects(self):
    pass
  def get_objects(self):
    pass
  def has_segmentation(self):
    pass
  def  recognition_enable_segmentation(self):
    pass
  def  recognition_disable_segmentation(self):
    pass
  def is_segmentation_enabled(self):
    pass
  def get_segmentation_image(self):
    pass
  def save_segmentation_image(self,filename,quality):
    pass
  def get_object(self,index):
    pass
  def get_red(self,image, width, x, y):
    pass
  def get_green(self,image, width, x, y):
    pass
  def get_blue(self,image, width, x, y):
    pass
  def get_grey(self,image, width, x, y):
    pass
