import numpy as np
import cv2 
from rclpy.node import Node
import io
import socket
import struct
from PIL import Image
import matplotlib.pyplot as pl

class Connection:
  def __init__(self,host='raspberrypi.local',port=8000):
    self.client_socket = socket.socket()
    self.client_socket.connect((host, port))
    self.connection = self.client_socket.makefile('wb')
  
  def getImageArray(self):
      while True:
        image_len = struct.unpack('<L', self.connection.read(struct.calcsize('<L')))[0]
        if not image_len:
            break
        # Construct a stream to hold the image data and read the image
        # data from the connection
        image_stream = io.BytesIO()
        image_stream.write(self.connection.read(image_len))
        # Rewind the stream, open it as an image with PIL and do some
        # processing on it
        image_stream.seek(0)
        image = Image.open(image_stream)
        
        if img is None:
            img = pl.imshow(image)
        else:
            img.set_data(image)

        pl.pause(0.01)
        pl.draw()

        print('Image is %dx%d' % image.size)
        image.verify()
        print('Image is verified')
        return image

class Camera(Node):
  def __init__(self):
    super().__init__('Camera')
    self.camera = None #Connection()

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
