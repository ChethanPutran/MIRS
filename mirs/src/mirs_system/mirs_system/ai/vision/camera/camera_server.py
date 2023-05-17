import socket
import pickle
import threading
import time
import struct
import numpy as np
import cv2 
from picamera2 import Picamera2
import cv2
import time

STERIO = True
DEBUG = True

class Connection:
  def __init__(self,host='raspberrypi.local',port=8000):
    self.host = host
    self.port = port
    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.HEADER_SIZE = 10

    #Binding socket
    self.bind_socket()

    #Accept connection
    self.accept_connection()

  def accept_connection(self):
    #Accepting the connection requrest
    print("Waiting for connection...")
    conn, addr = self.socket.accept()
    self.client = conn
    self.client_addr = addr
    print('\nConnection has been established : '+ str(self.client_addr[0]))
  
  def bind_socket(self):
    try:
        self.socket.bind((self.host, self.port))
        # Listening
        self.socket.listen(1)
        print("Server listening at ",self.host+":"+str(self.port))
    except socket.error as msg:
        print("Scoket Binding Error: "+str(msg)+"\n"+"Retrying...")
        self.bind_socket()
  
  def close(self):
        self.client.close()
        self.socket.close() 
        print("Exiting... Bye!")

  def send(self,message,data=None,error=None):
        if DEBUG:
            print("Sending...")
        res = {
                "message":message,
                "data":data,
                "error":error
            }
        
        res = pickle.dumps(res)
        res = bytes(f"{len(res):<{self.HEADER_SIZE}}",encoding="utf-8") + res
        self.client.sendall(res)

        if DEBUG:
            print("Sent.")

  def recieve(self):
    if DEBUG:
        print("Recieving...")
    new_msg = True
    req = b''
    msg_len = 0

    while True:
        msg = self.client.recv(16)

        if new_msg:
            msg_len = int(msg[:self.HEADER_SIZE])
            new_msg = False

        req += msg

        if(len(req)-self.HEADER_SIZE == msg_len):
            new_msg = True
            req = req[self.HEADER_SIZE:]
            break
    
    req = pickle.loads(req)
    if DEBUG:
        print("Recieved.")
    return req
  
  def send_live(self,status,frame):
    if not status:
        end = b'<END>'
        self.client.send(struct.pack("L", len(end))+end)
        time.sleep(3)
        self.send(message="Live ended")

    data = pickle.dumps(frame)
    self.client.sendall(struct.pack("L", len(data))+data)
                       

"""
Access camera of the robot
Get image one by one
Used to find the co-ordinate (x,y,z) of a point from the image
"""

class Camera(object):

    """ 
    ***** Calibration params (f, b, u0, v0) *****

    f - focal length
    b - base line
    u0 - x offset in image coordinate
    v0 - y offset in image coordinate

    """
    f = 2.6  # in mm
    b = 60   # in mm
    u0 = 2
    v0 = 1
    FOV_H = 73
    FOV_V = 50
    FOV_d = 83
    aperture = 2.4
    resolution = (3280 , 2464 )
    camera_left = 0
    camera_right = 1
    WIDTH = 640
    HEIGHT = 480
    FPS = 30.0

    def __init__(self):
        self.state = {
           "record":False,
           "recording":{
           "left":{
            "state":False,
            "fname":None
           },
           "right":{
            "state":False,
            "fname":None
           },
           },
           "active":False,
           "live":False
        }
        self.exit =False
        self.FPS = 20
        self.size = (self.HEIGHT,self.WIDTH)

        self.cam_left = Picamera2(0)
        time.sleep(2)

        self.fdl = (33333,33333)
        if self.FPS == 20:
            self.fdl = (40000,40000)
        elif self.FPS == 10:
            self.fdl = (100000,100000)

        video_config = self.cam_left.create_video_configuration(main={"size": self.size,"format":"RGB888"},
                                                                controls={"FrameDurationLimits":self.fdl})
        self.cam_left.configure(video_config)

        
        self.cam_right = Picamera2(1)
        time.sleep(2)
        video_config = self.cam_right.create_video_configuration(main={"size":self.size,"format":"RGB888"},
                                                            controls={"FrameDurationLimits":self.fdl})
        self.cam_right.configure(video_config)

    def start_recording(self):
        self.state['record'] = True
        self.exit = False
        self.record()

    def end_recording(self):
        self.exit = True

        if DEBUG:
            print("Exit updated!")

        # Wait untill recording get stopped
        while self.state['record']:
            time.sleep(1)

   
    def set_state(self,recording_left=None,recording_right=None,active=None):
        if recording_left:
            self.state['recording']['left'] = recording_left
        if recording_right:
            self.state['recording']['right'] = recording_right
        if not(active == None):
            self.state['active'] = active

    def get_state(self):
        return self.state
    
    def get_recording_state(self):
        return self.state['record']
    
    def record(self):
        try:
            FileName = str(int(time.time()))

            fname_l = f"left_{FileName}.avi"
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            out_l = cv2.VideoWriter(fname_l, fourcc, self.FPS, self.size)

            fname_r = f"right_{FileName}.avi"
            out_r = cv2.VideoWriter(fname_r, fourcc, self.FPS, self.size)

            self.cam_left.start()
            self.cam_right.start()

            while True:
                print("Exit :",self.exit)
                if self.exit:
                    print('Ending recording...')
                    break

                frame_l = self.cam_left.capture_array()
                frame_r = self.cam_right.capture_array()

                out_r.write(frame_r)
                out_l.write(frame_l)
            
                # img = self.camera.getImageArray()
                # img = np.asarray(img, dtype=np.uint8)
                # img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
                # img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
                # img = cv2.flip(img, 1)

            self.cam_left.stop()
            self.cam_right.stop()
          
            out_l.release()
            out_r.release()

            self.cam_left.close()
            self.cam_right.close()
                

            self.set_state({'state':True,
                             'fname':fname_l},
                                    {
                                        'state':True,
                                        'fname':fname_r
                                    })
            self.state['record'] = False

        except Exception as e:
            print("ERROR :",e)

    def get_recorded_file(self,sterio=STERIO):
        if not sterio:
            return self.state['recording']['left']['fname']
        else:
            return self.state['recording']['left']['fname'],self.state['recording']['right']['fname']

    def get_recording(self,sterio=STERIO):
        rec_file_name = self.get_recorded_file()
        print("Rec file :",rec_file_name)
        if not rec_file_name:
            return None,"No recording exits!"
    
        if not sterio:
            with open(rec_file_name, 'rb') as video:
                rec_l = video.read()
            return ((rec_file_name,rec_l),), None
        
        with open(rec_file_name[0], 'rb') as video:
            rec_l = video.read()
        
        with open(rec_file_name[1], 'rb') as video:
            rec_r = video.read()

        return ((rec_file_name[0],rec_l),(rec_file_name[1],rec_r),),None
    
    def live(self,callback):
        self.state['live'] = True
        cap = cv2.VideoCapture(0)

        while self.state['live']:
            _,img = cap.read()
            callback(True,img)
        callback(False,None)
       
    def start_live(self,camera_server):
        try:
            #Async recording
            t2 = threading.Thread(target=self.live,args=[camera_server.send_live,])
            t2.start()
            print("Live streaming started...")
        except Exception as e:
           print(e)
        
    def end_live(self):
        self.state['live'] = False

    def cannyImage(self, image):
        # 1.Conveting coloured image to grayscale image
        grayScaleImage = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        # 2.Reducing noise and smoothening image
        bluredGSImage = cv2.GaussianBlur(grayScaleImage, (5, 5), 0)

        # Determing the edges based on gradient(taking derivative(f(x,y)) x->along width of the message y-> along height)
        # (image,low_threshold,hight_threshold)
        canny = cv2.Canny(bluredGSImage, 50, 150)

        return canny
         
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

    def set_fps(self,fps):
      self.FPs = fps

    def get_fps(self):
      return self.FPS
    
    def get_width(self):
      pass
    
    def get_height(self):
       pass
  


def main():
    camera_server = Connection()
    camera= Camera()

    try:
        while True:
            # receive data stream. it won't accept data packet greater than 1024 bytes
            data = camera_server.recieve()
            print("Data : ",data)
            recording_status = camera.get_recording_state()
            
            if data['start_recording']:
                if recording_status:
                    camera_server.send("Error!",error="Alredy recording!")
                    continue

                #Async recording
                t1 = threading.Thread(target=camera.start_recording)
                t1.start()

                camera_server.send("Recording started.")

            elif data['end_recording']:
                try:
                    if not recording_status:
                        camera_server.send("Error!",error="Camera is not recording!")
                        continue

                    # End recording
                    camera.end_recording()
                    if not camera.get_recording_state():
                        camera_server.send("Recording ended.")
                    else:
                        camera_server.send("Error",error="Something went wrong! Coudn't stop recording.")
                except Exception as e:
                    raise e
            elif data['get_recording']:
                if recording_status:
                    error = "Can't send recording while camera is in recording state! Stop the recording first!"
                    camera_server.send("Error!",error=error)
                else:
                    recordings,error = camera.get_recording()
                    camera_server.send("Recordings",data=recordings)

            elif data['exit']:
                if recording_status:
                    camera_server.send("Error!",error="Can't stop camera while camera is in recording state! Stop the recording first!")
                else:
                    camera_server.send("Closing camera! Bye")
                    break

            elif data['start_live']:
                if camera.state['live'] == False:
                    camera.start_live(camera_server)
                elif recording_status:
                   camera_server.send("Error!",error="Can't start live streaming while camera is in recording state! Stop the recording first!")
                else:
                    camera_server.send("Error!",error="Already in live streaming!")

            elif data['end_live']:
                if camera.state['live'] == False:
                    camera_server.send("Error!",error="Not in live streaming!")
                else:
                    camera.end_live()
    except Exception as e:
       print('ERROR :',e)
       camera.exit = True
    camera_server.close()  # close the connection

if __name__ == '__main__':
  main()