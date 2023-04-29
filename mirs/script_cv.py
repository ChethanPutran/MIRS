from picamera2 import Picamera2
import cv2
import time


def record(self):

size = (640,480)
format = "RGB888"
FPS = 20.0
tm = str(int(time.time()))

fname_l = "left_"+tm+".avi"
fname_r = "right_"+tm+".avi"


fourcc = cv2.VideoWriter_fourcc(*'XVID')
out_l = cv2.VideoWriter(fname_l, fourcc, FPS, size)
out_r = cv2.VideoWriter(fname_r, fourcc, FPS, size)

RECORD_TIME = 10 #s


cam_left = Picamera2(0)
time.sleep(2)
video_config = cam_left.create_video_configuration(main={"size": size,"format":format})
cam_left.configure(video_config)

cam_right = Picamera2(1)
time.sleep(2)
video_config = cam_right.create_video_configuration(main={"size":size,"format":format})
cam_right.configure(video_config)

start = time.time()
end =  start + RECORD_TIME


cam_left.start()
cam_right.start()


while (time.time()<end):
    frame_l = cam_left.capture_array()
    frame_r = cam_right.capture_array()
    print(frame_l.shape,frame_r.shape)
    out_r.write(frame_r)
    out_l.write(frame_l)


out_l.release()
out_r.release()


