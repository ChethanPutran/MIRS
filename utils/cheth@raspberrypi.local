from picamera2 import Picamera2
import cv2
import time
import argparse


# Initialize parser
parser = argparse.ArgumentParser()

# Adding optional argument
parser.add_argument("-r", "--RecordTime", help = "Record Time in seconds")
parser.add_argument("-w", "--Width", help = "Width in pixels")
parser.add_argument("-hi", "--Height", help = "Height in pixels")
parser.add_argument("-f", "--FileName", help = "File name of the video")
parser.add_argument("-fp", "--Fps", help = "Frame per seconds")

# Read arguments from command line
args = parser.parse_args()


RECORD_TIME = 10 #s
WIDTH = 640
HEIGHT = 480
FPS = 30.0
FileName = ''

if args.RecordTime:
    RECORD_TIME = float(args.RecordTime)
if args.Width:
    WIDTH = int(args.Width)
if args.Height:
    HEIGHT = int(args.Height)
if args.FileName:
    FileName = str(args.FileName)
if args.Fps:
    FPS = int(args.Fps)

sterio = True
size = (HEIGHT,WIDTH)

cam_left = Picamera2(0)
time.sleep(2)

fdl = (33333,33333)
if FPS == 20:
    fdl = (40000,40000)
elif FPS == 10:
    fdl = (100000,100000)

video_config = cam_left.create_video_configuration(main={"size": size,"format":"RGB888"},controls={"FrameDurationLimits":fdl
})
cam_left.configure(video_config)

tm = str(int(time.time()))

fname_l = f"left_{FileName}.avi"
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out_l = cv2.VideoWriter(fname_l, fourcc, FPS, size)


cam_right = Picamera2(1)
time.sleep(2)
video_config = cam_right.create_video_configuration(main={"size":size,"format":"RGB888"})
cam_right.configure(video_config)
fname_r = f"right_{FileName}.avi"
out_r = cv2.VideoWriter(fname_r, fourcc, FPS, size)


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
