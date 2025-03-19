import subprocess

video_len = 5000 #5s
width =  900
height =  900
try:

    # Left cam
    res = subprocess.run(["libcamera-vid",
                          "-t",f"{video_len}",
                          "-cs","0",
                          "-o",f"t-l.h264",
                          "--width" ,f"{width}",
                          "--height" ,f"{height}",
                          ], check=True)
    
    print("The exit code left cam was: %d" % res.returncode)

    # Right cam
    res = subprocess.run(["libcamera-vid",
                          "-t",f"{video_len}",
                          "-cs","0",
                          "-o",f"t-l.h264",
                          "--width" ,f"{width}",
                          "--height" ,f"{height}",
                          ], check=True)
    
    print("The exit code right cam was: %d" % res.returncode)

except Exception as e:
    print(e)