import subprocess


for i in range(1,5):
    try:
        res = subprocess.run(["libcamera-jpeg","-o",f"img-left-{i}.jpg","--camera" ,"0"], check=True)
        print("The exit code left cam was: %d" % res.returncode)
        res = subprocess.run(["libcamera-jpeg","-o",f"img-right-{i}.jpg","--camera" ,"1"], check=True)
        print("The exit code right cam was: %d" % res.returncode)

    except Exception as e:
        print(e)
