import os
import paramiko
from scp import SCPClient


DEST_FOLDER= os.path.join(os.path.dirname(__file__),"stereo")

class Raspberrypi:
    def __init__(self,host="raspberrypi.local",user="cheth",password="root"):
        self.connected = False
        self.connect(host,user,password)

    def connect(self,host,user,passwd):
        try:
            print("Connecting to raspberrypi...")
            self.client = paramiko.SSHClient()
            self.client.load_system_host_keys()
            self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self.client.connect(host, username=user, password=passwd)
            self.scp = SCPClient(self.client.get_transport())
            print("Connected to raspberrypi.")
            self.connected = True
        except Exception as e:
            print("Failed to connect to raspberry pi! Error :",e)

    def execute(self,command):
        if self.connected:
            stdin, stdout, stderr = self.client.exec_command(command)
            return stdout,stderr
        else:
            print("No connection!")

    def close(self):
        self.client.close()
        self.connected = False

    def get_file(self,file_name,dest_folder=DEST_FOLDER):
        print("Recieving...")
        try:
            self.scp.get(file_name,local_path=dest_folder)
            print("Sucessfull.")
            return (os.path.join(dest_folder,file_name),None,)
        except Exception as e:
            return (None,e,)

    def get_stereo_images(self,fname,folder,width,height):
        file_name_l = "left_"+fname
        file_name_r = "right_"+fname
        cmd = f"libcamera-jpeg -o {file_name_l} --camera 0 --width={width} --height={height}"
        out,err = self.execute(cmd)

        if out.channel.recv_exit_status() == 0:
            calib_l_file,err = self.get_file(file_name_l,dest_folder=folder)
            if err:
                return ([],err,)

        else:
            return ([],"".join(err.readlines()),)
        
        cmd = f"libcamera-jpeg -o {file_name_r} --camera 1 --width={width} --height={height}"
        out,err = self.execute(cmd)

        if out.channel.recv_exit_status() == 0:
            calib_r_file,err= self.get_file(file_name_r,dest_folder=folder)
            if err:
                return ([],err)
        else:
            return ([],"".join(err.readlines()),)
        
        return ((calib_l_file,calib_r_file,),None,)

    def get_calibration_images(self,width=640,height=480):
        fname = "calib_image.jpg"
        files,err = self.get_stereo_images(fname,width=width,height=height)
        return (files,err,)
    
    def get_sample_image(self,folder,width=640,height=480):
        fname = "sample_image.jpg"
        files,err = self.get_stereo_images(fname,folder,width=width,height=height)
        return files,err 

    def get_sample_video(self,video_len=5,width=640,height=480):
        fname = "sample_video"

        cmd = f"python get_video.py -r {video_len} -f {fname} -w {width} -h {height}"         
        out,err = self.execute(command=cmd)

        if out.channel.recv_exit_status() == 0:
            left_v_pth,err= self.get_file("left_"+fname+".avi")
            if err:
                return ((None,None),err,)
            right_v_pth,err= self.get_file("right_"+fname+".avi")
            if err:
                return ((None,None),err,)
            return (left_v_pth,right_v_pth),None
        return ((None,None),"".join(err.readlines()),)



if __name__ == "__main__":
    rasp = Raspberrypi()

    out,err = rasp.execute("ls")

    print("Exit status : ",out.channel.recv_exit_status())
    print("\n\nTest output :")

    for line in out:
        print(line)

    print("\n\n")
    print("Test error :")

    for line in err.readlines():
        print(line)