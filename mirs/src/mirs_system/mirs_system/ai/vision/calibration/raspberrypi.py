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

    def get_file(self,file_name,dest_folder):
        print("Recieving...")
        try:
            self.scp.get(file_name,dest_folder=DEST_FOLDER)
            print("Sucessfull.")
            return os.path.join(dest_folder,file_name)
        except Exception as e:
            print(e)
        
    def get_calibration_images(self,width=640,height=480):
        fname_l = "calib_image_l.jpg"
        fname_r = "calib_image_r.jpg"

        cmd = f"libcamera-jpeg -o {fname_l} --camera 0 --width={width} --height={height}"
        out,err = self.execute(cmd)

        if out.channel.recv_exit_status() == 0:
            calib_l_file = self.get_file(fname_l)
        else:
            return [],err.readlines()
        
        cmd = f"libcamera-jpeg -o {fname_r} --camera 1 --width={width} --height={height}"
        out,err = self.execute(cmd)

        if out.channel.recv_exit_status() == 0:
            calib_r_file = self.get_file(fname_r)
        else:
            return [],err.readlines()
        
        return (calib_l_file,calib_r_file,),None
    
    def get_sample_image(self,folder,width=640,height=480):
        fname_l = "sample_image_l.jpg"
        fname_r = "sample_image_r.jpg"

        cmd = f"libcamera-jpeg -o {fname_l} --camera 0 --width={width} --height={height}"
        out,err = self.execute(cmd)

        if out.channel.recv_exit_status() == 0:
            calib_l_file = self.get_file(fname_l,folder)
        else:
            return [],err.readlines()
        
        cmd = f"libcamera-jpeg -o {fname_r} --camera 1 --width={width} --height={height}"
        out,err = self.execute(cmd)

        if out.channel.recv_exit_status() == 0:
            calib_r_file = self.get_file(fname_r,folder)
        else:
            return [],err.readlines()
        
        return (calib_l_file,calib_r_file,),None

    def get_sample_video(self,video_len=5,width=640,height=480):
        fname_l = "sample_video_left.avi"
        fname_r = "sample_video_right.avi"
        cmd_l = f"libcamera-vid -t {video_len} -cs 0 -o {fname_l} --width {width} --height{height}"
        cmd_r = f"libcamera-vid -t {video_len} -cs 0 -o {fname_r} --width {width} --height{height}"          
        pass




if __name__ == "__main__":
    rasp = Raspberrypi()

    out,err = rasp.execute("ls")

    print("Exit status : ",out.channel.recv_exit_status())
    print("\n\nTest output :")

    for line in out:
        print(line)

    print("\n\n")
    print("Test error :")

    for line in err:
        print(line)