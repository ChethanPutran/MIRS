import os
import paramiko
from scp import SCPClient

host = "raspberrypi.local"
user="cheth"
password="root"
port =22

dir_path = os.path.dirname(__file__)

def createSSHClient(server, port, user, password):
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(server, port, user, password)
    return client

ssh = createSSHClient(host, port, user, password)
scp = SCPClient(ssh.get_transport())

print(f"{dir_path}\\stereo\\")
print("Recieving...")
for i in range(1,3):
    try:
        scp.get(f"img-left-{i}.jpg",f"{dir_path}\\stereo\\")
        scp.get(f"img-right-{i}.jpg",f"{dir_path}\\stereo\\")
    except Exception as e:
        print(e)
print("Sucessfull.")