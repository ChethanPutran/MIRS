import os
import socket
import struct
import pickle
import cv2
import threading
import time

STERIO = False
RECORDING_DIR = 'recordings'
DEBUG = False

class CameraClient:
    def __init__(self,host='raspberrypi.local',port=8000):
        self.host = host
        self.port = port
        self.HEADER_SIZE = 10
        self.socket = socket.socket()

        self.socket.connect((self.host,self.port))
        self.live_ended = False

    def recieve(self,is_file=False,large_file=False):
        if DEBUG:
            print("Recieving...",end='')
        new_msg = True
        res = b''
        msg_len = 0

        buff_size = 16

        if is_file:
            if large_file:
                buff_size = 4096
            else:
                buff_size = 1024

        while True:
            if DEBUG:
                print('.',end='')
            msg = self.socket.recv(buff_size)

            if new_msg:
                msg_len = int(msg[:self.HEADER_SIZE])
                new_msg = False

            res += msg

            if(len(res)-self.HEADER_SIZE == msg_len):
                new_msg = True
                res = res[self.HEADER_SIZE:]
                break

        res = pickle.loads(res)

        if DEBUG:
            print("\nRecieved.")
        return res

    def send(self,got_size=None,start_recording=None,end_recording=None,get_recording=None,exit=None,start_live=None,end_live=None):
        if DEBUG:
            print("Sending...")
        req = {
            "got_size":got_size,
            "start_recording":start_recording,
            "end_recording":end_recording,
            "get_recording":get_recording,
            "exit":exit,
            "start_live":start_live,
            "end_live":end_live,
        }

        req = pickle.dumps(req)
        req = bytes(f"{len(req):<{self.HEADER_SIZE}}",encoding="utf-8") + req
        self.socket.sendall(req)

        if DEBUG:
            print("Sent.")

    def start_recording(self):
        self.send(start_recording=True)
        res = self.recieve()
        return res

    def end_recording(self):
        self.send(end_recording=True)
        res = self.recieve()
        return res

    def extract_video(self,recording,sterio=STERIO):
        recording_dir = os.path.join(os.path.dirname(__file__),RECORDING_DIR)

        fname_l = os.path.join(recording_dir,recording[0][0])
        buff_l = recording[0][1]

        with open(fname_l,'wb') as file:
            file.write(buff_l)

        if sterio:
            fname_r = os.path.join(recording_dir,recording[1][0])
            buff_r = recording[1][1]

            with open(fname_r,'wb') as file:
                file.write(buff_r)
            return fname_l,fname_r
        
        return fname_l
    
    def get_recording(self):
        self.send(get_recording=True)
        res = self.recieve(is_file=True)
        if res['error']:
            return None,res['error']
        else:
            print(res['message'])
        
        return self.extract_video(res['data']),None

    def close(self):
        self.socket.close()
        print("Exiting... Bye!")

    def exit(self):
        self.send(exit=True)
        return self.recieve()

    def start_live(self):
        self.send(start_live=True)

        self.live_ended = False
        th = threading.Thread(target=self.recieve_live_capture)
        th.start()

    def end_live(self):
        self.send(end_live=True)
        animation = "|/-\\"
        idx = 0
        while not self.live_ended:
            print(animation[idx % len(animation)], end="\r")
            idx += 1
            time.sleep(0.1)

        res = self.recieve()
        return res
    
    def recieve_live_capture(self):
        data = b''
        payload_size = struct.calcsize("L") 

        while not self.live_ended:
            while len(data) < payload_size:
                data += self.socket.recv(4096)

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            
            msg_size = struct.unpack("L", packed_msg_size)[0]

            while len(data) < msg_size:
                data += self.socket.recv(4096)
                
            frame_data = data[:msg_size]

            # Update data
            data = data[msg_size:]

            if frame_data == b"<END>":
                print('Ending live capture...')
                self.live_ended = True
                continue

            frame=pickle.loads(frame_data)
            cv2.imshow('frame',frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.send(end_live=True)

        cv2.destroyAllWindows()
           
def main():

    camera_client = CameraClient("localhost")

    print("*"*50)
    print("1 - Start recording \n\
        2 - End recording \n\
        3 - Get recording \n\
        4 - Start Live streaming \n\
        5 - End Live streaming \n\
        6 - Exit \n\
        ")

    try:
        while True:
            command = int(input(">"))

            if(command==1):
                res = camera_client.start_recording()
                if res['error']:
                    print(res["error"]) 
                else:
                    print(res["message"])
            elif(command==2):
                res = camera_client.end_recording()
                if res['error']:
                    print(res["error"])
                else:
                    print(res["message"])
            elif(command==3):
                recording,error= camera_client.get_recording()
                if  error:
                    print(error)
                else:
                    print("Received recording :",recording)
            elif(command==4):
                camera_client.start_live()
                print("Live started...")
            elif(command==5):
                res = camera_client.end_live()
                if res["error"]:
                    print(res["error"])
                else:
                    print(res["message"])
            elif(command==6):
                res = camera_client.exit()
                if res['error']:
                    print(res['error'])
                else:
                    print(res["message"])
                    break
    except Exception as e:
        print(e)
    camera_client.close()  # close the connection


if __name__ == '__main__':
    main()