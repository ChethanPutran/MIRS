import time
from http.server import HTTPServer, BaseHTTPRequestHandler
import serial
import socket


class Server(BaseHTTPRequestHandler):
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self):
        self.connection = self.socket.accept()

    def send(self, command):
        self.socket.send(command)


class Connection:
    """ Configuring the serial port data """
    @staticmethod
    def connect(host='localhost', port=3000):
        Connection.HOST_NAME = host
        Connection.SERVER_PORT = port
        Connection.arduino = serial.Serial(
            port="COM3", baudrate=9600, timeout=0.1)
        Connection.web_server = HTTPServer(
            (Connection.HOST_NAME, Connection.SERVER_PORT), Server)

    @staticmethod
    def start_server():
        Connection.web_server.serve_forever()
        print("Server started http://%s:%s" %
              (Connection.HOST_NAME, Connection.SERVER_PORT))

    @staticmethod
    def stop_server():
        # Stop server when Ctr + C is pressed
        Connection.web_server.server_close()
        print("Server stopped.")

    @staticmethod
    def get_data(data_label):
        """ Getting data from serial """
        Connection.arduino.write(bytes(data_label))
        time.sleep(0.0001)
        return Connection.arduino.readline().decode("utf-8").rstrip("\n\r")

    @staticmethod
    def set_data(data_label, value):
        data = "$"+data_label+"$"+value
        return Connection.arduino.write(bytes(data))
