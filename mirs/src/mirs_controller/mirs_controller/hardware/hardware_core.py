import binascii
import random
import threading
from pyfirmata import ArduinoMega, util,STRING_DATA

class Hardware:
    
    PORT = 'COM5'
    theta =  [0]*6
    theta_dot = [0]*6
    theta_dotdot = [0]*6
    torque = [0]*6
    delT = 0.1 #s

    # 0x00-0x0F (0-15) reserved for user-defined commands 

    MIRS_COMMAND = 0x04
    COMMAND_MOTOR_GET_POSITION = 0x05
    COMMAND_MOTOR_SET_POSITION = 0x06
    COMMAND_MOTOR_SET_VELOCITY = 0x07
    MOTORS = 1
 

    def init(self):
        print("Initializing the hardware connection...")
        self.board = ArduinoMega(self.PORT)

        # Add callback to recieve feedback 
        self._motor_feedback_callback = None
        self.board.add_cmd_handler(self.MIRS_COMMAND,self.mirs_command_handler)
        self.board.add_cmd_handler(STRING_DATA,self.print_msg)
        print("Connected to hardware.")

    def print_msg(self,*data, **kwargs):
        msg = util.two_byte_iter_to_str(data)
        print("MSG :" ,msg)

    def parse(self,data_f,data_l):
        return (data_f & 0x7F) | ((data_l & 0x7F) << 7)
    
    def mirs_command_handler(self,*data):
        #print("MIRS_RESPONSE : 0x{0}".format(bytearray(data)))
        if len(data)<1:
            print("Response with no data found!")
            return

        # Get header of 7bit
        command = (data[0] & 0x7F) | ((data[1] & 0x7F) << 7)  
        data = data[2:]

        if command == self.COMMAND_MOTOR_GET_POSITION:
            if len(data)<(2*self.MOTORS):
                print("Not enough params for feedback!")
                return
            
            theta = []

            for i in range(0,2*self.MOTORS,2):
                theta.append((data[i] & 0x7F) | ((data[i+1] & 0x7F) << 7))

            if self._motor_feedback_callback is not None:
                self._motor_feedback_callback(theta)
            else:
                print("No callback for motor feedback")
        else:
            print(data)
            print("Unknown command !")

    # Get motor feedback
    def get_feedback(self,callback):
        self._motor_feedback_callback = callback
        self.board.send_sysex(self.MIRS_COMMAND,[self.COMMAND_MOTOR_GET_POSITION])
    
    def set_torque(self,torque):
        print(torque)
        pass

    def display_feedback(self,data):
        print("Recieved feedback :",data)
    

    def get_state(self):
        self.theta,self.theta_dot,self.theta_dotdot,self.torque

    def set_state(self,motor_positons,motor_velocities):
        position_data = bytearray([self.COMMAND_MOTOR_SET_POSITION])
        velocity_data = bytearray([self.COMMAND_MOTOR_SET_VELOCITY])


        # Encode data (Convert to bytes data)
        # Pack 14bits into 2 - 7 bit bytes
        # val &= 0x3FFF  # Converting into 14 bits data
        # b1 = val & 0x7F  # Get last 7 bits
        # b2 = val >> 7 # Get remaining 7 bits
        # data.extend([b1,b2])
        
        for val in motor_positons:
            position_data.extend(util.to_two_bytes(val))

        for val in motor_velocities:
            velocity_data.extend(util.to_two_bytes(val))
        
        print("Sending sysex...")

        # First send velocity data then position data
        self.board.send_sysex(self.MIRS_COMMAND,velocity_data)
        self.board.send_sysex(self.MIRS_COMMAND,position_data)

    def set_feedback(self,data):
        print("Set feedback called")
        # Update state using  feedback data
        for i,val in enumerate(data):
            self.theta_dot[i]  = self.theta_dot[i] + ((self.theta[i]-val)/self.delT)
            self.theta[i]  = val

    def close(self):
        self.board.exit()


def main():
    while True:
        try:
            # Send motor values
            motor_positions = random.sample(test_positons,hardware.MOTORS)
            motor_velocities = random.sample(test_velocity,hardware.MOTORS)
            hardware.set_state(motor_positions,motor_velocities)
            time.sleep(3)

            # Recieve feedback
            hardware.get_feedback(hardware.display_feedback)
        except Exception as e:
            raise(e)
           


if __name__ == "__main__":
    import time

 
    hardware = Hardware()
    hardware.init()

    

    it = util.Iterator(hardware.board)
    it.start()

    # t1 = threading.Thread(main())
    # threading.Timer(1,user_input).start()
        
    
    test_positons = [60]*5
    test_velocity = [60]*5
    
    main()

    hardware.close()
