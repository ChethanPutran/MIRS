import pyfirmata

class Hardware:
    theta = []
    theta_dot = []
    theta_dotdot = []
    torque = []
    PORT = 'COM5'
    board = pyfirmata.Arduino(PORT)

    def __init__(self) -> None:
        # Add callback to recieve feedback 
        self.board.add_cmd_handler('GET_FEEBACK',self.set_feedback) 
        #self.board.send_sysex('GET_FEEBACK',None)
       
    def get_state(self):
        self.theta,self.theta_dot,self.theta_dotdot,self.torque

    def set_state(self,motor_vals):
        self.board.send_sysex('MOTOR_VALS',motor_vals)

    def set_feedback(self,data):
        motor_feedback_vals.clear()

        # Add feedback data
        for val in data:
            motor_feedback_vals.append(val)


def main():

    # Set feedbacks
    def set_feedback(values):
        print(values)
        motor_feedback_vals.clear()

        for val in values:
            motor_feedback_vals.append(val)

    import time

    motor_vals  = [0]*6
    motor_feedback_vals  = [0]*6

    PORT = 'COM5'
    board = pyfirmata.Arduino(PORT)

    # Add feedback call
    board.add_cmd_handler('GET_FEEBACK',set_feedback)


    while True:
        try:
            vals = []
            # Send motor values
            #board.send_sysex('GET_FEEBACK',None)
            data = input("Enter values to send :\n")

            for i in data.strip().split(" "):
                vals.append(float(i))
            board.send_sysex('MOTOR_VALS',motor_vals)
            time.sleep(.5)
            print("Recieved values")
            print(motor_feedback_vals)

        except Exception as e:
            print(e)
            break

if __name__ == "__main__":
    main()
