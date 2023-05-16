from pyfirmata import ArduinoMega, SERVO,OUTPUT,util,INPUT
from time import sleep


HIGH = 0x1
LOW = 0x0
DIR_PIN = 3
MICRO_STEPS = 2
MIN_ANGLE = 1.8  # in degrees
STEPS_PER_REV = 360 / MIN_ANGLE
PULSE_PER_REV = STEPS_PER_REV * MICRO_STEPS
MOTOR_RATED_VELOCITY = []
  
#M1
#  M1_rated_speed = ((0.17*1000L)/60) # At 6V
#M1_rated_speed = ((0.15 * 1000) / 60)  # At 7.4V
M1_rated_speed = ((0.13*1000)/60) # At 8.4V

#M2
M2_rated_speed = ((0.13 * 1000) / 60)  # At 7.2V
# M2_rated_speed = ((0.1*1000L)/60) # At 8.4V

#M3
# M3_rated_speed = ((0.185*1000L)/60) # At 6V
M3_rated_speed = ((0.151 * 1000) / 60)  # At 7.4V


#M4
# M4_rated_speed = ((0.18*1000L)/60) # At 4.8V
# M4_rated_speed = ((0.16*1000L)/60) # At 6V
M4_rated_speed = ((0.14 * 1000) / 60)  # At 7.2V

#M5
# M5_rated_speed = ((0.18*1000L)/60) # At 4.8V
# M5_rated_speed = ((0.16*1000L)/60) # At 6V
M5_rated_speed = ((0.14 * 1000) / 60)  # At 7.2V

MOTOR_RATED_VELOCITY = [
  M1_rated_speed,
  M2_rated_speed,
  M3_rated_speed,
  M4_rated_speed,
  M5_rated_speed
]

class Hardware:
    PORT = 'COM5'
    theta =  [0]*6
    theta_dot = [0]*6
    theta_dotdot = [0]*6
    torque = [0]*6
    delT = 0.1 #s
    SERVOS = [5,6,7,8,9]
    SERVOS_FEEDBACK = [0,1,2,3,4]
    STEPPER_DIR = 3
    STEPPER_PUl = 2
    n = 3
    MOTOR_PRE_POSITION = [0]*n
    MOTOR_POSITION = [0]*n
    MOTOR_VELOCITY = [0]*n
    MOTOR_EFFORT = [0]*n


    def init(self):
        print("Initializing the hardware connection...")
        self.board = ArduinoMega(self.PORT)

        for servo_pin,feedback_pin in zip(self.SERVOS,self.SERVOS_FEEDBACK):
            self.board.digital[servo_pin].mode = SERVO
            self.board.analog[feedback_pin].mode = INPUT

        self.board.digital[self.STEPPER_DIR].mode = OUTPUT
        self.board.digital[self.STEPPER_PUl].mode = OUTPUT

        # Run continuosly
        it = util.Iterator(self.board)
        it.start()

        print("Connected to hardware.")
    
    def set_servo(self, servo_num, angle,velocity): 
        print("Servo starting...")

        increment_angle = abs(self.MOTOR_PRE_POSITION[servo_num + 1] - angle)
        #time_sleep = abs((increment_angle / (increment_angle - 1)) * (velocity - MOTOR_RATED_VELOCITY[servo_num]))/1000  #s
        time_sleep = abs(velocity - MOTOR_RATED_VELOCITY[servo_num])/1000  #s

        if (self.MOTOR_PRE_POSITION[servo_num + 1] < angle):
            pos = 0
            for pos in range(self.MOTOR_PRE_POSITION[servo_num + 1] + 1,angle+1,1): 
                # in steps of 1 degree
                self.board.digital[self.SERVOS[servo_num]].write(angle)
                sleep(time_sleep)
            
            self.MOTOR_PRE_POSITION[servo_num + 1] = pos - 1
        else:
            pos = 0
            for pos in list(range(angle,self.MOTOR_PRE_POSITION[servo_num + 1],1))[::-1]: 
                self.board.digital[self.SERVOS[servo_num]].write(angle)
                sleep(time_sleep)
            
            self.MOTOR_PRE_POSITION[servo_num + 1] = pos + 1
        
        sleep(0.1)
        print("Servo ran.")

        # int direction = 1
        # float pre_state = MOTOR_STATE[servo_num]

        # pul_s = map(pre_state, 0, 180, SERVOMIN, SERVOMAX)
        # pul_e = map(new_state, 0, 180, SERVOMIN, SERVOMAX)

        # if (new_state < 0) 
        #   # Clock wise rotation
        #   for (pul = pul_s pul >= pul_e pul -= 1) 
        #     pwm.setPWM(servo_num, 0, pul)
        #     sleep(velocity)
        #   
        #  else 
        #   for (pul = pul_s pul <= pul_e pul += 1) 
        #     pwm.setPWM(servo_num, 0, pul)
        #     sleep(velocity)
        #   
        # 
        
    # Set stepper angle
    def set_stepper(self,angle,velocity):
        print("Stepper starting...")
        if (angle > 0): 
            self.board.digital[self.STEPPER_DIR].write(HIGH)
        else:
            self.board.digital[self.STEPPER_DIR].write(LOW)
        

        pulse_required = (angle / 360) * PULSE_PER_REV            # Total pulses required
        pulse_rate = abs(360*PULSE_PER_REV/velocity)  # pulse/sec
        steps_left = abs(pulse_required)

        step_sleep = 1 / (2 * (pulse_rate))  # msec / pulse

        # decrement the number of steps, moving one step each time:
        while (steps_left > 0):
            self.board.digital[self.STEPPER_PUl].write(HIGH)
            sleep(step_sleep)
            self.board.digital[self.STEPPER_PUl].write(LOW)
            sleep(step_sleep)
            steps_left-=1
        
        print("Stepper ran.")

    def set_motor(self,position,velocity,effort=[]):
        self.MOTOR_POSITION[0] = position[0]
        self.MOTOR_VELOCITY[0] = velocity[0]

        for i in range(0,self.n):
            pos = position[i]
            vel = velocity[i]

            print(i)
            if i==0:
                self.set_stepper(pos,vel)
            else:
                self.set_servo(i-1,pos,vel)

    
    def get_feedback(self):
        for i,feedback_pin in self.SERVOS_FEEDBACK:
            pre_pos = self.MOTOR_POSITION[i+1]
            self.MOTOR_POSITION[i+1] = self.board.analog[feedback_pin].read()
            self.MOTOR_VELOCITY[i+1] = (self.MOTOR_POSITION[i+1]-pre_pos)/self.delT

        return (self.MOTOR_POSITION,self.MOTOR_VELOCITY)

    def exit(self):
        self.board.exit()


def main():
    import tkinter as tk
    from tkinter import ttk,messagebox
        
    MOTOR_STATE = [0]* 6
    MOTOR_FEEDBACK = []


    
    def set_motor():
        hardware.set_motor(MOTOR_STATE)

    # root window
    root = tk.Tk()

    hardware = Hardware()
    messagebox.showinfo("Notification", "Connecting to Arduino...")
    root.update()

    try:
        hardware.init()
        messagebox.showinfo("Notification", "Connected.")
    except Exception as e:
        messagebox.showinfo("Error", str(e))
        root.destroy()
        exit(0)

    root.update()

        
   
    

    root.geometry('300x200')
    root.resizable(False, True)
    root.title('MIRS Motor Controller')


    root.columnconfigure(0, weight=1)
    root.columnconfigure(1, weight=3)


    # slider current value
    motor0_val = tk.DoubleVar()
    motor1_val = tk.DoubleVar()
    motor2_val = tk.DoubleVar()
    motor3_val = tk.DoubleVar()
    motor4_val = tk.DoubleVar()
    motor5_val = tk.DoubleVar()

    vel = 5
    def get_motor0_value():
        MOTOR_STATE[0] = (int(motor0_val.get()),vel)
        return round(MOTOR_STATE[0][0])

    def get_motor1_value():
        MOTOR_STATE[1] =(int( motor1_val.get()),vel)
        return round(MOTOR_STATE[1][0])

    def get_motor2_value():
        MOTOR_STATE[2]  = (int(motor2_val.get()),vel)
        return round(MOTOR_STATE[2][0])

    def get_motor3_value():
        print('Hii')
        MOTOR_STATE[3]  = (int(motor3_val.get()),vel)
        return round(MOTOR_STATE[3][0])

    def get_motor4_value():
        MOTOR_STATE[4]  = (int(motor4_val.get()),vel)
        return round(MOTOR_STATE[4][0])

    def get_motor5_value():
        MOTOR_STATE[5]  = (int(motor5_val.get()),vel)
        return round(MOTOR_STATE[5][0])


    def motor0_slider_changed(event):
        motor0_value_label.configure(text=get_motor0_value())
    def motor1_slider_changed(event):
        motor1_value_label.configure(text=get_motor1_value())
    def motor2_slider_changed(event):
        motor2_value_label.configure(text=get_motor2_value())
    def motor3_slider_changed(event):
        motor3_value_label.configure(text=get_motor3_value())
    def motor4_slider_changed(event):
        motor4_value_label.configure(text=get_motor4_value())
    def motor5_slider_changed(event):
        motor5_value_label.configure(text=get_motor5_value())


    # Motor0
    motor0_slider_label = ttk.Label(
        root,
        text='Motor 0:'
    )

    motor0_slider_label.grid(
        row=0,
        column=0,
        sticky='w'
    )

    motor0_slider = ttk.Scale(
        root,
        from_=-180,
        to=180,
        orient='horizontal',  # vertical
        command=motor0_slider_changed,
        variable=motor0_val
    )

    motor0_slider.grid(
        row=0,
        column=1,
        sticky='we'
    )

    # value label
    motor0_value_label = ttk.Label(
        root,
        text=get_motor0_value(),
        width=15,
        anchor='center'
        
    )
    motor0_value_label.grid(
        row=0,
        column=2,
    )

        # Motor1
    motor1_slider_label = ttk.Label(
        root,
        text='Motor 1:'
    )

    motor1_slider_label.grid(
        row=1,
        column=0,
        sticky='w'
    )

    motor1_slider = ttk.Scale(
        root,
        from_=0,
        to=180,
        orient='horizontal',  # vertical
        command=motor1_slider_changed,
        variable=motor1_val
    )

    motor1_slider.grid(
        row=1,
        column=1,
        sticky='we'
    )


    # value label
    motor1_value_label = ttk.Label(
        root,
        text=get_motor1_value()
    )
    motor1_value_label.grid(
        row=1,
        column=2
    )

    # Motor2
    motor2_slider_label = ttk.Label(
        root,
        text='Motor 2:'
    )

    motor2_slider_label.grid(
        row=2,
        column=0,
        sticky='w'
    )

    motor2_slider = ttk.Scale(
        root,
        from_=0,
        to=180,
        orient='horizontal',  # vertical
        command=motor2_slider_changed,
        variable=motor2_val
    )

    motor2_slider.grid(
        row=2,
        column=1,
        sticky='we'
    )


    # value label
    motor2_value_label = ttk.Label(
        root,
        text=get_motor2_value()
    )
    motor2_value_label.grid(
        row=2,
        column=2
    )



    # Motor3
    motor3_slider_label = ttk.Label(
        root,
        text='Motor 3:'
    )

    motor3_slider_label.grid(
        row=3,
        column=0,
        sticky='w'
    )

    motor3_slider = ttk.Scale(
        root,
        from_=0,
        to=180,
        orient='horizontal',  # vertical
        command=motor3_slider_changed,
        variable=motor3_val
    )

    motor3_slider.grid(
        row=3,
        column=1,
        sticky='we'
    )


    # value label
    motor3_value_label = ttk.Label(
        root,
        text=get_motor3_value()
    )
    motor3_value_label.grid(
        row=3,
        column=2
    )

    # Motor 4
    motor4_slider_label = ttk.Label(
        root,
        text='Motor 4:'
    )

    motor4_slider_label.grid(
        row=4,
        column=0,
        sticky='w'
    )

    motor4_slider = ttk.Scale(
        root,
        from_=0,
        to=180,
        orient='horizontal',  # vertical
        command=motor4_slider_changed,
        variable=motor4_val
    )

    motor4_slider.grid(
        row=4,
        column=1,
        sticky='we'
    )


    # value label
    motor4_value_label = ttk.Label(
        root,
        text=get_motor4_value()
    )
    motor4_value_label.grid(
        row=4,
        column=2
    )


    # Motor5
    motor5_slider_label = ttk.Label(
        root,
        text='Motor 5:'
    )

    motor5_slider_label.grid(
        row=5,
        column=0,
        sticky='w'
    )

    motor5_slider = ttk.Scale(
        root,
        from_=0,
        to=180,
        orient='horizontal',  # vertical
        command=motor5_slider_changed,
        variable=motor5_val
    )

    motor5_slider.grid(
        row=5,
        column=1,
        sticky='we'
    )


    # value label
    motor5_value_label = ttk.Label(
        root,
        text=get_motor5_value()
    )
    motor5_value_label.grid(
        row=5,
        column=2
    )


    # Recieve feedback
    def get_motor_values(event):
        hardware.get_feedback(hardware.display_feedback)
        
    # Send button
    send_button = ttk.Button(
        root,
        text="Send",
        command=hardware.set_motor
    )

    send_button.grid(
        row=6,
        column=0
    )

    recieve_button = ttk.Button(
        root,
        text="Recieve",
        command=get_motor_values
    )
    recieve_button.grid(
        row=6,
        column=1
    )

    def on_closing():
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            hardware.exit()
            root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()
            
if __name__ == '__main__':
    main()