import tkinter as tk
from functools import partial
from tkinter import ttk, messagebox, Frame, filedialog
import numpy as np
np.set_printoptions(precision=4)


HISTORY_FILE = "history.txt"


class PARAMS:
    GOAL_X = "goal_x"
    GOAL_Y = "goal_y"
    GOAL_Z = "goal_z"
    GOAL_T = "goal_t"


class Console:

    def __init__(self, n_joints, width=600, height=600, precision=4):
        self.n = n_joints
        self.theta = np.zeros((self.n, 1))
        self.theta_dot = np.zeros((self.n, 1))
        self.joints = [None]*n_joints
        self.joints_label = [None]*n_joints
        self.joints_value_label = [None]*n_joints
        self.robot = None
        self.joints_slider = [None]*n_joints
        self.params = {
            PARAMS.GOAL_X: 0.0,
            PARAMS.GOAL_Y: 0.0,
            PARAMS.GOAL_Z: 0.0,
            PARAMS.GOAL_T: 0.0,
        }

        # root window
        self.root = tk.Tk()
        # self.root.geometry('900x300')
        # self.root.resizable(False, False)
        self.root.title('MIRS Console')
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=3)
        self.window = Frame(self.root)
        # self.window = Frame(self.root, width=width, height=height)
        self.window.grid(row=0, column=0, padx=(10, 10), pady=(10, 10))

        self.load_saved_params()

    # def add_canvas_to_window(self, figure, row=5, col=0, rowspan=7, colspan=7):
    #     self.canvas = FigureCanvasTkAgg(figure, master=self.window)
    #     self.canvas.get_tk_widget().grid(
    #         row=row, column=col, columnspan=colspan, rowspan=rowspan)
    #     self.canvas.draw()

    def get_graphic_canvas(self):
        return self.canvas

    def get_root(self):
        return self.root

    def get_window(self):
        return self.window

    def move_to_goal(self):
        x = self.goal_x.get()
        y = self.goal_y.get()
        z = self.goal_z.get()
        t = self.goal_t.get()

        self.params[PARAMS.GOAL_X] = x
        self.params[PARAMS.GOAL_Y] = y
        self.params[PARAMS.GOAL_Z] = z
        self.params[PARAMS.GOAL_T] = t

        self.goal_callback(np.array([x, y, z]).reshape(-1, 1), t)

    def set_robot_params(self, robot):
        self.robot = robot

    def load_saved_params(self):
        with open(HISTORY_FILE, 'r') as f:
            for line in f.readlines():
                param = line.strip().split("=")
                param_name = param[0]
                param_value = param[1]
                self.params[param_name] = float(param_value)

    def save_params(self):
        with open(HISTORY_FILE, 'w') as f:
            for param in self.params:
                line = param+"="+str(self.params[param])+"\n"
                f.write(line)

    def before_exit(self, save_params=True):
        if save_params:
            self.save_params()

        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.root.withdraw()
            self.root.destroy()

        if self.on_exit_callback is None:
            pass
        else:
            self.on_exit_callback()

    def on_new_program(self, callback):
        self.new_program_callback = callback

    def on_goal(self, callback):
        self.goal_callback = callback

    def on_manual_control(self, callback):
        self.manual_control_callback = callback

    def on_exit(self, callback):
        self.on_exit_callback = callback

    def get_joint_value(self, i, deg=True):
        if deg:
            return self.joints[i].get()
        return np.deg2rad(self.joints[i].get())

    def slider_change_callback(self, *params):
        i = params[0]
        self.theta[i, 0] = self.get_joint_value(i, deg=False)
        self.joints_value_label[i].configure(text=round(self.theta[i, 0], 2))
        self.manual_control_callback(self.theta)

    def select_file(self):
        # file_path = filedialog.askopenfilename(title="Select a file", filetypes=[
        #                                        ("Text files", "*.txt"), ("All files", "*.*")])
        self.new_program_callback()

    def create_ui(self):
        # Create joint variable, joint slider & label
        for i in range(0, self.n):
            # Joint
            self.joints[i] = tk.DoubleVar()

            # Joint Label
            self.joints_label[i] = ttk.Label(self.window, text=f'Joint {i+1}:')
            self.joints_label[i].grid(row=i, column=0, sticky='w')

            # Joint Slider
            self.joints_slider[i] = ttk.Scale(self.window, from_=self.robot.joint_limits[i][0], to=self.robot.joint_limits[i][1], orient='horizontal',
                                              variable=self.joints[i], command=partial(self.slider_change_callback, i))
            self.joints_slider[i].grid(row=i, column=1, sticky='we')

            # Joint Value Label
            self.joints_value_label[i] = ttk.Label(
                self.window, text=self.get_joint_value(i), width=15, anchor='center')
            self.joints_value_label[i].grid(row=i, column=2)

        self.goal_x = tk.DoubleVar()
        self.goal_y = tk.DoubleVar()
        self.goal_z = tk.DoubleVar()
        self.goal_t = tk.DoubleVar()

        # Set loaded params
        self.goal_x.set(self.params[PARAMS.GOAL_X])
        self.goal_y.set(self.params[PARAMS.GOAL_Y])
        self.goal_z.set(self.params[PARAMS.GOAL_Z])
        self.goal_t.set(self.params[PARAMS.GOAL_T])

        goal_label = ttk.Label(self.window, text='Goal')
        goal_label.grid(row=0, column=3, sticky='w')
        goal_x_label = ttk.Label(self.window, text='X :')
        goal_x_label.grid(row=1, column=3, sticky='w')
        goal_x_entry = ttk.Entry(self.window, textvariable=self.goal_x)
        goal_x_entry.grid(row=1, column=4, sticky='w')

        goal_y_label = ttk.Label(self.window, text='Y :')
        goal_y_label.grid(row=2, column=3, sticky='w')
        goal_y_entry = ttk.Entry(self.window, textvariable=self.goal_y)
        goal_y_entry.grid(row=2, column=4, sticky='w')

        goal_z_label = ttk.Label(self.window, text='Z :')
        goal_z_label.grid(row=3, column=3, sticky='w')
        goal_z_entry = ttk.Entry(self.window, textvariable=self.goal_z)
        goal_z_entry.grid(row=3, column=4, sticky='w')

        goal_t_label = ttk.Label(self.window, text='T :')
        goal_t_label.grid(row=4, column=3, sticky='w')
        goal_t_entry = ttk.Entry(self.window, textvariable=self.goal_t)
        goal_t_entry.grid(row=4, column=4, sticky='w')

        # Move to goal button
        goal_button = ttk.Button(
            self.window, text="Move", command=partial(self.move_to_goal))
        goal_button.grid(row=1, column=5)

        # Program select button
        program_select_button = ttk.Button(
            self.window, text="Execute Program", command=partial(self.select_file))
        program_select_button.grid(row=2, column=5)

        # Exit button
        exit_button = ttk.Button(
            self.window, text="Exit", command=partial(self.before_exit))
        exit_button.grid(row=3, column=5)

    def loop(self):

        self.create_ui()

        self.root.protocol("WM_DELETE_WINDOW", self.before_exit)
        self.root.mainloop()



if __name__ == "__main__":
    def test1(*args):
        print(args)

        con = Console(6)
        con.main(test1, test1, None)
