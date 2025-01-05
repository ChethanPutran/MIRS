from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np
from utils import get_coordinates
from trajectory_generator import Trajectory
import uuid
import time


class Vector:
    def __init__(self, start_point, end_point):
        self.start = np.array([[start_point[0]], [start_point[1]]])
        self.end = np.array([[end_point[0]], [end_point[1]]])

    def magnitude(self):
        return np.sqrt((self.end[0]-self.start[0])**2 + (self.end[1]-self.start[1])**2)


class TrajectoryPlotter:
    def __init__(self, ax):
        self.ax = ax
        self.trajectory = None
        self.trajectory_plot = None
        self.marker_color = "red"
        self.marker_type = "dot"
        self.marker_size = 3

    def set_plot_params(self, marker_size=3, marker_color="red", marker_type="dot"):
        self.marker_size = marker_size
        self.marker_color = marker_color
        self.marker_type = marker_type

    def plot_trajectory(self, trajectory):
        if self.trajectory_plot:
            self.trajectory_plot.remove()

        self.trajectory = trajectory
        # print("Q :", trajectory.q)
        points_mat = None
        if (self.trajectory.trajectory_method == Trajectory.JOINT_SPACE):
            points_mat = self.trajectory.to_cartesian_space()
            # print(points_mat)
        else:
            points_mat = self.trajectory.q

        X, Y, Z = points_mat[0, :], points_mat[1, :], points_mat[2, :]

        self.trajectory_plot = self.ax.scatter(
            X, Y, Z, linewidth=3, color=self.marker_color, s=self.marker_size)

        plt.draw()


class RoboPlotter(TrajectoryPlotter):
    def __init__(self, window_size, robot_model, frame_display=True, graph_display=True, FRAME_LINE_WIDTH=1, debug=False):
        self.robot = robot_model
        self.debug = debug
        self.FRAME_LINE_WIDTH = FRAME_LINE_WIDTH
        self.n = robot_model.n
        self.link_plots = [None]*robot_model.n

        self.frame_plots = np.array(
            [[None]*(robot_model.n+1) for _ in range(3)])

        self.fig = plt.figure(figsize=window_size)

        gs = GridSpec(3, 4, figure=self.fig)

        # gs.update(wspace=2, hspace=1, left=0, right=1)
        gs.update(wspace=.5, hspace=.5, left=0.01,
                  right=0.99, top=0.95, bottom=0.05)
        gs.tight_layout(figure=self.fig, pad=10, w_pad=5)
        plt.subplots_adjust()

        # create sub plots as grid

        # For Robot visualization
        self.ax1 = self.fig.add_subplot(
            gs[0:, :2], projection='3d', autoscale_on=False)

        super().__init__(self.ax1)
        # For parameter graph visualization
        self.ax2 = self.fig.add_subplot(gs[0, 2:])
        self.ax3 = self.fig.add_subplot(gs[1, 2:])
        self.ax4 = self.fig.add_subplot(gs[2, 2:])

        self.frame_display = frame_display
        self.graph_display = graph_display

        self.th_r = [0]
        self.th_a = [0]
        self.th_d_r = [0]
        self.th_d_a = [0]
        self.u = [0]
        self.t = [0]

    def plot_trajectory(self, trajectory):
        self.plot_trajectory(trajectory)

    def init_simulation(self):
        self.show_graphics()

    def animate2(self):
        for i in range(self.trajectory.n_steps):
            time_stamp, theta, theta_d, theta_dd = self.trajectory.get_cur_goal()
            # print(f"T{time_stamp} --> theta : {theta}")
            self.step(theta, theta_d, theta_dd)
            # self.update_graphics(self.robot.links)
            self.time_text.set_text(
                f'time = {time_stamp}s')
            self.trajectory.update_cur_goal()
            plt.pause(self.trajectory.delta_t)

    def init(self):
        return self.link_plots, self.frame_plots

    def start_simulation(self, trajectory, call_back):
        self.trajectory = trajectory
        self.step = call_back
        self.animate2()

    def close(self):
        print("Closing simulation...")
        plt.close(self.fig)

    def display_graphs(self):
        self.ref_th_plot = self.ax2.plot(
            self.t, self.th_r, label="Planned Position", color="orange")[0]
        self.act_th_plot = self.ax2.plot(
            self.t, self.th_a, label="Actual Position", color="green")[0]
        self.ax2.legend()
        self.ax2.set_xlabel("Time")
        self.ax2.set_ylabel("Position")
        self.ax2.set_title("Ref Position vs Actual Position")
        self.ref_th_d_plot = self.ax3.plot(
            self.t, self.th_d_r, label="Planned Velocity", color="orange")[0]
        self.act_th_d_plot = self.ax3.plot(
            self.t, self.th_d_a, label="Actual Velocity", color="green")[0]
        self.ax3.set_title("Ref Velocity vs Actual Velocity")
        self.ax3.legend()
        self.ax3.set_xlabel("Time")
        self.ax3.set_ylabel("Velocity")
        self.control_plot = self.ax4.plot(self.t, self.u, color="orange")[0]
        self.ax4.set_title("Control Input vs Time")
        self.ax4.set_xlabel("Time")
        self.ax4.set_ylabel("Control Input")

    def get_3D_axis(self):
        return self.ax1

    def display_links(self):
        self.ax1.axes.set_xlim3d(left=-self.robot.model.H_MAX,
                                 right=self.robot.model.H_MAX)
        self.ax1.axes.set_ylim3d(
            bottom=-self.robot.model.H_MAX, top=self.robot.model.H_MAX)
        self.ax1.axes.set_zlim3d(bottom=0, top=self.robot.model.H_MAX)
        self.ax1.grid()
        self.ax1.set_title('MIRS')
        self.ax1.set_xlabel("X")
        self.ax1.set_ylabel("Y")
        self.ax1.set_zlabel("Z")
        self.time_text = self.ax1.text(
            0.5, 0.5, 0.5, s='', transform=self.ax1.transAxes)
        self.trajectory = None
        for i in range(0, self.n):
            link = self.robot.links[i]
            link_colour = self.robot.colors[i]
            X, Y, Z = get_coordinates(link)
            self.link_plots[i], = self.ax1.plot(
                X, Y, Z, linewidth=3, color=link_colour)

    def display_frames(self):
        for i in range(0, self.n+1):
            frame = self.robot.transform.get_frame(i)
            for axis in range(3):
                co_ax = frame[:-1, 2*axis:2*(axis+1)].T
                self.frame_plots[axis, i], = self.ax1.plot(
                    co_ax[:, 0], co_ax[:, 1], co_ax[:, 2], linewidth=self.FRAME_LINE_WIDTH, color=self.robot.axis_colors[axis])

    def show_graphics(self):
        self.display_links()
        if self.frame_display:
            self.display_frames()
        if self.graph_display:
            self.display_graphs()
        self.fig.show()

    def update_graphics(self, time_stamp=None, theta=[[0], [0]], theta_d=[[0], [0]], control=[], pause_time=0.01):
        print(time_stamp)
        if time_stamp:
            self.time_text.set_text(
                f'time = {time_stamp}s')

            if self.graph_display:

                self.t.append(time_stamp)
                self.th_r.append(round(theta[0][0, 0], 4))
                self.th_a.append(round(theta[1][0, 0], 4))
                self.th_d_r.append(round(theta_d[0][0, 0], 4))
                self.th_d_a.append(round(theta_d[1][0, 0], 4))
                self.u.append(control[0, 0])

                if self.debug:
                    print("T :", self.t)
                    print("th_r :", self.th_r)
                    print("th :", self.th_a)
                    print("th_d_r :", self.th_d_r)
                    print("th_d :", self.th_d_a)
                    print("u :", self.u)

                self.ref_th_plot.remove()
                self.act_th_plot.remove()
                self.ref_th_d_plot.remove()
                self.act_th_d_plot.remove()
                self.control_plot.remove()

                self.display_graphs()

                # self.ref_th_plot = self.ax2.plot(self.t, self.th_r)[0]
                # self.act_th_plot = self.ax2.plot(self.t, self.th_a)[0]
                # self.ref_th_d_plot = self.ax3.plot(self.t, self.th_d_r)[0]
                # self.act_th_d_plot = self.ax3.plot(self.t, self.th_d_a)[0]
                # self.control_plot = self.ax4.plot(self.t, self.u)[0]

        # Update links
        for i in range(0, self.n):
            X, Y, Z = get_coordinates(self.robot.links[i])
            self.link_plots[i].set_data_3d(X, Y, Z)

        if self.frame_display:
            for i in range(0, self.n+1):
                frame = self.robot.transform.get_frame(i)
                for axis in range(3):
                    co_ax = frame[:-1, 2*axis:2*(axis+1)].T
                    self.frame_plots[axis, i].set_data_3d(
                        co_ax[:, 0], co_ax[:, 1], co_ax[:, 2])
        plt.draw()
        plt.pause(Trajectory.delta_t)


class SimpleObject:
    SHAPES = ['cube', 'sphere', 'cone', 'cylinder', 'cuboid']

    def __init__(self, name, dim=3, shape='cube', mass=5, length=0.1, width=0.1, height=0.1, gravity=-9.81, coef_res=0.9):
        self.name = name
        self.coef_res = coef_res
        self.id = uuid.uuid1()
        self.shape = shape
        self.length = length
        self.width = width
        self.height = height
        self.origin = np.eye(4)
        self.mass = mass
        self.position = np.zeros((dim, 1))
        self.velocity = np.zeros((dim, 1))
        self.acceleration = np.zeros((dim, 1))
        self.cg = np.array(
            [[self.length / 2], [self.width / 2], [self.height / 2]])
        self.weight = np.array([[0],
                                [0],
                                [mass*gravity]])

    def get_state(self):
        return self.position, self.velocity, self.acceleration

    def set_acceleration(self, acc):
        self.acceleration = acc

    def get_projected_area(self):
        if self.shape == 'cube':
            return self.length*self.length
        elif self.shape == 'sphere':
            return 0.25*np.pi*self.diameter**2

    def set_state(self, position, velocity=[], acceleration=[], time_stamp=0):
        idx = 0 if np.ndim(position) == 1 else [0]

        self.position[:, idx] = position

        if len(velocity) > 0:
            self.velocity[:, idx] = velocity

        if len(acceleration) > 0:
            self.acceleration[:, idx] = acceleration
        # print(
        #     f"STATE_{obj.name} at {time_stamp}s Position :{obj.position[2,0]} Velocity :{obj.velocity[2,0]} Acceleration : {obj.acceleration[2,0]}")

    def apply_force(self, forces):
        res_F = np.copy(self.weight)

        for force in forces:
            res_F += force

        self.set_acceleration(np.round(res_F/self.mass, 4))


class World:
    def __init__(self, name, dt=0.1, ground=[-np.inf, -np.inf, 0], air=[0, 0, 0], air_density=1.25, out=None):
        self.name = name
        self.__objects = {}
        self.n_ojects = 0
        self.time_stamp = 0
        self.dt = dt
        self.ground = np.array(ground).reshape(-1, 1)
        self.air_density = air_density
        self.air = np.zeros((3, 1))
        self.air[:, 0] = air
        self.air_res = True if len(air) > 0 else False
        print(f"Air resistance,Net force,Acceleration,Velocity,Position")

    def write_output(self, F_air, weight, acc, cur_vel, cur_pos):

        print(
            f"{tuple(F_air.flatten())},{tuple((F_air+weight).flatten())},{tuple(acc.flatten())},{tuple(cur_vel.flatten())},{tuple(cur_pos.flatten())}")

    def get_air_res(self, obj):
        F_air = 0.5*self.air_density*obj.get_projected_area()*(self.air-obj.velocity)**2
        print("F_air :", F_air)
        return np.round(F_air, 4)

    def set_objects(self, objects):
        for obj in objects:
            self.__objects[obj.id] = obj
            self.n_ojects += 1

    def get_first_object(self):
        return next(iter(self.get_objects()))

    def get_object(self, id):
        return self.__objects[id]

    def get_objects(self):
        return self.__objects.values()

    def on_object_state_change(self, id, position, *args):
        self.get_object(id).set_state(position, *args)

    def compute_state(self, dt, pre_pos, pre_vel, acc):
        cur_vel = pre_vel + acc * dt
        cur_pos = pre_pos + pre_vel * dt + acc * dt**2
        return cur_pos, cur_vel

    def update(self, time_stamp):
        self.time_stamp = time_stamp

        for obj in self.get_objects():
            F_air = self.get_air_res(obj)
            obj.apply_force(F_air)
            pre_pos, pre_vel, acc = obj.get_state()
            print(f"pre pos :{pre_pos}, pre vel :{pre_vel}, acc :{acc}")

            cur_pos, cur_vel = self.compute_state(
                self.dt, pre_pos, pre_vel, acc)

            if (cur_pos < self.ground).any():
                displacements = self.ground-pre_pos
                ax_collision = np.argmin(np.abs(displacements))

                # Compute the velocity before & after collision
                val = pre_vel[ax_collision, 0]**2 + 2 * \
                    acc[ax_collision, 0]*displacements[ax_collision, 0]

                if val > 0:

                    v_b_collision = np.sqrt(val)

                    # Handling the sign of velocity
                    v_b_collision = -1 * v_b_collision if pre_vel[ax_collision, 0] < 0 \
                        else v_b_collision

                    # Direction gets reversed after collision
                    v_a_collision = -obj.coef_res*v_b_collision
                    t_collision = (v_b_collision -
                                   pre_vel[ax_collision, 0])/acc[ax_collision, 0]

                    # Compute new velocity of the object along collision axis after the time stamp
                    p, v = self.compute_state(
                        (self.dt-t_collision), self.ground[ax_collision, 0], v_a_collision, acc[ax_collision, 0])
                    # print(
                    #     f"Collision at {round(time_stamp-self.dt+t_collision,4)}s Position :{self.ground[ax_collision, 0]} Vecolity: {v_a_collision} Acceleration: {acc[ax_collision, 0]}")

                    cur_pos[ax_collision, 0] = p
                    cur_vel[ax_collision, 0] = v
                else:
                    cur_vel[:, 0] = [0, 0, 0]

            self.write_output(F_air, obj.weight, acc, cur_vel, cur_pos)
            obj.set_state(cur_pos, cur_vel, time_stamp=time_stamp)


class Simulation(RoboPlotter):
    GRAPHIC_WINDOW_SIZE = (14, 7)

    def __init__(self, world, robot_model, window_size=GRAPHIC_WINDOW_SIZE):
        super().__init__(window_size, robot_model)
        self.plots = {}
        self.world = world
        self.window_size = window_size

        self.ax = self.get_3D_axis()
        self.create_objects()

    def create_objects(self):
        for obj in self.world.get_objects():
            # Create axis
            axes = [obj.length, obj.width, obj.height]
            data = np.ones(axes, dtype='bool')
            vox = self.ax.voxels(data, facecolors='red')
            self.plots[obj.id] = vox


class Simulation2():
    GRAPHIC_WINDOW_SIZE = (14, 7)

    def __init__(self, world, step_time=0.1, plot=False):
        self.plots = {}
        self.step_time = step_time
        self.world = world
        self.P = []
        self.t = []
        self.V = []

        if plot:
            self.fig = plt.figure(figsize=(5, 5))
            self.ax = self.fig.add_subplot(111, projection='3d')
            # viewrange for z-axis should be [-4,4]
            self.ax.set_zlim3d(-100, 500)
            self.ax.set_ylim3d(-100, 500)
            self.ax.set_xlim3d(-100, 500)
            # self.ax = self.fig.add_subplot(111)
            self.create_objects()
        self.fig.canvas.mpl_connect('close_event', exit)

    def exit(self):
        exit(0)

    def update(self, t):
        for obj in self.world.get_objects():
            self.P.append(obj.position[2, 0])
            self.t.append(t)
            self.V.append(obj.velocity[2, 0])
            # self.plots[obj.id].remove()
            print(*obj.position)
            self.plots[obj.id].set_data_3d(*obj.position)
            # self.plots[obj.id] = self.ax.plot(self.t, self.P, color='red')[0]
        plt.draw()
        plt.pause(self.step_time)

    def create_objects(self):

        for obj in self.world.get_objects():
            # self.plots[obj.id] = self.ax.plot(self.t, self.P, color='red')[0]
            self.plots[obj.id] = self.ax.plot(
                *obj.position, marker='o', markersize=20, color='red')[0]
            # Create axis
            # axes = [obj.length, obj.width, obj.height]
            # data = np.ones(axes, dtype=bool)
            # vox = self.ax.voxels(data, facecolors='red')
            # self.plots[obj.id] = vox

    def loop(self, T=30, infinity=False):
        t = 0

        def check_condition():
            if infinity:
                return True
            return t < T

        while check_condition():
            t = round(t + self.step_time, 4)
            # try:
            #     self.world.update(t)
            #     time.sleep(self.step_time)
            #     t = t + self.step_time
            # except Exception as e:
            #     print(e)
            #     break
            self.world.update(t)
            self.update(t)
            if (self.world.get_first_object().velocity[2, 0]) == 0:
                break


if __name__ == "__main__":
    DELTA_T = 0.1

    with open('out.csv', 'w') as f:
        world = World("empty_world", dt=DELTA_T, air=[-.03, 0, 0], out=f)
        obj = SimpleObject('obj1', length=1, width=1, height=1)
        obj.set_state([10, 10, 100], [
            0, 0, 0], [0, 0, -9.81])
        world.set_objects([obj])
        sim = Simulation2(world, DELTA_T, plot=True)
        sim.loop(infinity=True)
