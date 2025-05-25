import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# --- DH Transformation Matrix ---
def dh_transform(theta, d, a, alpha):
    theta = np.deg2rad(theta)
    alpha = np.deg2rad(alpha)
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

# --- Parameters ---
a1 = 1.0
a2 = 1.0
theta1_0 = 45
theta2_0 = 45

# Setup plot
fig = plt.figure()
ax = fig.add_subplot(111)
plt.subplots_adjust(left=0.25, bottom=0.25)
ax.set_aspect('equal')
ax.set_xlim(-2.5, 2.5)
ax.set_ylim(-2.5, 2.5)
ax.grid()
ax.set_title("2R Robot with Coordinate Frames")

# Robot links
[line] = ax.plot([], [], 'o-', lw=4, color='blue')

# Frame arrows (quivers)
quivers_x = []
quivers_y = []
for _ in range(3):  # base, joint1, joint2
    qx = ax.quiver(0, 0, 0, 0, color='r', angles='xy', scale_units='xy', scale=1)
    qy = ax.quiver(0, 0, 0, 0, color='g', angles='xy', scale_units='xy', scale=1)
    quivers_x.append(qx)
    quivers_y.append(qy)

# --- Update Function ---
def update(val):
    theta1 = slider_theta1.val
    theta2 = slider_theta2.val

    # DH transforms
    T0_1 = dh_transform(theta1, 0, a1, 0)
    T1_2 = dh_transform(theta2, 0, a2, 0)
    T0_2 = T0_1 @ T1_2

    # Origins
    o0 = np.array([0, 0])
    o1 = T0_1[0:2, 3]
    o2 = T0_2[0:2, 3]

    # Points to plot the robot
    x_data = [o0[0], o1[0], o2[0]]
    y_data = [o0[1], o1[1], o2[1]]
    line.set_data(x_data, y_data)

    # Frame directions (X and Y) and positions
    Ts = [np.eye(4), T0_1, T0_2]
    for i, T in enumerate(Ts):
        origin = T[0:2, 3]
        x_axis = T[0:2, 0]  # x-direction
        y_axis = T[0:2, 1]  # y-direction
        quivers_x[i].set_offsets(origin)
        quivers_x[i].set_UVC(x_axis[0], x_axis[1])
        quivers_y[i].set_offsets(origin)
        quivers_y[i].set_UVC(y_axis[0], y_axis[1])

    fig.canvas.draw_idle()

# Sliders
ax_theta1 = plt.axes([0.25, 0.15, 0.65, 0.03])
ax_theta2 = plt.axes([0.25, 0.1, 0.65, 0.03])
slider_theta1 = Slider(ax_theta1, 'Theta1 (°)', -180, 180, valinit=theta1_0)
slider_theta2 = Slider(ax_theta2, 'Theta2 (°)', -180, 180, valinit=theta2_0)
slider_theta1.on_changed(update)
slider_theta2.on_changed(update)

# Initial plot
update(None)
plt.show()
