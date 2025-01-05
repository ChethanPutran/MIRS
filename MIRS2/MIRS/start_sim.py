WINDOW_WIDTH = 600
WINDOW_HEIGHT = 600
N = 3

PRECISION = 4

if __name__ == '__main__':
    from console import Console
    from mirs import *

    con = Console(N, width=WINDOW_WIDTH,
                  height=WINDOW_HEIGHT, precision=PRECISION)
    robot_model = RobotModel(N)
    robot = Robot(robot_model, con, 0.1, precision=PRECISION)
    robot.start()
