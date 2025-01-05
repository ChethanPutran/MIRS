import numpy as np
from robot import Robot
from robot_model import RobotModel
from transform import Transform
from console import Console
from simulation import Simulation, World, SimpleObject

np.set_printoptions(precision=4)

UNIT = 1  # in m
DEBUG = False
FRAME_SCALE = 2
FRAME_SIZE = FRAME_SCALE*0.025*UNIT
FRAME_LINE_WIDTH = 1
N = 3
SYMBOLIC = False

if __name__ == "__main__":
    robot_model = RobotModel(n=N, unit=UNIT)
    transform = Transform(robot_model=robot_model)
    console = Console(n_joints=N)
    robot = Robot(robot_model=robot_model, transform=transform, simp_sim=False)
    world = World("mirs world")
    obj = SimpleObject('box')
    world.set_objects([obj])
    simulation = Simulation(world, robot_model=robot)

    console.set_robot_params(robot)
    robot.on_exit(simulation.close)
    robot.on_simp_sim(simulation.start_simulation)
    robot.on_joint_move(simulation.update_graphics)
    robot.on_new_trajectory(simulation.plot_trajectory)

    console.on_new_program(robot.execute_program)
    console.on_manual_control(robot.move_joint)
    console.on_goal(robot.move_to_goal)
    console.on_exit(robot.exit)

    simulation.init_simulation()
    console.loop()
