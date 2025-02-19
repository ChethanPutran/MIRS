#import copy
import math
import rclpy
import numpy as np
# from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from mirs_controller.trajectory.control.controllers import ArmJointController,EEController
from mirs_controller.dynamics.dynamics import Dynamics
from mirs_system.conf.topics import TOPICS
from mirs_interfaces.msg import Trajectory

class TrajectoryFollower:
    def __init__(self,goal_time_tolerance=[0.05, 0.05, 0.05, 0.05, 0.05, 0.05]):
        self.dynamics = Dynamics()
        self.joints = self.robot.joints 
        self.Kp = np.zeros((self.robot.DOF),1)
        self.Kd = np.zeros((self.robot.DOF),1)
        self.Ki = np.zeros((self.robot.DOF),1)

        for idx,joint in enumerate(self.joints):
            self.Kp[idx]=joint.Kp
            self.Kd[idx]=joint.Kd
            self.Ki[idx]=joint.Ki
            
        self.joint_goal_tolerances = goal_time_tolerance
        self.trajectory_subscriber = self.create_subscription(Trajectory,TOPICS.TOPIC_TRAJECTORY,self.follow_and_act)

    def trajectory_is_finite(self,trajectory):
        """Check if trajectory contains infinite or NaN value."""
        for point in trajectory.points:
            for position in point.positions:
                if math.isinf(position) or math.isnan(position):
                    return False
            for velocity in point.velocities:
                if math.isinf(velocity) or math.isnan(velocity):
                    return False
        return True

    def has_velocities(self,trajectory):
        """Check that velocities are defined for this trajectory."""
        for point in trajectory.points:
            if len(point.velocities) != len(point.positions):
                return False
        return True

    def within_tolerance(self,a_vec, b_vec, tol_vec):
        """Check if two vectors are equals with a given tolerance."""
        for a, b, tol in zip(a_vec, b_vec, tol_vec):
            if abs(a - b) > tol:
                return False
        return True
   
    def reorder_trajectory_joints(self,trajectory, joint_names):
        """Reorder the trajectory points according to the order in joint_names."""
        order = [trajectory.joint_names.index(j) for j in joint_names]
        new_points = []
        for point in trajectory.points:
            new_points.append(JointTrajectoryPoint(
                positions=[point.positions[i] for i in order],
                velocities=[point.velocities[i] for i in order] if point.velocities else [],
                accelerations=[point.accelerations[i] for i in order] if point.accelerations else [],
                time_from_start=point.time_from_start))
        trajectory.joint_names = joint_names
        trajectory.points = new_points

    def start(self):
        """Initialize and start the action server."""
        self.init_trajectory()
        self.server.start()
        print("The action server for this driver has been started")

    def on_goal(self, goal_handle):
        """Handle a new goal trajectory command."""
        # Checks if the joints are just incorrect
        if set(goal_handle.get_goal().trajectory.joint_names) != set(self.prefixedJointNames):
            rospy.logerr("Received a goal with incorrect joint names: (%s)" %
                         ', '.join(goal_handle.get_goal().trajectory.joint_names))
            goal_handle.set_rejected()
            return

        if not trajectory_is_finite(goal_handle.get_goal().trajectory):
            rospy.logerr("Received a goal with infinites or NaNs")
            goal_handle.set_rejected(text="Received a goal with infinites or NaNs")
            return

        # Checks that the trajectory has velocities
        if not has_velocities(goal_handle.get_goal().trajectory):
            rospy.logerr("Received a goal without velocities")
            goal_handle.set_rejected(text="Received a goal without velocities")
            return

        # Orders the joints of the trajectory according to joint_names
        reorder_trajectory_joints(goal_handle.get_goal().trajectory, self.prefixedJointNames)

        # Inserts the current setpoint at the head of the trajectory
        now = self.robot.getTime()
        point0 = sample_trajectory(self.trajectory, now - self.trajectory_t0)
        point0.time_from_start = rospy.Duration(0.0)
        goal_handle.get_goal().trajectory.points.insert(0, point0)
        self.trajectory_t0 = now

        # Replaces the goal
        self.goal_handle = goal_handle
        self.trajectory = goal_handle.get_goal().trajectory
        goal_handle.set_accepted()

    def on_cancel(self, goal_handle):
        """Handle a trajectory cancel command."""
        if goal_handle == self.goal_handle:
            # stop the motors
            for i in range(len(TrajectoryFollower.jointNames)):
                self.motors[i].setPosition(self.sensors[i].getValue())
            self.goal_handle.set_canceled()
            self.goal_handle = None
        else:
            goal_handle.set_canceled()

    def update(self):
        if self.robot and self.trajectory:
            now = self.robot.getTime()
            if (now - self.trajectory_t0) <= self.trajectory.points[-1].time_from_start.to_sec() and \
               self.goal_handle:  # check if goal is still accepted
                # Sending intermediate points
                setpoint = sample_trajectory(self.trajectory, now - self.trajectory_t0)
                for i in range(len(setpoint.positions)):
                    self.motors[i].setPosition(setpoint.positions[i])
                    # Velocity control is not used on the real robot and gives bad results in the simulation
                    # self.motors[i].setVelocity(math.fabs(setpoint.velocities[i]))
            elif self.goal_handle and self.goal_handle.get_goal_status().status == actionlib_msgs.msg.GoalStatus.ACTIVE:
                # All intermediate points sent, sending last point to make sure we reach the goal.
                last_point = self.trajectory.points[-1]
                state = self.jointStatePublisher.last_joint_states
                position_in_tol = within_tolerance(state.position, last_point.positions, self.joint_goal_tolerances)
                setpoint = sample_trajectory(self.trajectory, self.trajectory.points[-1].time_from_start.to_sec())
                for i in range(len(setpoint.positions)):
                    self.motors[i].setPosition(setpoint.positions[i])
                    # Velocity control is not used on the real robot and gives bad results in the simulation
                    # self.motors[i].setVelocity(math.fabs(setpoint.velocities[i]))
                position_in_tol = within_tolerance(state.position, last_point.positions, [0.1] * 6)
                velocity_in_tol = within_tolerance(state.velocity, last_point.velocities, [0.05] * 6)
                if position_in_tol and velocity_in_tol:
                    # The arm reached the goal (and isn't moving) => Succeeded
                    self.goal_handle.set_succeeded()

    def follow_and_act(self,trajectory,action_sequence):
        print("Trajectory follower started...")
        error = None 
        status = True

        ee_controller = EEController()
        arm_controller = ArmJointController(self.joints,self.Kp,self.Kd,self.Ki)

        try:
            for point,time_stamp in zip(trajectory.get_trajectory_points(),trajectory.get_trajectory_times()):
                theta,theta_dot,theta_dotdot = point

                # Move joints with joint controller
                arm_res = arm_controller.forward(theta,theta_dot,theta_dotdot)
                ee_res = ee_controller.forward(action_sequence[time_stamp])

                if (not arm_res) or (not ee_res):
                    raise Exception("Could not move to point")
            
        except Exception as e:
            error = "Error in following trajectory"  + str(e)
            status = False
        return status,error

    
