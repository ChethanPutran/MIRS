from mirs_interfaces.msg import MotorSensorFeedback
from mirs_controller.common.topics import TOPICS
from rclpy.node import Node

class JointSensor(Node):
    def __init__(self,name)-> None:
        super().__init__('Sensor')
        self.name = name
        self.state = MotorSensorFeedback()
        self.state_subscriber = self.create_subscription(MotorSensorFeedback,TOPICS.TOPIC_MOTOR_SENSOR_STATE,self.set_state,1)

    def get_position(self):
        return self.state
    
    def get_velocity(self):
        return self.state
    
    def set_state(self,msg):
        self.state.position = msg.position
        self.state.velocity = msg.velocity
        self.state.torque = msg.torque


class IMU(Node):
    def __init__(self,name)-> None:
        super().__init__('Sensor')
        self.name = name
        self.state = MotorSensorFeedback()
        self.state_subscriber = self.create_subscription(MotorSensorFeedback,TOPICS.TOPIC_MOTOR_SENSOR_STATE,self.set_state,1)

    def get_position(self):
        return self.state
    
    def get_velocity(self):
        return self.state
    
    def set_state(self,msg):
        imu_msg = Imu()
        imu_msg.header.frame_id = IMU_FRAME

        # Read the acceleration vals
        accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0
        accel_y = read_word_2c(ACCEL_YOUT_H) / 16384.0
        accel_z = read_word_2c(ACCEL_ZOUT_H) / 16384.0
        
        # Calculate a quaternion representing the orientation
        '''accel = accel_x, accel_y, accel_z
        ref = np.array([0, 0, 1])
        acceln = accel / np.linalg.norm(accel)
        axis = np.cross(acceln, ref)
        angle = np.arccos(np.dot(acceln, ref))
        orientation = quaternion_about_axis(angle, axis)'''

        # Read the gyro vals
        gyro_x = read_word_2c(GYRO_XOUT_H) / 131.0
        gyro_y = read_word_2c(GYRO_YOUT_H) / 131.0
        gyro_z = read_word_2c(GYRO_ZOUT_H) / 131.0
        
        # Load up the IMU message
        '''o = imu_msg.orientation
        o.x, o.y, o.z, o.w = orientation'''

        imu_msg.linear_acceleration.x = accel_x*9.8
        imu_msg.linear_acceleration.y = accel_y*9.8
        imu_msg.linear_acceleration.z = accel_z*9.8

        imu_msg.angular_velocity.x = gyro_x*0.0174
        imu_msg.angular_velocity.y = gyro_y*0.0174
        imu_msg.angular_velocity.z = gyro_z*0.0174

        imu_msg.header.stamp = rospy.Time.now()

        imu_pub.publish(imu_msg)
