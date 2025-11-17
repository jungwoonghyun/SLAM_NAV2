#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState
from message_interface.msg import Stepsub
from math import pi

angle = 0.0
linear = 0.0
ang_com = 0.0
lin_com = 0.0
iin_com_bef = 0.0
right_enc_val = 0.0
left_enc_val = 0.0
imu_use = False
imu_data = 0.0

class JointDriver(Node):
    def __init__(self):
        super().__init__('fake_driver')

        # joint states publisher
        self.pub_joint_states = self.create_publisher(JointState, "joint_states", 10)
        # cmd vel subscriber
        self.sub_cmd_vel = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        # encoder val subscriber
        self.enc_val = self.create_subscription(Stepsub, "motor_encoder_sub", self.enc_val_callback, 10)
        # vel raw publisher
        self.pub_vel_raw = self.create_publisher(TwistStamped, "vel_raw", 10)

        # init variable
        self.joint_states = JointState()
        self.joint_states.header.frame_id = "joint_states"
        self.joint_states.name = ["left_wheel_joint", "right_wheel_joint"]
        self.joint_states.position = [0.0, 0.0]

        self.vel_raw = TwistStamped()

        # timer
        self.timer = self.create_timer(0.025, self.publish_callback)

    def enc_val_callback(self, msg):
        global right_enc_val
        global left_enc_val
        global imu_use
        global imu_data

 #       self.vel_raw.twist = msg
        right_enc_val = msg.rightwheel_step
        left_enc_val = msg.leftwheel_step
        imu_use = msg.use_imu
        imu_data = msg.imu_yaw

    def cmd_vel_callback(self, msg):
        global ang_com
        global lin_com
        self.get_logger().info(f'recv message {msg}')
        lin_com = msg.linear.x
        ang_com = msg.angular.z

    def publish_callback(self):
        global angle
        global linear
        global right_enc_val
        global left_enc_val
        global iin_com_bef
        linear_scale = 120.0 * (9.0/7.0) # wheel rotate (radian, 2pi / 0.508pi -> 1m/s)
        angle_scale = 120.0 * (9.0/7.0) * (0.47 / 2.0); # car rotate (radian, m_cir_dia * theta / 2 * linear_scale) # tity = 0.455, rooty = 0.48
        dt = 0.025
        curr_time = self.get_clock().now()

        # joint states
        self.joint_states.header.stamp = curr_time.to_msg()

        # vel raw
        self.vel_raw.header.stamp = curr_time.to_msg()

        # wheel position value
        right_wheel_rotate = (right_enc_val / 120.0) * (2.0 * pi)
        left_wheel_rotate = (left_enc_val / 120.0) * (2.0 * pi)

        # wheel speed value
        step_spd_right = right_enc_val / dt
        step_spd_left = left_enc_val / dt

        linear = (step_spd_right + step_spd_left) / (2.0 * linear_scale) #* 1.0011

        angle = (step_spd_right - step_spd_left) / (2.0 * angle_scale)
        # if (linear < 0.001) and (linear > -0.001):
        #     if ang_com < 0.0:
        #         angle = ((step_spd_right - step_spd_left) / (2.0 * angle_scale)) * 1.00310
        #     elif ang_com > 0.0 :
        #         angle = ((step_spd_right - step_spd_left) / (2.0 * angle_scale)) * 1.00632
        #     else :
        #         angle = (step_spd_right - step_spd_left) / (2.0 * angle_scale)
        # else :
        #     if iin_com_bef > -0.001 and iin_com_bef < 0.001 :
        #         angle = ((step_spd_right - step_spd_left) / (2.0 * angle_scale))
        #         # if ang_com < 0.0:
        #         #     angle = ((step_spd_right - step_spd_left) / (2.0 * angle_scale)) + 0.001250 #
        #         # elif ang_com > 0.0 :
        #         #     angle = ((step_spd_right - step_spd_left) / (2.0 * angle_scale)) + 0.001250 #
        #         # else:
        #         #     angle = ((step_spd_right - step_spd_left) / (2.0 * angle_scale)) + 0.001250 # 3750
        #     else :
        #         angle = ((step_spd_right - step_spd_left) / (2.0 * angle_scale))
        #         # if ang_com < 0.0:
        #         #     angle = ((step_spd_right - step_spd_left) / (2.0 * angle_scale)) * 1.00310 #1.00310
        #         # elif ang_com > 0.0 :
        #         #     angle = ((step_spd_right - step_spd_left) / (2.0 * angle_scale)) * 1.00632 #1.00632
        #         # else :
        #         #     angle = ((step_spd_right - step_spd_left) / (2.0 * angle_scale)) #

        iin_com_bef = lin_com

        self.vel_raw.twist.linear.x = linear
        self.vel_raw.twist.angular.z = angle

        self.pub_joint_states.publish(self.joint_states)
        self.pub_vel_raw.publish(self.vel_raw)

        # simulate wheel rotate
        self.joint_states.position[0] += left_wheel_rotate
        self.joint_states.position[1] += right_wheel_rotate

def main(args=None):
    rclpy.init(args=args)

    driver = JointDriver()
    executor = MultiThreadedExecutor()
    rclpy.spin(driver, executor=executor)

    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
