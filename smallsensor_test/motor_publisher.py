import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

# from motor_con import MotorController
from motor_damiao import motor_con

class Motor_publisher(Node):

    def __init__(self):
        super().__init__('motor_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'motor_message', 30)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        motor_controller_data = MotorState()
        msg = Float64MultiArray()
        msg.data = [0.0,0.0,0.0]
        msg.data[0] = motor_controller_data.position
        msg.data[1] = motor_controller_data.speed
        msg.data[2] = motor_controller_data.tau

        self.publisher_.publish(msg)
        self.get_logger().info('Send: \n Motor position: %.2f rad; Motor speed: %.5f rad/s; Motor torque: %.2f Nm ' % (msg.data[0], msg.data[1], msg.data[2] ))
        # self.get_logger().info('fankui Send: \n Motor position: %.2f rad ' % (msg.data[0]))
        self.i += 1

def MotorState(args = None):
    motor = motor_con.Motor_damiao()
    motor.read_data()
    return motor


def main(args=None):
    rclpy.init(args=args)

    motor_publisher = Motor_publisher()
    rclpy.spin(motor_publisher)

    motor_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()