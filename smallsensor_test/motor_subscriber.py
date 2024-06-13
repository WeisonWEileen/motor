import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

# from motor_con import MotorController


class Motor_subscriber(Node):

    def __init__(self):
        super().__init__('motor_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'motor_message',
            self.listener_callback,
            30)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Get: \n Motor position: %.2f rad; Motor speed: %.2f rad/s; Motor torque: %.2f Nm' % (msg.data[0], msg.data[1], msg.data[2]))

def main(args=None):
    rclpy.init(args=args)

    motor_subscriber = Motor_subscriber()

    rclpy.spin(motor_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()