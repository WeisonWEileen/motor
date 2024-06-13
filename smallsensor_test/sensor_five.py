#!/usr/bin/python3

import numpy as np
import struct
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


class ForceSensorNode(Node):
    def __init__(self):
        super().__init__('force_sensor_node')
        self.sensors = []
        self.publisherss = []
        num_sensors = 6  # 设置为需要的传感器数量
        x = 0
        for i in range(num_sensors):
            # 为每个传感器声明和获取IP地址和端口参数
            ip_addr = self.declare_parameter(f'IP_ADDR_{i}', f'192.168.0.{100+i + 5}').get_parameter_value().string_value
            print(ip_addr)

            port = 4008
            # 创建socket连接
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((ip_addr, port))
            self.sensors.append(sock)

            # 发送初始化命令
            # self.send_data(sock, "AT+SGDM=(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)\r\n")
            self.send_data(sock, "AT+GSD\r\n")

            # 创建对应的ROS publisher
            topic_name = f'force_sensor_{i}'
            pub = self.create_publisher(WrenchStamped, topic_name, 10)
            self.publisherss.append(pub)

            # 设置定时器，用于接收数据
            self.create_timer(0.01, lambda sock=sock, pub=pub: self.receive_data(sock, pub))

    def send_data(self, sock, data):
        sock.send(data.encode())
        recv_data = bytearray(sock.recv(1000))
        self.get_logger().info(str(recv_data))

    def receive_data(self, sock, pub):
        if not rclpy.ok():
            return
        # sock.send("AT+GOD\r\n".encode())
        data = sock.recv(1000)
        if len(data) < 30:
            return  # 检查数据完整性
        fx = struct.unpack("f", data[6:10])[0]
        fy = struct.unpack('f', data[10:14])[0]
        fz = struct.unpack('f', data[14:18])[0]
        mx = struct.unpack('f', data[18:22])[0]
        my = struct.unpack('f', data[22:26])[0]
        mz = struct.unpack('f', data[26:30])[0]

        sensor_msg = WrenchStamped()
        sensor_msg.header.stamp = self.get_clock().now().to_msg()
        sensor_msg.wrench.force.x = fx
        sensor_msg.wrench.force.y = fy
        sensor_msg.wrench.force.z = fz
        sensor_msg.wrench.torque.x = mx
        sensor_msg.wrench.torque.y = my
        sensor_msg.wrench.torque.z = mz
        pub.publish(sensor_msg)
        self.get_logger().info(f'Sensor data from {sock.getpeername()}: '
                        f'Fx={sensor_msg.wrench.force.x:.2f}, Fy={sensor_msg.wrench.force.y:.2f}, Fz={sensor_msg.wrench.force.z:.2f}, '
                        f'Mx={sensor_msg.wrench.torque.x:.2f}, My={sensor_msg.wrench.torque.y:.2f}, Mz={sensor_msg.wrench.torque.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    force_sensor_node = ForceSensorNode()
    rclpy.spin(force_sensor_node)
    for sock in force_sensor_node.sensors:
        sock.send("AT+GSD=STOP\r\n".encode())
        recv_data = bytearray(sock.recv(1000))
        force_sensor_node.get_logger().info(str(recv_data))
    force_sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# ros2 bag record -o sensor_1 /force_sensor_0 /force_sensor_1 /force_sensor_2 /force_sensor_3 /force_sensor_4  /force_sensor_big