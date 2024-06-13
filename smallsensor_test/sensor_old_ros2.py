#!/usr/bin/python3

from ast import Param
import numpy as np
import struct
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import WrenchStamped


class ForceSensorNode(Node):
    def __init__(self):
        super().__init__('force_sensor_node')
        self.topic_name = self.declare_parameter('topic_name','force_sensor_node').get_parameter_value().string_value
        self.pub = self.create_publisher(WrenchStamped, self.topic_name, 10)
        self.sensor_msg = WrenchStamped()
        self.ip_addr = self.declare_parameter('IP_ADDR','192.168.0.110').get_parameter_value().string_value
        self.port = 4008

        # 创建连接插口
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 连接
        self.socket.connect((self.ip_addr, self.port))

        # # 查看连接地址
        # self.send_data("AT+EIP=?\r\n")

        # # 查看解耦矩阵
        # self.send_data("AT+DCPM=?\r\n")

        # shezhi host
        # self.send_data("AT+EIP=192.168.0.110\r\n")

        # cha xun host
        self.send_data("AT+EIP=?\r\n")

        # 查看计算单位
        # self.send_data("AT+DCPCU=?\r\n")

        # 设置传感器参数
        # 设置解耦矩阵，仅需要设置一次
        # decouple_matrix = "(0.74447,-0.10432,-4.05718,113.20804,-0.19615,-112.78654);" \
        #                   "(-1.55915,-131.20568,-12.28236,63.88653,2.47576,65.56931);" \
        #                   "(320.82506,-6.00990,322.04951,1.83496,324.31629,-3.44262);" \
        #                   "(0.13401,0.15065,-11.66469,-0.15551,11.42006,-0.23770);" \
        #                   "(13.19140,-0.21842,-6.52051,0.03898,-6.59893,0.00286);" \
        #                   "(-0.01730,4.59975,-0.19084,4.56898,0.06837,4.54050)\r\n"
        # set_decouple_matrix = "AT+DCPM=" + decouple_matrix
        # self.send_data(set_decouple_matrix)

        # # 设置采样频率
        self.send_data("AT+SMPR=10\r\n")

        # # 设置矩阵运算单位
        # self.send_data("AT+DCPCU=MVPV\r\n")

        # 上传数据格式
        self.send_data("AT+SGDM=(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)\r\n")

        # 连续上传数据包
        self.send_data("AT+GSD\r\n")

        # 开始接收数据
        self.timer = self.create_timer(0.01, self.receive_data)

    def send_data(self, data):
        self.socket.send(data.encode())
        recv_data = bytearray(self.socket.recv(1000))
        self.get_logger().info(str(recv_data))

    def receive_data(self):
        if not rclpy.ok():
            return
        data = self.socket.recv(1000)
        fx = struct.unpack("f", data[6:10])[0]
        fy = struct.unpack('f', data[10:14])[0]
        fz = struct.unpack('f', data[14:18])[0]
        mx = struct.unpack('f', data[18:22])[0]
        my = struct.unpack('f', data[22:26])[0]
        mz = struct.unpack('f', data[26:30])[0]
        self.sensor_msg.header.stamp = self.get_clock().now().to_msg()
        self.sensor_msg.wrench.force.x = fx
        self.sensor_msg.wrench.force.y = fy
        self.sensor_msg.wrench.force.z = fz
        self.sensor_msg.wrench.torque.x = mx
        self.sensor_msg.wrench.torque.y = my
        self.sensor_msg.wrench.torque.z = mz
        self.pub.publish(self.sensor_msg)
        self.get_logger().info('\n %.2f %.2f %.2f %.2f %.2f %.2f' % (fx, fy,fz, mx, my, mz))

def main(args=None):
    rclpy.init(args=args)
    force_sensor_node = ForceSensorNode()
    rclpy.spin(force_sensor_node)
    force_sensor_node.socket.send("AT+GSD=STOP\r\n".encode())
    print("=----------------------------------")
    recv_data = bytearray(force_sensor_node.socket.recv(1000))
    force_sensor_node.get_logger().info(str(recv_data))
    force_sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
