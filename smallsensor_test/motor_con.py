import rclpy
from rclpy.node import Node
import can
import struct
import time
import math
from math import pi
import threading
import ctypes
# from motor_control_mode import MotorController
import pandas as pd
import csv

class Motor_damiao(Node):
    def __init__(self):
        super().__init__('motor_damiao')
        self.motor_id = 0x100 + 0x01  # 位置速度模式需要加偏置
        self.motor_master_id = 0x91  # 根据实际情况调整ID
        self.bus0 = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
        self.error_state = 0
        self.speed = 0
        self.position = 0
        self.tau = 0

        self.maxspeed = 30
        self.maxtau = 20
    
    def read_data(self):
        read_encoder_command = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        read_encoder_frame = can.Message(arbitration_id=self.motor_master_id, data=read_encoder_command, is_extended_id=False)
        
        self.bus0.send(read_encoder_frame)
        message = self.bus0.recv(1)  # 设置超时时间为1秒

        if message is not None:
            
            # print(str(message.data[3]),str(message.data[2]),str(message.data[1]),str(message.data[0]))
            # print("\n")
            # 读取数据消息
            motor_position_bytes = [message.data[3], message.data[2], message.data[1], message.data[0]]
            raw_motor_position = struct.unpack('>f', bytes(motor_position_bytes))[0]  # 电机位置
            # raw_motor_position = sum(byte << (8 * i) for i, byte in enumerate(motor_position_bytes))
            # motor_position = float(raw_motor_position)
            raw_motor_speed = (message.data[4] & 0xFF) + ((message.data[5] & 0xFF) << 8)  # 电机速度
            raw_motor_tau = (message.data[6] & 0xFF) + ((message.data[7] & 0xFF) << 8)  # 电机转矩

            motor_position  = raw_motor_position

            motor_speed = raw_motor_speed / 65536 * self.maxspeed
            if motor_speed > self.maxspeed / 2:
                motor_speed = motor_speed
            else:
                motor_speed = -motor_speed

            motor_tau = raw_motor_tau / 65536 * self.maxtau
            if motor_tau > self.maxtau / 2:
                motor_tau = motor_tau
            else:  
                motor_tau = -motor_tau
            
            self.position = motor_position
            self.speed = motor_speed
            self.tau = motor_tau
            

    def change_zero_point(self):
        # 手动控制电机到零点位置
        # ... 执行手动控制电机的操作 ...
        write_command = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
        write_frame = can.Message(arbitration_id=self.motor_id, data=write_command, is_extended_id=False)
        self.bus0.send(write_frame)
        self.get_logger().info('Motor initial position change command sent')

    #reset bus
    def reset_bus(self):
        self.bus0.shutdown()
        self.bus0 = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)

    def set_position(self, angle, speed):

        data = [0] * 8
        # 将 position 转换为字节序列
        position_bytes = struct.pack('<f', angle)

        # 将 speed 转换为字节序列
        speed_bytes = struct.pack('<f', speed)
        print(str(position_bytes))

        # 将 position_bytes 拆分成四个字节，并存储到 message.data[0] 到 message.data[3] 中
        for i in range(4):
            data[i] = position_bytes[i]

        # 将 speed_bytes 拆分成四个字节，并存储到 message.data[4] 到 message.data[7] 中
        for i in range(4):
            data[i + 4] = speed_bytes[i]

        print(data)
        # 创建并发送CAN消息
        position_control_frame = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)
        self.bus0.send(position_control_frame)
        self.get_logger().info(f'Position control command sent for motor {self.motor_id}')
        # 读取返回的消息
        data_message = self.bus0.recv(1)  # 设置超时时间为1秒

    def run_motor(self):
        run_command = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
        run_frame = can.Message(arbitration_id=self.motor_id, data=run_command, is_extended_id=False)
        self.bus0.send(run_frame)
        self.get_logger().info('Motor run command sent')

    def stop_motor(self):
        stop_command = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
        stop_frame = can.Message(arbitration_id=self.motor_id, data=stop_command, is_extended_id=False)
        self.bus0.send(stop_frame)
        self.get_logger().info('Motor stop command sent')
    
    def __del__(self):
        self.bus0.shutdown()
    
def main(args=None):
    rclpy.init(args=args)
    motor = Motor_damiao()

    t = 0.0
    T = 1.2
    speed1 = 0.1  #rad/s
    angle1 = 270 * pi / 180

    motor.run_motor()
    motor.reset_bus()
    motor.change_zero_point()
    n = 0
    t1 = 0.1
    while rclpy.ok():
        motor.set_position(angle1,speed1)  # leg_1
        # motor.set_position(-angle1,speed1)  # leg_2
        time.sleep(t1)
        t += t1

        if t >= t1:
            break
    # motor.change_zero_point()
    # motor.set_position(100,speed1)
    # time.sleep(30)
    # motor.set_position(0,speed1)
    # time.sleep(30)
    motor.stop_motor()
        
    motor.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

#every time open a terminal need to use this order
#source ./install/setup.bash

#build the workspace after changing the codes
#colcon build --packages-select motor_test

#run the motor test
#ros2 run motor_test motor_controller 

#start the network of can0 and can1
#sudo ip link set can0 up type can bitrate 1000000
#sudo ip link set can1 up type can bitrate 1000000
    
#close the network of can0
#sudo ip link set can0 down

#source install/local_setup.sh

#ros2 bag record -o motor_file /motor_message
