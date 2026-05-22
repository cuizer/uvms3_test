#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import can
import struct
import threading
import time

# 引入自定义的 ROS 2 消息类型
from hal.msg import CanMsgIn, CanMsgOut

class HalCanBridgeNode(Node):
    def __init__(self):
        super().__init__('hal_can_bridge')
        self.get_logger().info(">>> [HAL] 正在初始化 AUV 天线 CAN 桥接器... <<<")

        # 1. 初始化 SocketCAN 物理网卡 (绑定 can0)
        try:
            # 兼容带有底层硬件环回和报错过滤的环境
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan', receive_own_messages=False)
            self.get_logger().info(">>> [HAL] 物理网卡 can0 绑定成功！(波特率: 125k) <<<")
        except Exception as e:
            self.get_logger().error(f"【网络硬伤】无法绑定物理网卡 can0: {str(e)}")
            self.get_logger().error("请检查是否执行了: sudo ip link set can0 up type can bitrate 125000")
            raise e

        # 2. 订阅 C++ 节点发出的底层数据 (出局链路)
        self.canout_sub = self.create_subscription(
            CanMsgOut,
            '/hal/canout',
            self.canout_callback,
            10
        )

        # 3. 发布从物理总线接收到的数据给 C++ 节点 (入局链路)
        self.canin_pub = self.create_publisher(CanMsgIn, '/hal/canin', 10)

        # 4. 开启独立的硬件监听线程，防止阻塞 ROS 主线程
        self.rx_thread = threading.Thread(target=self.can_rx_loop, daemon=True)
        self.rx_thread.start()

        # 5. 开启心跳定时器 (1Hz)
        self.timer = self.create_wall_timer(1.0, self.heartbeat_callback)
        self.get_logger().info(">>> [HAL] CAN 桥接器已就绪，双向通信链路已打通 <<<")

    def heartbeat_callback(self):
        """1Hz 状态自检，用于确认 Python 节点存活"""
        self.get_logger().info("【心跳正常】桥接器运行中，线程未死锁...")

    def canout_callback(self, msg: CanMsgOut):
        """
        核心回调：接收 C++ 节点下发的话题，完美透传给 Linux SocketCAN 驱动
        """
        try:
            # 1. 验证并提取底层数据
            arbitration_id = msg.id
            dlc = msg.dlc
            
            # 将 ROS 2 的 msg.data (通常为 vector/array) 转换为标准的 Python 字节流
            raw_data = bytes(msg.data[:dlc])

            # 2. 特殊日志：拦截并打印大数据指令（如 0xA4 / 0xA8 移动指令）
            if len(raw_data) > 0 and raw_data[0] in [0xA4, 0xA8]:
                # 解析出速度和目标位置（小端 32 位带符号整数）
                if len(raw_data) >= 8:
                    speed = struct.unpack('<H', raw_data[2:4])[0]
                    target_lsb = struct.unpack('<i', raw_data[4:8])[0]
                    self.get_logger().warn(f"【拦截大数据】发现控制指令 0x{raw_data[0]:02X} | 目标速度: {speed} | 目标位置: {target_lsb} LSB")

            # 3. 封装成 python-can 的底层 Message 对象
            can_msg = can.Message(
                arbitration_id=arbitration_id,
                data=raw_data,
                is_extended_id=False  # 天线电机使用 11 位标准帧 (0x141)
            )

            # 4. 强推至 Linux 物理网卡
            self.bus.send(can_msg)
            
            # 记录基础调试日志
            hex_str = " ".join(f"{b:02X}" for b in raw_data)
            self.get_logger().info(f"[TX -> can0] ID: 0x{arbitration_id:X} | 数据: [{hex_str}]")

        except can.CanError as e:
            self.get_logger().error(f"【硬件拒发】网卡 SocketCAN 拒绝发送报文！可能陷入 Bus-Off 或硬件短路。错误码: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"【解析异常】Python 回调函数内部处理失败 (捕获到未知溢出): {str(e)}")

    def can_rx_loop(self):
        """
        底层硬件监听线程：实时读取电机回传的所有数据（RX），并无缝打包发布给 ROS 2
        """
        self.get_logger().info("[RX 线程] 底层 SocketCAN 接收监听已拉起...")
        while rclcpp.ok():
            try:
                # 阻塞式读取物理网卡报文，超时时间 0.5 秒
                can_msg = self.bus.recv(0.5)
                if can_msg is None:
                    continue

                # 组装 ROS 2 消息
                ros_msg = CanMsgIn()
                ros_msg.id = can_msg.arbitration_id
                ros_msg.dlc = can_msg.dlc
                
                # 初始化 8 字节空数组并填入物理数据
                ros_msg.data = [0] * 8
                for i in range(min(can_msg.dlc, 8)):
                    ros_msg.data[i] = can_msg.data[i]

                # 顺着通道向上抛给 C++ 大脑
                self.canin_pub.publish(ros_msg)

            except Exception as e:
                self.get_logger().error(f"[RX 线程报错] 底层数据接收异常: {str(e)}")
                time.sleep(0.1)

def main(args=None):
    rclcpp.init(args=args)
    node = HalCanBridgeNode()
    try:
        rclcpp.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn(">>> [HAL] 检测到退出信号，天线桥接器正在安全关闭... <<<")
    finally:
        # 显式释放 SocketCAN 网卡资源
        if hasattr(node, 'bus'):
            node.bus.shutdown()
        node.destroy_node()
        rclcpp.shutdown()

if __name__ == '__main__':
    main()
