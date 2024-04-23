# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# author：Teddy
# time:2023-09-07
# update time: 2024-04-18
# e-mail: teddy@pixmoving.net

import rclpy
from rclpy.node import Node
from pixmoving_hmi_msgs.msg import DisplayPlayCtrl 
from builtin_interfaces.msg import Time  

class DisplayPlayCtrlPublisher(Node): 
    def __init__(self, node):
        super().__init__('display_play_ctrl_publisher_node')
        self.node = node
        self.publisher_ = self.create_publisher(DisplayPlayCtrl, '/app/display/voice_display_play_ctrl', 10)
        self.timer = self.create_timer(1, self.publish_message)
        self.msg = DisplayPlayCtrl()                            # 初始化消息对象

    def publish_message(self):
        self.msg.timestamp = self.get_clock().now().to_msg()    # 当前时间戳
        self.publisher_.publish(self.msg)