# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# author：Teddy
# time:2023-09-07
# update time: 2024-04-18
# e-mail: teddy@pixmoving.net

import rclpy
from rclpy.duration import Duration
from dataclasses import dataclass
from autoware_adapi_v1_msgs.msg import (
    RouteState,
    MrmState,
    OperationModeState,
    MotionState,
    LocalizationInitializationState,
    VelocityFactorArray,
)
from tier4_debug_msgs.msg import Float64Stamped

from pix_robobus_driver_msgs.msg import (
    VcuReport1,                     # 车辆信息反馈
    BcmMessage4,                    # 安全带、座椅检测反馈
    VehicleDoorReport,              # 开关门反馈

)

from pixmoving_hmi_msgs.msg import (
    V2dCurrentStationInfo,          # 目标站点ID、车辆当前站点、下一站点、预计到达时间/min、预计到站距离/m
    V2dVehicleInfo,                 # 车辆信息
    DisplayReport,                  # 大屏反馈
    RemoteReport,                   # 远程驾驶反馈
)

from autoware_auto_system_msgs.msg import AutowareState
from autoware_auto_vehicle_msgs.msg import (
    VelocityReport,                 # 速度信息反馈
    TurnIndicatorsReport,           # 转向灯反馈
    ControlModeReport,              # 遥控切换模式反馈
)

from sensor_msgs.msg import Range   # 底盘超声波数据


@dataclass
class AutowareInformation:
    stop_reasons: list
    velocity_factors: list
    autoware_control: bool = False       
    remote_control_mode: int = 0                # remote control mode report
    operation_mode: int = 0                     # autoware control mode report   
    mrm_behavior: int = 0
    route_state: int = 0
    turn_signal: int = 0
    velocity: float = 0.0
    motion_state: int = 0
    localization_init_state: int = 0
    
    #------- pix add -------#
    vehicle_door: int = 0                       # 车门状态
    alarm_triggered: bool = False               # 报警触发
    alert_message: str = ""
    alarm_flag: bool = True
    autoware_state: int = 0
    _target_station_id: int = 0                 # 目标站点ID
    _current_station_id: int = 0                # 当前站点ID
    _target_station_name: str = ""              # 目标站名
    _current_station_name: str = ""             # 当前站名
    _estimate_time: float = 0.0                 # 预计到站时间    
    _target_distance: int = 0                   # 估算距离目标距离
    _co2_concentration_voltage: int = 0         # 二氧化碳传感器电压值; 整型数 S32; 单位 mV; -1:未知;   
    _smoke_concentration_voltage: int = 0       # 烟雾传感器电压值; 整型数 S32; 单位 mV; -1:未知;   
    _remote_mode_report: int = 0                # 远程驾驶模式初始化
    _remote_call_report: int = 0                # 远程通话初始化
    _display_status_report: int = 0             # 播放状态, [0 未播放、1正在播放]
    _display_progress_report: float = 0.0       # 播放进度, 0到100;
    _display_total_report: int = 0              # 视频总时间长度，单位ms;
    _display_volume_report: int = 0             # 音量，0到100;

    
    _ultra_sonic_radar_0: float = 0             # 底盘超声波数据 0-11
    _ultra_sonic_radar_1: float = 0    
    _ultra_sonic_radar_2: float = 0  
    _ultra_sonic_radar_3: float = 0    
    _ultra_sonic_radar_4: float = 0    
    _ultra_sonic_radar_5: float = 0  
    _ultra_sonic_radar_6: float = 0    
    _ultra_sonic_radar_7: float = 0    
    _ultra_sonic_radar_8: float = 0
    _ultra_sonic_radar_9: float = 0    
    _ultra_sonic_radar_10: float = 0    
    _ultra_sonic_radar_11: float = 0
        
    # ------- pixmoving end -------#

class AutowareInterface:
    def __init__(self, node):
        self._node = node
        self.information = AutowareInformation([], [])

        sub_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.SYSTEM_DEFAULT,
        )
        api_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Ros Sub       
        node.create_subscription(
            OperationModeState,
            "/api/operation_mode/state",
            self.sub_operation_mode_callback,
            api_qos,
        )
        node.create_subscription(
            RouteState,
            "/api/routing/state",
            self.sub_routing_state_callback,
            api_qos,
        )
        node.create_subscription(
            MrmState,
            "/api/fail_safe/mrm_state",
            self.sub_mrm_callback,
            sub_qos,
        )
        node.create_subscription(
            VelocityFactorArray,
            "/api/planning/velocity_factors",
            self.sub_velocity_factor_callback,
            sub_qos,
        )
        node.create_subscription(
            MotionState, 
            "/api/motion/state", 
            self.sub_motion_state_callback, 
            api_qos
        )
        
        node.create_subscription(
            LocalizationInitializationState,
            "/api/localization/initialization_state",
            self.sub_localization_initialization_state_callback,
            api_qos,
        )
        
        #---------- pixmoving add --------#
        node.create_subscription(
            AutowareState,
            "/autoware/state",
            self.sub_autoware_state_callback, # 订阅autoware状态
            sub_qos,
        )
        
        node.create_subscription(
            TurnIndicatorsReport,
            "/vehicle/status/turn_indicators_status",
            self.sub_vehicle_turn_indicators_callback, # 订阅转向灯状态
            sub_qos,
        )
        
        node.create_subscription(
            VelocityReport,
            "/vehicle/status/velocity_status",
            self.sub_vehicle_velocity_callback, # 订阅车辆速度
            sub_qos,
        )
        
        node.create_subscription(
            BcmMessage4,
            "/pix_robobus/bcm_message4",
            self.sub_vehicle_seat_belt_callback, # 安全带、座椅
            sub_qos,
        )
        
        node.create_subscription(
            VehicleDoorReport,
            "/pix_robobus/vehicle_door_report",
            self.sub_door_open_and_close_callback,      # 订阅开关门反馈
            sub_qos,
        )
        
        node.create_subscription(
            V2dCurrentStationInfo,
            "/app/display/current_station_info",
            self.sub_current_station_callback,    # 车辆站点信息回调函数
            sub_qos,
        )
        
        node.create_subscription(
            ControlModeReport,
            "/vehicle/status/control_mode",
            self.sub_remote_control_mode_callback, # 遥控模式反馈
            sub_qos,
        )
        
        node.create_subscription(
            V2dVehicleInfo,
            "/app/display/vehicle_info",
            self.sub_vehicle_info_callback,        # 车辆信息反馈
            sub_qos,
        )
        
        node.create_subscription(
            RemoteReport,
            "/app/voice/remote_report",             
            self.sub_remote_report_callback,       # 远程驾驶状态反馈
            sub_qos,
        )
        
        node.create_subscription(
            DisplayReport,
            "/app/voice/display_report",
            self.sub_display_report_callback,      # 大屏广告状态反馈
            sub_qos,
        )
        
        # --------- 底盘超声波反馈 ------------# 
        node.create_subscription(
            Range,
            "/sensing/ultra_sonic_radar/ultra_sonic_radar_0", # 超声波0
            self.sub_ultra_sonic_radar_0_callback,
            sub_qos,
        )
        
        node.create_subscription(
            Range,
            "/sensing/ultra_sonic_radar/ultra_sonic_radar_1", # 超声波1
            self.sub_ultra_sonic_radar_1_callback,
            sub_qos,
        )
        
        node.create_subscription(
            Range,
            "/sensing/ultra_sonic_radar/ultra_sonic_radar_2", # 超声波2
            self.sub_ultra_sonic_radar_2_callback,
            sub_qos,
        )
        
        node.create_subscription(
            Range,
            "/sensing/ultra_sonic_radar/ultra_sonic_radar_3", # 超声波3
            self.sub_ultra_sonic_radar_3_callback,
            sub_qos,
        )
        
        node.create_subscription(
            Range,
            "/sensing/ultra_sonic_radar/ultra_sonic_radar_4", # 超声波4
            self.sub_ultra_sonic_radar_4_callback,
            sub_qos,
        )
        
        node.create_subscription(
            Range,
            "/sensing/ultra_sonic_radar/ultra_sonic_radar_5", # 超声波5
            self.sub_ultra_sonic_radar_5_callback,
            sub_qos,
        )
        
        node.create_subscription(
            Range,
            "/sensing/ultra_sonic_radar/ultra_sonic_radar_6", # 超声波6
            self.sub_ultra_sonic_radar_6_callback,
            sub_qos,
        )
        
        node.create_subscription(
            Range,
            "/sensing/ultra_sonic_radar/ultra_sonic_radar_7", # 超声波7
            self.sub_ultra_sonic_radar_7_callback,
            sub_qos,
        )
        
        node.create_subscription(
            Range,
            "/sensing/ultra_sonic_radar/ultra_sonic_radar_8", # 超声波8
            self.sub_ultra_sonic_radar_8_callback,
            sub_qos,
        )
        
        node.create_subscription(
            Range,
            "/sensing/ultra_sonic_radar/ultra_sonic_radar_9", # 超声波9
            self.sub_ultra_sonic_radar_9_callback,
            sub_qos,
        )
        
        node.create_subscription(
            Range,
            "/sensing/ultra_sonic_radar/ultra_sonic_radar_10", # 超声波10
            self.sub_ultra_sonic_radar_10_callback,
            sub_qos,
        )
        
        node.create_subscription(
            Range,
            "/sensing/ultra_sonic_radar/ultra_sonic_radar_11", # 超声波11
            self.sub_ultra_sonic_radar_11_callback,
            sub_qos,
        )
        

        
        self._autoware_connection_time = self._node.get_clock().now()
        self._node.create_timer(2, self.reset_timer)

    def reset_timer(self):
        if self._node.get_clock().now() - self._autoware_connection_time > Duration(seconds=10):
            # self.information = AutowareInformation([], [])
            # self._node.get_logger().error("Autoware disconnected", throttle_duration_sec=10)
            pass

    # autoware mode report
    def sub_operation_mode_callback(self, msg):
        try:
            self.information.autoware_control = msg.is_autoware_control_enabled
            self.information.operation_mode = msg.mode
        except Exception as e:
            self._node.get_logger().error("Unable to get the operation mode, ERROR: " + str(e))
    
    # remote mode report
    def sub_remote_control_mode_callback(self,msg):
        try:
            self.information.remote_control_mode = msg.mode
            # print("debug----remote mode is : ", self.information.remote_control_mode)
        except Exception as e:
            self._node.get_logger().error("Can't get new remote control mode feedback, ERROR: " + str(e))

    def sub_routing_state_callback(self, msg):
        try:
            self.information.route_state = msg.state
        except Exception as e:
            self._node.get_logger().error("Unable to get the routing state, ERROR: " + str(e))

    def sub_mrm_callback(self, msg):
        try:
            self.information.mrm_behavior = msg.behavior
        except Exception as e:
            self._node.get_logger().error("Unable to get the mrm behavior, ERROR: " + str(e))

    def sub_vehicle_turn_indicators_callback(self, msg):
        try:
            self.information.turn_signal = msg.report    
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))
    
    def sub_vehicle_velocity_callback(self, msg):
        try:
            self.information.velocity = msg.longitudinal_velocity  

        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))  

    def sub_autoware_state_callback(self, msg):
        try:
            self.information.autoware_state = msg.state
            # if (self.information.autoware_state == AutowareState.INITIALIZING):
            #     print("初始化,请稍等")
            #     pass
            # if (self.information.autoware_state == AutowareState.WAITING_FOR_ROUTE):
            #     print("等待目标站点,请稍等")
            #     pass
            # if (self.information.autoware_state == AutowareState.PLANNING):
            #     print("路线规划中,请稍等")
            #     pass
            # if (self.information.autoware_state == AutowareState.WAITING_FOR_ENGAGE):
            #     print("已准备就绪")
            #     pass
            # if (self.information.autoware_state == AutowareState.DRIVING):
            #     print("自动驾驶中")
            #     pass
            # if (self.information.autoware_state == AutowareState.ARRIVED_GOAL):
            #     print("已到达目标站点")
            #     pass
            # if (self.information.autoware_state == AutowareState.FINALIZING):
            #     print("FINALIZING")
            #     pass
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))

    def sub_velocity_factor_callback(self, msg):
        try:
            self.information.velocity_factors = msg.factors
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))

    def sub_motion_state_callback(self, msg):
        try:
            self.information.motion_state = msg.state
        except Exception as e:
            self._node.get_logger().error("Unable to get the motion state, ERROR: " + str(e))

    def sub_localization_initialization_state_callback(self, msg):
        try:
            self.information.localization_init_state = msg.state
        except Exception as e:
            self._node.get_logger().error(
                "Unable to get the localization init state, ERROR: " + str(e)
            )
    
    def sub_vehicle_seat_belt_callback(self, msg):
        try:
            # 创建一个字典来存储座椅和安全带的状态
            seat_belt_status = {
                'seat1': (msg.safe_belt_sta1, msg.seat_sta1),
                'seat2': (msg.safe_belt_sta2, msg.seat_sta2),
                'seat3': (msg.safe_belt_sta3, msg.seat_sta3),
                'seat4': (msg.safe_belt_sta4, msg.seat_sta4),
                'seat5': (msg.safe_belt_sta5, msg.seat_sta5),
                'seat6': (msg.safe_belt_sta6, msg.seat_sta6)
            }
                
            # 遍历座椅状态并设置相应的标志
            self.information.alarm_flag = True
            for seat, (safe_belt, seat_status) in seat_belt_status.items():
                
                # 座位无人、未系安全带
                if not seat_status and  safe_belt:
                    pass
                # 座位无人、系安全带
                elif not seat_status and  not safe_belt:
                    pass
                # 座位有人、系安全带
                elif seat_status and not safe_belt:
                    pass
                # 座位有人、未系安全带
                elif seat_status and  safe_belt:
                    self.trigger_alarm(seat)

        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))


    def trigger_alarm(self, seat):
        self.information.alert_message = f"座位{seat[-1]}有人但未系安全带，触发报警！"
        # print("debug:",self.information.alert_message)
        self.information.alarm_flag = False


    def sub_door_open_and_close_callback(self,msg):
        try:   
            if(msg.door_open_inplace and (not msg.door_close_inplace)):
                # self.information.vehicle_door = msg.door_open_inplace
                self.information.vehicle_door = 1
                # print("开门状态")
            elif((not msg.door_open_inplace) and msg.door_close_inplace):
                # self.information.vehicle_door = msg.door_close_inplace
                self.information.vehicle_door = 2
                # print("关门状态")
            else:
                self.information.vehicle_door = 0
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))  


    # 车辆的站点信息
    def sub_current_station_callback(self,msg):
        try:
            self._autoware_connection_time = self._node.get_clock().now()
            self.information._current_station_name = msg.current_station.name
            self.information._current_station_id = msg.current_station.id
            self.information._target_station_name = msg.target_station.name
            self.information._target_station_id = msg.target_station.id
            self.information._estimate_time = msg.estimate_time   
            self.information._target_distance = msg.target_distance
  
        except Exception as e:
            self._node.get_logger().error("Unable to get the goal distance, ERROR: " + str(e))  
    
    
    # 车辆信息
    def sub_vehicle_info_callback(self,msg):
        try:
            self.information._co2_concentration_voltage = msg.co2_concentration_voltage
            self.information._smoke_concentration_voltage = msg.smoke_concentration_voltage

        except Exception as e:
            self._node.get_logger().error("Vehicle information not obtained, ERROR: " + str(e))  
            
    # 远程驾驶状态
    def sub_remote_report_callback(self,msg):
        try:
            self.information._remote_mode_report = msg.remote_mode
            self.information._remote_call_report = msg.remote_call
        except Exception as e:
            self._node.get_logger().error("Unable to read remote driving status, ERROR: " + str(e))  
    
    # 大屏广告状态
    def sub_display_report_callback(self,msg):
        try:
            self.information._display_status_report = msg.status
            self.information._display_progress_report = msg.progress
            self.information._display_total_report = msg.total
            self.information._display_volume_report = msg.volume
            
        except Exception as e:
            self._node.get_logger().error("Unable to read large screen ad status, ERROR: " + str(e))  
    
    # 底盘超声波数据       
    def sub_ultra_sonic_radar_0_callback(self,msg):
        try:
            self.information._ultra_sonic_radar_0 = msg.range
        except Exception as e:
            self._node.get_logger().error("Unable to read the vehicle ultrasonic 0 data, ERROR: " + str(e)) 
    
    def sub_ultra_sonic_radar_1_callback(self,msg):
        try:
            self.information._ultra_sonic_radar_1 = msg.range
        except Exception as e:
            self._node.get_logger().error("Unable to read the vehicle ultrasonic 1 data, ERROR: " + str(e)) 
    
    def sub_ultra_sonic_radar_2_callback(self,msg):
        try:
            self.information._ultra_sonic_radar_2 = msg.range
        except Exception as e:
            self._node.get_logger().error("Unable to read the vehicle ultrasonic 2 data, ERROR: " + str(e)) 
    
    def sub_ultra_sonic_radar_3_callback(self,msg):
        try:
            self.information._ultra_sonic_radar_3 = msg.range
        except Exception as e:
            self._node.get_logger().error("Unable to read the vehicle ultrasonic 3 data, ERROR: " + str(e)) 
    
    def sub_ultra_sonic_radar_4_callback(self,msg):
        try:
            self.information._ultra_sonic_radar_4 = msg.range
        except Exception as e:
            self._node.get_logger().error("Unable to read the vehicle ultrasonic 4 data, ERROR: " + str(e)) 
    
    def sub_ultra_sonic_radar_5_callback(self,msg):
        try:
            self.information._ultra_sonic_radar_5 = msg.range
        except Exception as e:
            self._node.get_logger().error("Unable to read the vehicle ultrasonic 5 data, ERROR: " + str(e)) 
    
    def sub_ultra_sonic_radar_6_callback(self,msg):
        try:
            self.information._ultra_sonic_radar_6 = msg.range
        except Exception as e:
            self._node.get_logger().error("Unable to read the vehicle ultrasonic 6 data, ERROR: " + str(e)) 
    
    def sub_ultra_sonic_radar_7_callback(self,msg):
        try:
            self.information._ultra_sonic_radar_7 = msg.range
        except Exception as e:
            self._node.get_logger().error("Unable to read the vehicle ultrasonic 7 data, ERROR: " + str(e)) 
    
    def sub_ultra_sonic_radar_8_callback(self,msg):
        try:
            self.information._ultra_sonic_radar_8 = msg.range
        except Exception as e:
            self._node.get_logger().error("Unable to read the vehicle ultrasonic 8 data, ERROR: " + str(e)) 
    
    def sub_ultra_sonic_radar_9_callback(self,msg):
        try:
            self.information._ultra_sonic_radar_9 = msg.range
        except Exception as e:
            self._node.get_logger().error("Unable to read the vehicle ultrasonic 9 data, ERROR: " + str(e)) 
    
    def sub_ultra_sonic_radar_10_callback(self,msg):
        try:
            self.information._ultra_sonic_radar_10 = msg.range
        except Exception as e:
            self._node.get_logger().error("Unable to read the vehicle ultrasonic 10 data, ERROR: " + str(e)) 
    
    def sub_ultra_sonic_radar_11_callback(self,msg):
        try:
            self.information._ultra_sonic_radar_11 = msg.range
        except Exception as e:
            self._node.get_logger().error("Unable to read the vehicle ultrasonic 11 data, ERROR: " + str(e)) 