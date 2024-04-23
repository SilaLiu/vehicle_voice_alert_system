# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8
import simpleaudio as sa
import time
import alsaaudio
import os
import subprocess   # 声卡选择

from os import path
from dataclasses import dataclass
from simpleaudio import WaveObject
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.time import Time
from pulsectl import Pulse
from autoware_adapi_v1_msgs.msg import (
    RouteState,
    MrmState,
    OperationModeState,
    MotionState,
    LocalizationInitializationState,
)
from std_msgs.msg import Float32
from tier4_hmi_msgs.srv import SetVolume
from tier4_external_api_msgs.msg import ResponseStatus
from autoware_auto_system_msgs.msg import AutowareState
from autoware_auto_vehicle_msgs.msg import ControlModeReport # 遥控切换模式反馈


PRIORITY_DICT = {
    "CN_robobus_zzqd_qzwfh_ndlcjjks": 5, # 车辆装备出发
    "CN_cljjddzd_qjbhndssxl": 4,         # 即将到站
    "CN_clydz_kmhqyxxc": 4,              # 车辆已到站
    "CN_Please_your_seat_belt": 3,       # 请系好安全带
    "CN_DoorOpen": 2,                    # 开门
    "CN_DoorClosed": 2,                  # 关门
    "CN_Turnleft": 1,                    # 左转
    "CN_Turnright": 1,                   # 右转
    
    "EN_star": 5,
    "EN_end": 4,
    "EN_The_Robobus_is_arriving": 4,
    "EN_clydz_kmhqyxxc": 4,
    "EN_Please_your_seat_belt": 3,
    "EN_please_seat_belts_fastened": 3,
    "EN_Obstacle_Detected_Emergency_Stop": 3,
    "EN_Door_Closing": 2,
    "EN_Door_Opening": 2,
    "EN_Turnleft": 1,
    "EN_Turnright": 1,
    
    "JP_robobus_zzqd_qzwfh_ndlcjjks": 5, # 车辆装备出发
    "JP_cljjddzd_qjbhndssxl": 4,         # 即将到站
    "JP_clydz_kmhqyxxc": 4,              # 车辆已到站
    "JP_Please_your_seat_belt": 3,       # 请系好安全带
    "JP_DoorOpen": 2,                    # 开门
    "JP_DoorClosed": 2,                  # 关门
    "JP_Turnleft": 1,                    # 左转
    "JP_Turnright": 1,                   # 右转
    
    
    "CN_Station1_start": 5,              # 站点
    "CN_Station2_end": 5,
    "CN_Station2_start": 5,
    "CN_Station3_end": 5,
    "CN_Station3_start": 5,
    "CN_Station4_end": 5,
    
    "EN_station1_depart": 5,
    "EN_station2_arrival": 5,
    "EN_station2_depart": 5,
    "EN_station3_arrival": 5,
    "EN_station3_depart": 5,
    "EN_station4_arrival": 5,
        
    
}

# 参照元: https://github.com/autowarefoundation/autoware_adapi_msgs/blob/main/autoware_adapi_v1_msgs/planning/msg/PlanningBehavior.msg
STOP_ANNOUNCE_BEHAVIORS = [
    "avoidance",
    "crosswalk",
    "goal-planner",
    "intersection",
    "lane-change",
    "merge",
    "no-drivable-lane",
    "no-stopping-area",
    "rear-check",
    "route-obstacle",
    "sidewalk",
    "start-planner",
    "stop-sign",
    "surrounding-obstacle",
    "traffic-signal",
    "user-defined-attention-area",
    "virtual-traffic-light",
]

CURRENT_VOLUME_PATH = "/opt/autoware/volume.txt"

@dataclass
class TimeoutClass:
    stop_reason: Time
    turn_signal: Time
    in_emergency: Time
    driving_bgm: Time
    accept_start: Time


class AnnounceControllerProperty:
    def __init__(
        self,
        node,
        ros_service_interface,
        parameter_interface,
        autoware_interface,
        display_play_ctrl_interface,
    ):
        super(AnnounceControllerProperty, self).__init__()
        self._node = node
        self._ros_service_interface = ros_service_interface
        self._parameter = parameter_interface.parameter
        self._mute_parameter = parameter_interface.mute_parameter
        self._autoware = autoware_interface
        self._display_play_ctrl = display_play_ctrl_interface
        
        self._timeout = TimeoutClass(
            node.get_clock().now(),
            node.get_clock().now(),
            node.get_clock().now(),
            node.get_clock().now(),
            node.get_clock().now(),
        )
        self._engage_trigger_time = self._node.get_clock().now()
        self._in_emergency_state = False
        self._prev_motion_state = MotionState.UNKNOWN
        self._current_announce = ""
        self._pending_announce_list = []
        self._wav_object = None
        self._music_object = None
        self._in_stop_status = False
        self._in_driving_state = False
        self._announce_arriving = False
        self._skip_announce = False
        self._stop_announce_executed = False
        self._announce_engage = False
        self._in_slow_stop_state = False

        # ------- pixmoving add -------#
        #---车门音频控制---
        self.vehicle_open_door_labels = False
        self.vehicle_close_door_labels =False
        self.current_door_state = None     
        
        #---转向音频控制---
        self.voice_turn_signal_l = False
        self.voice_turn_signal_r = False
        
        #---站点音频控制---
        self.voice_current_station = ""
        self.voice_next_station = ""
        self.voice_estimate_time = int
        self.voice_target_distance = int
        self.bgm_volume = int
        
        #---大屏广告控制---
        self.PLAY = 0
        self.PAUSE = 1
        self.STOP = 2
        self.SEEK = 3
        self.SET_VOLUME = 4
        self.display_play_cmd = int          # display_play_cmd    整数-播放相关指令,[0:播放开始(play);  1: 播放暂停(pause); 2: 播放停止(stop); 3: 快进到达(seek) ;4:音量设置(set_volume)]
        self.display_play_param = float      # display_play_param  参数值，浮点数, 只有在cmd为3(快进大)和4(音量设置)，才有意义.对于cmd为3，值范围为[0，100]，对于cmd为4，值范围为[0，100]；对于cmd为其他时值，1改值总算为0;

        #---声卡选择---
        self.sound_card_choice = ""
        
        # ------- pixmoving end -------#


        self._package_path = (
            get_package_share_directory("vehicle_voice_alert_system") + "/resource/sound"
        )

        self._running_bgm_file = self.get_filepath("robobus_bgm") # 获取BGM音频
        self._node.create_timer(0.5, self.check_playing_callback)                   # 检测是否有音频在播放
        self._node.create_timer(7.0, self.turn_signal_callback)                     # 检测转向灯信号
        self._node.create_timer(0.5, self.door_beeps_starting)                      # 检测车门状态
        self._node.create_timer(8.0, self.seat_belt_tips_callback)                  # 检测安全带状态
        self._node.create_timer(0.5, self.emergency_checker_callback)               # 检测是否处于紧急状态
        self._node.create_timer(0.5, self.stop_reason_checker_callback)             # 检测停止原因
        self._node.create_timer(0.1, self.announce_engage_when_starting)            # 启动时宣布启动
        
        self._node.create_timer(2.5, self.expected_arrival_reminders)               # 预计到站提醒
        # self._node.create_timer(5.0, self.vehicle_hazard_warning)                 # 车辆危险警告⚠️
        # self._node.create_timer(0.1, self.display_play_ctrl_publisher)              # 发布测试

     

        self._pulse = Pulse()
        if os.path.isfile(CURRENT_VOLUME_PATH):
            with open(CURRENT_VOLUME_PATH, "r") as f:
                self._sink = self._pulse.get_sink_by_name(
                    self._pulse.server_info().default_sink_name
                )
                self._pulse.volume_set_all_chans(self._sink, float(f.readline()))

        self._get_volume_pub = self._node.create_publisher(Float32, "~/get/volume", 1)
        self._node.create_timer(1.0, self.publish_volume_callback)
        self._node.create_service(SetVolume, "~/set/volume", self.set_volume)

    def set_timeout(self, timeout_attr):
        setattr(self._timeout, timeout_attr, self._node.get_clock().now())

    def reset_all_timeout(self):
        for attr in self._timeout.__dict__.keys():
            trigger_time = getattr(self._timeout, attr)
            duration = getattr(self._mute_parameter, attr)
            setattr(
                self._timeout,
                attr,
                self._node.get_clock().now() - Duration(seconds=duration),
            )

    def in_interval(self, timeout_attr):
        trigger_time = getattr(self._timeout, timeout_attr)
        duration = getattr(self._mute_parameter, timeout_attr)
        return self._node.get_clock().now() - trigger_time < Duration(seconds=duration)

    def check_in_autonomous(self):
        return self._autoware.information.operation_mode == OperationModeState.AUTONOMOUS

    def check_in_remote_control_mode(self):
        return self._autoware.information.remote_control_mode == ControlModeReport.AUTONOMOUS

    def get_filepath(self, filename):
        primary_voice_folder_path = (
            self._parameter.primary_voice_folder_path + "/" + filename + ".wav"
        )
        if path.exists(primary_voice_folder_path):
            return primary_voice_folder_path
        elif not self._parameter.skip_default_voice:
            return self._package_path + "/" + filename + ".wav"
        else:
            return ""

    def process_running_music(self):
        try:
            if not self._running_bgm_file:
                return

            if self.in_interval("driving_bgm"):
                return

            if (
                self._parameter.mute_overlap_bgm
                and self._wav_object
                and self._wav_object.is_playing()
            ):
                self.set_timeout("driving_bgm")
                return

            if (
                self.check_in_autonomous()
                # and self.check_in_remote_control_mode()
                # and not self._in_emergency_state
                and self._autoware.information.autoware_control
            ):
                if not self._announce_engage:
                    
                    self.display_play_cmd = self.SET_VOLUME     
                    self.display_play_param = 30.0
                    self.display_play_ctrl_publisher()
                    
                    self.departure_broadcast()
                    self._announce_engage = True
                    
                # 播放背景音乐    
                if (
                    not self._music_object 
                    or not self._music_object.is_playing() 
                    or not self._announce_engage
                ):
                    sound = WaveObject.from_wave_file(self._running_bgm_file)
                    self.set_bgm_volume(20)
                    
                    self.sound_card_choice = "Inside"   # 选择播放声卡
                    self.switch_audio_device()

                    self._music_object = sound.play()
                    self._announce_engage = True
                    
                
                if (
                    
                    self._autoware.information.autoware_state == AutowareState.DRIVING
                    and not self._announce_engage
                ):
                    
                    if self._parameter.sys_language == "CN":
                        self.send_announce("CN_clzdjsz_qzwfh")
                    elif self._parameter.sys_language == "JP":
                        self.send_announce("JP_clzdjsz_qzwfh")
                    elif self._parameter.sys_language == "EN":
                        self.send_announce("EN_clzdjsz_qzwfh") 
                    
                    # self.send_announce("CN_clzdjsz_qzwfh")
                    print("语音提示: 车辆自动驾驶中,请坐稳扶好")
                    self._announce_engage = True
                    
           
           # 人工驾驶时也播放背景音乐
            elif (
                self._parameter.manual_driving_bgm
                and not self._autoware.information.autoware_control
                and not self.in_range(
                    self._autoware.information.velocity,
                    self._parameter.driving_velocity_threshold,
                )
            ):
                if not self._music_object or not self._music_object.is_playing():
                    sound = WaveObject.from_wave_file(self._running_bgm_file)
                    self.set_bgm_volume(20)
                    
                    self.sound_card_choice = "Inside"   # 选择播放声卡
                    self.switch_audio_device()

                    self._music_object = sound.play()
                    
                    
                    self.display_play_cmd = self.SET_VOLUME     
                    self.display_play_param = 20.0
                    self.display_play_ctrl_publisher()
       
            else:
                if self._music_object and self._music_object.is_playing():
                    self._music_object.stop()
                    self.send_announce("CN_beep-beep")
                    

            if (
                self._autoware.information.route_state == RouteState.ARRIVED # 已到达
                and self._autoware.information.autoware_control
                and self._autoware.information.operation_mode == OperationModeState.STOP #  add
                and self._in_driving_state
            ):
                # Skip announce if is in manual driving 
                self.arrival_announcement()
                self._announce_arriving = False

            if self._autoware.information.route_state == RouteState.ARRIVED:
                self._skip_announce = False
                self._announce_engage = False

            self._in_driving_state = self.check_in_autonomous()
            self.set_timeout("driving_bgm")
        except Exception as e:
            self._node.get_logger().error(
                "not able to check the pending playing list: " + str(e),
                throttle_duration_sec=10,
            )

    def in_range(self, input_value, range_value):
        return -range_value <= input_value <= range_value

    def announce_engage_when_starting(self):
        try:
            if (
                self._autoware.information.localization_init_state
                == LocalizationInitializationState.UNINITIALIZED
            ):
                self._prev_motion_state = MotionState.UNKNOWN
                return

            if (
                self._autoware.information.motion_state
                in [MotionState.STARTING, MotionState.MOVING]
                and self._prev_motion_state == MotionState.STOPPED
            ):   
                self._stop_announce_executed = False
                if not self._skip_announce:
                    self._skip_announce = True
                elif self._node.get_clock().now() - self._engage_trigger_time > Duration(
                    seconds=self._mute_parameter.accept_start
                ):
                    # self.send_announce("CN_temporary_stop")
                    # print("语音提示: 临时停车")
                    self._engage_trigger_time = self._node.get_clock().now()
                self.reset_all_timeout()
                if self._autoware.information.motion_state == MotionState.STARTING:
                    self._service_interface.accept_start()

            # Check to see if it has not stopped waiting for start acceptance
            if self._autoware.information.motion_state != MotionState.STARTING:
                self.set_timeout("accept_start")

            # Send again when stopped in starting state for a certain period of time
            if (
                self._autoware.information.motion_state == MotionState.STARTING
                and self.in_interval("accept_start")
            ):
                self._service_interface.accept_start()

            self._prev_motion_state = self._autoware.information.motion_state
        except Exception as e:
            self._node.get_logger().error("not able to play the announce, ERROR: {}".format(str(e)))

    def check_playing_callback(self):
        try:
            self.process_running_music()
            if not self._wav_object:
                self._current_announce = ""
                return

            if not self._wav_object.is_playing():
                self._current_announce = ""
        except Exception as e:
            self._node.get_logger().error("not able to check the current playing: " + str(e))

    def play_sound(self, message):
        if (
            self._parameter.mute_overlap_bgm
            and self._music_object
            and self._music_object.is_playing()
        ):
            self._music_object.stop()

        filepath = self.get_filepath(message)
        if filepath:
            sound = WaveObject.from_wave_file(filepath)
            self._wav_object = sound.play()

        else:
            self._node.get_logger().info(
                "Didn't found the voice in the primary voice folder, and skip default voice is enabled"
            )

    # 发送声音提示 
    def send_announce(self, message):
        # if not self._autoware.information.autoware_control:
        #     self._node.get_logger().info("The vehicle is not control by autoware, skip announce")
        #     return

        priority = PRIORITY_DICT.get(message, 0)
        previous_priority = PRIORITY_DICT.get(self._current_announce, 0)

        if priority >= previous_priority:
            if self._wav_object:
                self._wav_object.stop()
            self.play_sound(message)
        self._current_announce = message

    # 检查是否处于紧急状态
    def emergency_checker_callback(self):
        if self._autoware.information.operation_mode == OperationModeState.STOP:
            in_emergency = False
        else:
            in_emergency = self._autoware.information.mrm_behavior == MrmState.EMERGENCY_STOP

        in_slow_stop = (
            self._autoware.information.mrm_behavior == MrmState.COMFORTABLE_STOP
            and self._autoware.information.motion_state == MotionState.STOPPED
        )

        if in_emergency and not self._in_emergency_state:
        #     self.send_announce("emergency")
        # elif in_emergency and self._in_emergency_state:
        #     if not self.in_interval("in_emergency"):
        #         self.send_announce("in_emergency")
        #         self.set_timeout("in_emergency")
        # elif in_slow_stop and self._in_slow_stop_state:
        #     if not self.in_interval("in_emergency"):
        #         self.send_announce("in_emergency")
        #         self.set_timeout("in_emergency")
            pass

        self._in_emergency_state = in_emergency
        self._in_slow_stop_state = in_slow_stop

    # 检查转向灯状态
    def turn_signal_callback(self):
        # if self.in_interval("turn_signal"):
        #     return
        # elif self._in_emergency_state or self._in_stop_status:
        #     return
        self.sound_card_choice = "Outside"   # 选择播放声卡
        self.switch_audio_device()

        
        if (self._autoware.information.turn_signal == 2):
            if self._parameter.sys_language == "CN":
                self.send_announce("CN_Turnleft")
            elif self._parameter.sys_language == "JP":
                self.send_announce("JP_Turnleft")
            elif self._parameter.sys_language == "EN":
                self.send_announce("EN_Turnleft")
 
            self.voice_turn_signal_l = True
        elif (self._autoware.information.turn_signal == 3):
            if self._parameter.sys_language == "CN":
                self.send_announce("CN_Turnright")
            elif self._parameter.sys_language == "JP":
                self.send_announce("JP_Turnright")
            elif self._parameter.sys_language == "EN":
                self.send_announce("EN_Turnright")    
            
            self.voice_turn_signal_r = True
            
        elif (self._autoware.information.turn_signal == 0):
            self.voice_turn_signal_l = False
            self.voice_turn_signal_r = False
        else:
            print("未获取到转向状态..")
            
        self.set_timeout("turn_signal")

    # 停止原因检测
    def stop_reason_checker_callback(self):
        if not self.check_in_autonomous():
            self._node.get_logger().warning(
                "The vehicle is not in driving state, do not announce",
                throttle_duration_sec=30,
            )
            return

        # skip when emergency stop
        if self._in_emergency_state:
            return

        if self._stop_announce_executed == True:
            return

        execute_stop_announce = False
        for velocity_factor in self._autoware.information.velocity_factors:
            if velocity_factor.behavior in STOP_ANNOUNCE_BEHAVIORS:
                execute_stop_announce = True
                break

        # 音声の通知
        if execute_stop_announce == True and self._autoware.information.motion_state == MotionState.STOPPED:
            if self.in_interval("stop_reason"):
                return
            
            # self.announce_stop_reason("CN_temporary_stop")
            self._stop_announce_executed = True
        else:
            self._in_stop_status = False

    def announce_stop_reason(self, file):
        self._in_stop_status = True
        self.send_announce(file)
        self.set_timeout("stop_reason")

    # 发布音量回掉函数
    def publish_volume_callback(self):
        self._sink = self._pulse.get_sink_by_name(self._pulse.server_info().default_sink_name)
        self._get_volume_pub.publish(Float32(data=self._sink.volume.value_flat))

    # 设置音量
    def set_volume(self, request, response):
        try:
            self._sink = self._pulse.get_sink_by_name(self._pulse.server_info().default_sink_name)
            self._pulse.volume_set_all_chans(self._sink, request.volume)
            with open(CURRENT_VOLUME_PATH, "w") as f:
                f.write(f"{self._sink.volume.value_flat}\n")
            response.status.code = ResponseStatus.SUCCESS
        except Exception:
            response.status.code = ResponseStatus.ERROR
        return response
     
    # 车门状态检测
    def door_beeps_starting(self):
        try:
            self.current_door_state = self._autoware.information.vehicle_door
            if self.current_door_state == 1 and (not self.vehicle_open_door_labels):
                if self._parameter.sys_language == "CN":
                    self.send_announce("CN_DoorOpen")
                elif self._parameter.sys_language == "JP":
                    self.send_announce("JP_DoorOpen")
                elif self._parameter.sys_language == "EN":
                    self.send_announce("EN_Door_Opening")  
                             
                print("语音提示: 车门已打开")
                self.vehicle_open_door_labels = True

            elif self.current_door_state == 2 and (not self.vehicle_close_door_labels):  
                if self._parameter.sys_language == "CN":
                    self.send_announce("CN_DoorClosed")
                elif self._parameter.sys_language == "JP":
                    self.send_announce("JP_DoorClosed")
                elif self._parameter.sys_language == "EN":
                    self.send_announce("EN_Door_Closing")

                print("语音提示: 车门已关闭")
                self.vehicle_close_door_labels = True
            
            elif self.current_door_state == 0:
                self.vehicle_open_door_labels = False
                self.vehicle_close_door_labels = False
            else:
                pass       
        except Exception as e:
            # 处理异常情况
            self._node.get_logger().error("未能获取车门状态消息: " + str(e))
    
    # 行驶状态下,座椅安全带提示
    def seat_belt_tips_callback(self):
        
        if (
            self._autoware.information.velocity > 2.0 
            or self._autoware.information.velocity < -2.0
        ):
        
            if (
                self.check_in_autonomous()
                or self.check_in_remote_control_mode()
                or not self._autoware.information.autoware_control
                
            ):
            
                if ( not self._autoware.information.alarm_flag ):
                    if self._parameter.sys_language == "CN":
                        self.send_announce("CN_Please_your_seat_belt")
                    elif self._parameter.sys_language == "JP":
                        self.send_announce("JP_Please_your_seat_belt")
                    elif self._parameter.sys_language == "EN":
                        self.send_announce("EN_please_seat_belts_fastened")

                    print("语音提示: 请系好安全带!")

    
    # 设置背景音乐音量
    def set_bgm_volume(self,volume):
        mixer = alsaaudio.Mixer()
        mixer.setvolume(volume)
    
    
    def expected_arrival_reminders(self):
        if (
            20 < self._autoware.information._target_distance
            and self._autoware.information._target_distance < self._parameter.announce_arriving_distance
            ):
                # announce if the goal is with the distance
                if self._parameter.sys_language == "CN":
                    self.send_announce("CN_cljjddzd_qjbhndssxl")
                elif self._parameter.sys_language == "JP":
                    self.send_announce("JP_cljjddzd_qjbhndssxl")
                elif self._parameter.sys_language == "EN":
                    self.send_announce("EN_The_Robobus_is_arriving")
              
                print("语音提示: 车辆即将到达站点,请准备好您的随身行李")



    # 出发广播：更具不同的场景更改音频名字以及站点ID
    def departure_broadcast(self):
        target_station_id = self._autoware.information._target_station_id
        print("出发ID: ",target_station_id)

    
        if target_station_id == 1:
            if self._parameter.sys_language == "CN":
                self.send_announce("CN_robobus_zzqd_qzwfh_ndlcjjks")
            if self._parameter.sys_language == "EN":
                self.send_announce("EN_star")
                
        if target_station_id == 2:
            if self._parameter.sys_language == "CN":
                self.send_announce("CN_Station1_start")
            if self._parameter.sys_language == "EN":
                self.send_announce("EN_station1_depart")

        elif target_station_id == 3:
            if self._parameter.sys_language == "CN":
                self.send_announce("CN_Station2_start")
            if self._parameter.sys_language == "EN":
                self.send_announce("EN_station2_depart")
                
        elif target_station_id == 4:
            if self._parameter.sys_language == "CN":
                self.send_announce("CN_Station3_start")
            if self._parameter.sys_language == "EN":
                self.send_announce("EN_station3_depart")

        else:
            if self._parameter.sys_language == "CN":
                self.send_announce("CN_robobus_zzqd_qzwfh_ndlcjjks")
            elif self._parameter.sys_language == "JP":
                self.send_announce("JP_robobus_zzqd_qzwfh_ndlcjjks")
            elif self._parameter.sys_language == "EN":
                self.send_announce("EN_star")


    # 到达广播：更具不同的场景更改音频名字以及站点ID
    def arrival_announcement(self):
        current_station_id = self._autoware.information._current_station_id
        
        print("到达ID: ",current_station_id)

        if current_station_id == 0:
            if self._parameter.sys_language == "CN":
                self.send_announce("CN_clydz_kmhqyxxc")
            if self._parameter.sys_language == "EN":
                self.send_announce("EN_end")
                
        if current_station_id == 1:
            if self._parameter.sys_language == "CN":
                self.send_announce("CN_clydz_kmhqyxxc")
            if self._parameter.sys_language == "EN":
                self.send_announce("EN_end")
                
        elif current_station_id == 2:
            if self._parameter.sys_language == "CN":
                self.send_announce("CN_Station2_end")
            if self._parameter.sys_language == "EN":
                self.send_announce("EN_station2_arrival")
                
        elif current_station_id == 3:
            if self._parameter.sys_language == "CN":
                self.send_announce("CN_Station3_end")
            if self._parameter.sys_language == "EN":
                self.send_announce("EN_station3_arrival")
            
        elif current_station_id == 4:
            if self._parameter.sys_language == "CN":
                self.send_announce("CN_Station4_end")
            if self._parameter.sys_language == "EN":
                self.send_announce("EN_station4_arrival")
                

        else:
            if self._parameter.sys_language == "CN":
                self.send_announce("CN_clydz_kmhqyxxc")
            elif self._parameter.sys_language == "JP":
                self.send_announce("JP_clydz_kmhqyxxc")
            elif self._parameter.sys_language == "EN":
                self.send_announce("EN_end")
                
    
    # 二氧化碳报警
    def vehicle_hazard_warning(self):
        if self._autoware.information._co2_concentration_voltage <= 270:
            # self.send_announce("beep_beep")
            pass
        
        
        if self._autoware.information._smoke_concentration_voltage >= 1:
            # self.send_announce("beep_beep")
            pass
    
    
    # 发布大屏广告控制  
    def display_play_ctrl_publisher(self):
        # 在这个函数中调用 display_play_ctrl_interface 中的方法
        if self.display_play_cmd == self.PLAY:
            self._display_play_ctrl.msg.cmd = self.PLAY
            self._display_play_ctrl.msg.param = 0.0
            
        elif self.display_play_cmd == self.PAUSE:
            self._display_play_ctrl.msg.cmd = self.PAUSE
            self._display_play_ctrl.msg.param = 0.0
            
        elif self.display_play_cmd == self.STOP:
            self._display_play_ctrl.msg.cmd = self.STOP
            self._display_play_ctrl.msg.param = 0.0
            
        elif self.display_play_cmd == self.SEEK:
            self._display_play_ctrl.msg.cmd = self.SEEK
            self._display_play_ctrl.msg.param = self.display_play_param
            
        elif self.display_play_cmd == self.SET_VOLUME:
            self._display_play_ctrl.msg.cmd = self.SET_VOLUME
            self._display_play_ctrl.msg.param = self.display_play_param
            
        else:
            self._display_play_ctrl.msg.cmd = 0
            self._display_play_ctrl.msg.param = 0.0
            
        self._display_play_ctrl.publish_message()
        
      
    def switch_audio_device(self):
        
        choice = self.sound_card_choice

        # 检测用户的输入并根据选择切换默认的音频输出设备
        if choice == 'Inside ':
            device_name = "alsa_output.usb-KTMicro_KT_USB_Audio_2021-04-13-0000-0000-0000--00.analog-stereo"
            print("已切换到 USB Audio --- Inside ")
        elif choice == 'Outside':
            device_name = "alsa_output.pci-0000_00_1f.3.analog-stereo"
            print("已切换到 92HD73C1X5 Analog --- Outside")
        else:
            return

        # 执行命令切换音频设备
        subprocess.run(["pacmd", "set-default-sink", device_name])


