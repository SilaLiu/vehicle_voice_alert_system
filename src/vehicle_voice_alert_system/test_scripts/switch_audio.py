#!/usr/bin/env python3

import subprocess

def switch_audio_device():
    # 显示选项菜单
    print("请选择要使用的音频设备：")
    print("1. USB Audio")
    print("2. 92HD73C1X5 Analog")

    # 接收用户的输入
    choice = input("请输入选项(1 或 2): ")

    # 检测用户的输入并根据选择切换默认的音频输出设备
    if choice == '1':
        device_name = "alsa_output.usb-KTMicro_KT_USB_Audio_2021-04-13-0000-0000-0000--00.analog-stereo"
        print("已切换到 USB Audio")
    elif choice == '2':
        device_name = "alsa_output.pci-0000_00_1f.3.analog-stereo"
        print("已切换到 92HD73C1X5 Analog")
    else:
        print("无效的选项,请重新运行脚本并输入有效选项(1 或 2)")
        return

    # 执行命令切换音频设备
    subprocess.run(["pacmd", "set-default-sink", device_name])

if __name__ == "__main__":
    switch_audio_device()
