# #!/bin/bash

# # 检测是否连接了 USB 音频设备
# usb_device_connected=false
# if lsusb | grep "KT USB Audio" > /dev/null; then
#     usb_device_connected=true
# fi

# # 检测是否存在集成的音频设备
# integrated_audio_device_connected=false
# # if lspci | grep -i "audio" | grep -i -E "intel|nvidia" > /dev/null; then  # 有显卡的情况下会出现两张声卡：主板+显卡
# if lspci | grep -i "audio" | grep -i "Intel" > /dev/null; then              # 只使用主板上的声卡
#     integrated_audio_device_connected=true
# fi

# # 根据连接的设备切换默认的音频输出设备
# if [ "$usb_device_connected" = true ] && [ "$integrated_audio_device_connected" = true ]; then
#     # USB 和集成的音频设备都连接着
#     pacmd set-default-sink alsa_output.usb-KT_USB_Audio-00.analog-stereo
#     echo "切换到 KT USB 音频设备"
# elif [ "$usb_device_connected" = false ] && [ "$integrated_audio_device_connected" = true ]; then
#     # 只有集成的音频设备连接着
#     pacmd set-default-sink alsa_output.pci-0000_00_1b.0.analog-stereo
#     echo "切换到 HDA Intel PCH"
# elif [ "$usb_device_connected" = true ] && [ "$integrated_audio_device_connected" = false ]; then
#     # 只有 USB 音频设备连接着
#     pacmd set-default-sink alsa_output.usb-KT_USB_Audio-00.analog-stereo
#     echo "切换到 KT USB 音频设备"
# else
#     echo "未检测到音频设备"
# fi



# #!/bin/bash

# # 显示选项菜单
# echo "请选择要使用的音频设备："
# echo "1. KT USB 音频设备"
# echo "2. HDA Intel PCH"

# # 接收用户的输入
# read -p "请输入选项(1 或 2): " choice

# # 检测用户的输入并根据选择切换默认的音频输出设备
# case $choice in
#     1)
#         pacmd set-default-sink alsa_output.usb-KT_USB_Audio-00.analog-stereo
#         echo "已切换到 KT USB 音频设备"
#         ;;
#     2)
#         pacmd set-default-sink alsa_output.pci-0000_00_1b.0.analog-stereo
#         echo "已切换到 HDA Intel PCH"
#         ;;
#     *)
#         echo "无效的选项,请重新运行脚本并输入有效选项(1 或 2)"
#         ;;
# esac



#!/bin/bash

# 显示选项菜单
echo "请选择要使用的音频设备："
echo "1. USB Audio"
echo "2. 92HD73C1X5 Analog"

# 接收用户的输入
read -p "请输入选项(1 或 2): " choice

# 检测用户的输入并根据选择切换默认的音频输出设备
case $choice in
    1)
        pacmd set-default-sink alsa_output.usb-KTMicro_KT_USB_Audio_2021-04-13-0000-0000-0000--00.analog-stereo
        echo "已切换到 USB Audio"
        ;;
    2)
        pacmd set-default-sink alsa_output.pci-0000_00_1f.3.analog-stereo
        echo "已切换到 92HD73C1X5 Analog"
        ;;
    *)
        echo "无效的选项,请重新运行脚本并输入有效选项(1 或 2)"
        ;;
esac
