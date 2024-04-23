
# Vehicle Voice Alert System

## Tested environments

| OS           | python     | ros           |
| ------------ | ---------- | ------------- |
| Ubuntu 20.04 | python3.8  | ros2 galactic |
| Ubuntu 22.04 | python3.10 | ros2 humble   |

## setup

### Install Autoware

refer to here

<https://autowarefoundation.github.io/autoware-documentation/main/installation/>

https://github.com/tier4/vehicle_voice_alert_system/tree/develop

pixmoving develop version

https://github.com/pixmoving-auto/vehicle_voice_alert_system/tree/feature/robobus

```bash
mkdir -p vehicle_voice_alert_system/src && cd vehicle_voice_alert_system/src
git clone https://github.com/pixmoving-auto/vehicle_voice_alert_system.git -b robobus

```

### Required library

- simpleaudio
- gtts
- pyudev
- pyaudio
- pygame
- pydub

```bash
cd vehicle_voice_alert_system/scripts
./install_libraries.sh

```

## build

```bash
cd ../..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select vehicle_voice_alert_system vehicle_voice_alert_system_version
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select vehicle_voice_alert_system vehicle_voice_alert_system_version

```

## start

```bash
source {AUTOWARE_PATH}/install/setup.bash
bash start.sh
```

## Sound definition

- The sound playing currently only support **wav** format

| Name           | Announce Timing                                                                                                                                                         | Priority (Descending order) | Trigger Source                                                                                       | Note                                                                                                 |
| -------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------- | ---------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| departure      | When the vehicle engage                                                                                                                                                 | 4                           | Service: /api/vehicle_voice/set/announce                                                             |                                                                                                      |
| emergency      | When emergency is trigger                                                                                                                                               | 4                           | Topic: /awapi/autoware/get/status<br /> Data: emergency                                              |                                                                                                      |
| in_emergency   | During emergency mode                                                                                                                                                   | 3                           | Topic: /awapi/autoware/get/status<br /> Data: emergency<br /> Flag: self.\_in_emergency_state        | Only trigger every mute_timeout during emergency                                                     |
| obstacle_stop  | When obstacle is detected                                                                                                                                               | 3                           | Topic: /awapi/autoware/get/status <br /> Data: stop_reason                                           | Only trigger after the mute_timeout period is over                                                   |
| restart_engage | when the vehicle restart the engage from stop, and velocity is more that 0                                                                                              | 4                           | Service: /api/vehicle_voice/set/announce<br /> Topic: /awapi/vehicle/get/status<br /> Data: velocity | Only trigger after the mute_timeout period is over                                                   |
| running_music  | The BGM to show that the vehicle is running. Trigger when autoware_state is Driving and will keep loop play while running.                                              | -                           | Topic: /awapi/autoware/get/status<br /> Data: autoware_state<br /> Flag: self.\_in_driving_state     | Independent from other announce. Will play together with other announce. Stop when in emergency mode |
| stop           | When the autoware_state is in following state and self.\_in_driving_state is True<br /> - WaitingForRoute<br /> - WaitingForEngage<br /> - ArrivedGoal<br /> - Planning | 4                           | Topic: /awapi/autoware/get/status<br /> Data: autoware_state<br /> Flag: not self.\_in_driving_state |                                                                                                      |
| temporary_stop | When vehicle stop in:<br /> - stop line<br /> - walkway<br /> - crosswalk<br /> - Merge from private road                                                               | 2                           | Topic: /awapi/autoware/get/status<br /> Data: stop_reason                                            | Only trigger after the mute_timeout period is over                                                   |
| turning_left   | When vehicle is turning left                                                                                                                                            | 1                           | Topic: /awapi/vehicle/get/status<br /> Data: turn_signal                                             | Trigger every mute_timeout when turning                                                              |
| turning_right  | When vehicle is turning right                                                                                                                                           | 1                           | Topic: /awapi/vehicle/get/status<br /> Data: turn_signal                                             | Trigger every mute_timeout when turning                                                              |

## License


voice and music:魔王魂

如使用在线gtts
Tools: https://ttsmaker.com/zh-cn


<!-- src
    announce controller 宣布控制器
    autoware interface autoware接口
    parameter interface 参数界面
    ros service interface Ros服务接口
    vehicle voice alert system 车辆语音报警系统 -->



## 声卡绑定配置
### 步骤一
### 本功能包基于的声卡是绿联：https://detail.tmall.com/item.htm?abbucket=12&id=559780789362&ns=1&spm=a21n57.1.0.0.748e523c5aANXB
### 本设备hidraw0、hidraw1为两个外接的USB声卡

#### 查看声卡是否接入
    lsusb
    Bus 001 Device 003: ID 0c76:1676 JMTek, LLC. USB PnP Audio Device
    Bus 001 Device 002: ID 0c76:1676 JMTek, LLC. USB PnP Audio Device
    ***
    ***
    ***

#### 出现的两个外置USB声卡串口号是一样的，是因为厂家采用的串口芯片是一样的；所以不能用串口号来区分两张声卡，接下来我们采用的是利用端口号加路径的方式来区分并绑定设备。
    ls /dev/hidraw*
    hidraw0 hidraw1 hidraw3 hidraw4

#### 查看串口以及端口信息,测试了发现没有USB串口独立的特征信息，所以无法从串口信息来识别, 唯一一个不变的3-3:1.0, 而这个实际上是代表电脑上这个USB口编号,也就是说只是连在这个USB端口上的串口都叫这个ID.

    udevadm info /dev/hidraw*

    P: /devices/pci0000:00/0000:00:14.0/usb1/1-3/1-3:1.3/0003:0C76:1676.0001/hidraw/hidraw0
    N: hidraw0
    L: 0
    E: DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-3/1-3:1.3/0003:0C76:1676.0001/hidraw/hidraw0
    E: DEVNAME=/dev/hidraw0
    E: MAJOR=239
    E: MINOR=0
    E: SUBSYSTEM=hidraw

    P: /devices/pci0000:00/0000:00:14.0/usb1/1-4/1-4:1.3/0003:0C76:1676.0002/hidraw/hidraw1
    N: hidraw1
    L: 0
    E: DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-4/1-4:1.3/0003:0C76:1676.0002/hidraw/hidraw1
    E: DEVNAME=/dev/hidraw1
    E: MAJOR=239
    E: MINOR=1
    E: SUBSYSTEM=hidraw

### 查看声卡所接的USB端口对应的端口号方法
    ll /sys/class/hidraw/hidraw*

    lrwxrwxrwx 1 root root 0  9月 11 09:33 /sys/class/hidraw/hidraw0 -> ../../devices/pci0000:00/0000:00:14.0/usb1/1-3/1-3:1.3/0003:0C76:1676.0001/hidraw/hidraw0/
    lrwxrwxrwx 1 root root 0  9月 11 09:33 /sys/class/hidraw/hidraw1 -> ../../devices/pci0000:00/0000:00:14.0/usb1/1-4/1-4:1.3/0003:0C76:1676.0002/hidraw/hidraw1/
    lrwxrwxrwx 1 root root 0  9月 11 09:33 /sys/class/hidraw/hidraw2 -> ../../devices/pci0000:00/0000:00:14.0/usb1/1-7/1-7:1.2/0003:046D:C52B.0005/hidraw/hidraw2/
    lrwxrwxrwx 1 root root 0  9月 11 09:33 /sys/class/hidraw/hidraw3 -> ../../devices/pci0000:00/0000:00:14.0/usb1/1-7/1-7:1.2/0003:046D:C52B.0005/0003:046D:404D.0006/hidraw/hidraw3/



### 绑定【USB端口号】KERNELS硬件端口号绑定
    sudo gedit /etc/udev/rules.d/com_port.rules

#### 根据自身情况，修改rules文件。    
#### KERNELS表示硬件的USB接口名,不同编号,表示不同的usb接口.
#### 其中，第一条代表连接到USB端口号为1-3:1.3的声卡对应的软链接文件为Speaker/Interior；第二条代表连接到USB端口号为1-4:1.3的声卡对应的软链接文件为Speaker/Exterior；然后保存并重启电脑。

    SUBSYSTEM=="hidraw", SUBSYSTEMS=="usb", KERNELS=="1-3:1.3", MODE="0666", SYMLINK+="Interior_Speaker", OWNER="pixkit3", GROUP="root"
    SUBSYSTEM=="hidraw", SUBSYSTEMS=="usb", KERNELS=="1-4:1.3", MODE="0666", SYMLINK+="Exterior_Speaker", OWNER="pixkit3", GROUP="root"

#### 运行以下命令使修改立即生效
    sudo udevadm trigger

#### 查看是否软连接成功，运行以下命令查看USB设备名更改情况：
    ls -l /dev |grep hidraw*

    lrwxrwxrwx   1 root    root           7  9月 11 11:15 Exterior_Speaker -> hidraw1
    crw-rw-rw-   1 pixkit3 pixkit3 239,   0  9月 11 11:15 hidraw0
    crw-rw-rw-   1 pixkit3 pixkit3 239,   1  9月 11 11:15 hidraw1
    crw-------   1 root    root    239,   2  9月 11 11:15 hidraw2
    crw-------   1 root    root    239,   3  9月 11 11:15 hidraw3
    lrwxrwxrwx   1 root    root           7  9月 11 11:15 Interior_Speaker -> hidraw0

### 如输出以上信息，表示已经软连接成功。
### 通过以上步骤，就把hidraw0取别名为Interior_Speaker，把hidraw1取别名为Exterior_Speaker,以后在程序里直接访问Interior_Speaker\Exterior_Speaker，就可以与这两个设备通信了。
### 特注意：只要是插到这个USB端口的串口都会被改为指定名，所以该方法只能将设备插入在指定的USB端口上。


### 步骤二
#### 配置alsad的asound.conf文件，如果没有则自行新建一个asound.conf文件，把Interior_Speaker、Exterior_Speaker 映射到声卡0、声卡1上，具体在那个需要 aplay -l查看。
    sudo gedit /etc/asound.conf

    pcm.Interior_Speaker {
        type hw
        card 0
    }

    ctl.Interior_Speaker {
        type hw
        card 0
    }


    pcm.Exterior_Speaker {
        type hw
        card 1
    }

    ctl.Exterior_Speaker {
        type hw
        card 1
    }

#### 保存配置文件即可


#### 查看系统声卡索引号
#### 在输出的内容里面就有我们插入的两张声卡。
    aplay -l or aplay -L

    **** List of PLAYBACK Hardware Devices ****
    card 0: Device_1 [USB PnP Audio Device], device 0: USB Audio [USB Audio]
    Subdevices: 1/1
    Subdevice #0: subdevice #0
    card 1: Device [USB PnP Audio Device], device 0: USB Audio [USB Audio]
    Subdevices: 1/1
    Subdevice #0: subdevice #0
    card 2: PCH [HDA Intel PCH], device 3: HDMI 0 [HDMI 0]
    Subdevices: 1/1


### 安装 pavucontrol 可视化工具
    sudo apt-get install pavucontrol   # Debian/Ubuntu系统

    按照下图所示进行设置
![Alt text](image.png)



## 更新声卡，之前采用的是同一厂家的USB声卡，所以串口序号是一样的，导致系统有时候不能够正常的去区分；所以现采用了不同厂家的USB声卡，教程如下
    lsusb
        Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
        Bus 001 Device 004: ID 04d8:0205 Microchip Technology, Inc. innodisk USB Dual CAN
        Bus 001 Device 012: ID 0c76:1676 JMTek, LLC. USB PnP Audio Device
        Bus 001 Device 013: ID 001f:0b21 Generic iStore Audio
        Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub

        # 可以看到Device012、013的 ID分别是 0c76:1676、001f:0b21，这样就可以容易的区别两张声卡设备了。
    
    sudo vim /etc/udev/rules.d/99-custom-usb-rules.rules

    # 把下面的代码加入到99-custom-usb-rules.rules文件当中。

        # Rule for Generic iStore Audio
        SUBSYSTEMS=="usb", ATTRS{idVendor}=="001f", ATTRS{idProduct}=="0b21", OWNER="pixkit"

        # Rule for JMTek USB PnP Audio Device
        SUBSYSTEMS=="usb", ATTRS{idVendor}=="0c76", ATTRS{idProduct}=="1676", OWNER="pixkit"

    # 保存之后，执行以下命令，重新加载udev规则
    
    sudo udevadm control --reload-rules


    # update ---- 20231211
    # update ---- 20240115
