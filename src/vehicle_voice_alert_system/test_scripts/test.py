# import threading
# import time
# import wave
# import pyaudio

# def play_audio(filename, device_index):
#     CHUNK = 1024

#     wf = wave.open(filename, 'rb')

#     p = pyaudio.PyAudio()
#     stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
#                     channels=wf.getnchannels(),
#                     rate=wf.getframerate(),
#                     output=True,
#                     output_device_index=device_index)

#     data = wf.readframes(CHUNK)

#     while data:
#         stream.write(data)
#         data = wf.readframes(CHUNK)

#     stream.stop_stream()
#     stream.close()
#     p.terminate()

# def play_audio_threaded(filename, device_index):
#     thread = threading.Thread(target=play_audio, args=(filename, device_index))
#     thread.start()

# if __name__ == "__main__":
#     # 音频文件路径
#     audio1 = '/home/pixbus/pix/robobus/autoware-robobus/src/HMI/pixmoving/vehicle_voice_alert_system/src/vehicle_voice_alert_system/resource/bgm/chill_abstract_intention.wav'
#     audio2 = '/home/pixbus/pix/robobus/autoware-robobus/src/HMI/pixmoving/vehicle_voice_alert_system/src/vehicle_voice_alert_system/resource/bgm/summer_adventures.wav'


#     # USB Audio 设备索引和 92HD73C1X5 Analog 设备索引，根据实际情况设置
#     usb_audio_device_index = 0
#     analog_audio_device_index = 3

#     # 分别播放音频文件1和2，每个音频文件分别在一个独立的线程中播放
#     play_audio_threaded(audio1, usb_audio_device_index)
#     play_audio_threaded(audio2, analog_audio_device_index)

#     # 这里可以添加额外的逻辑，或者让主线程等待音频播放完成


import multiprocessing
import time
import wave
import pyaudio

def play_audio(filename, device_index):
    CHUNK = 1024

    wf = wave.open(filename, 'rb')

    p = pyaudio.PyAudio()
    stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                    channels=wf.getnchannels(),
                    rate=wf.getframerate(),
                    output=True,
                    output_device_index=device_index)

    data = wf.readframes(CHUNK)

    while data:
        stream.write(data)
        data = wf.readframes(CHUNK)

    stream.stop_stream()
    stream.close()
    p.terminate()

if __name__ == "__main__":
    # 音频文件路径
    audio1 = '/home/pixbus/pix/robobus/autoware-robobus/src/HMI/pixmoving/vehicle_voice_alert_system/src/vehicle_voice_alert_system/resource/bgm/chill_abstract_intention.wav'
    audio2 = '/home/pixbus/pix/robobus/autoware-robobus/src/HMI/pixmoving/vehicle_voice_alert_system/src/vehicle_voice_alert_system/resource/bgm/summer_adventures.wav'

    # USB Audio 设备索引和 92HD73C1X5 Analog 设备索引，根据实际情况设置
    usb_audio_device_index = 0
    analog_audio_device_index = 3

    # 创建两个进程，分别播放音频文件1和2
    process1 = multiprocessing.Process(target=play_audio, args=(audio1, usb_audio_device_index))
    process2 = multiprocessing.Process(target=play_audio, args=(audio2, analog_audio_device_index))

    # 启动两个进程
    process1.start()
    process2.start()

    # 主进程等待两个进程执行完成
    process1.join()
    process2.join()

    print("All audio playback completed.")
