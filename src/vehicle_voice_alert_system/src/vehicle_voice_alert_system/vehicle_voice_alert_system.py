# This Python file uses the following encoding: utf-8

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


from vehicle_voice_alert_system.heartbeat import Heartbeat
from vehicle_voice_alert_system.announce_controller import AnnounceControllerProperty
from vehicle_voice_alert_system.ros_service_interface import RosServiceInterface
from vehicle_voice_alert_system.parameter_interface import ParameterInterface
from vehicle_voice_alert_system.autoware_interface import AutowareInterface
from ament_index_python.packages import get_package_share_directory

from vehicle_voice_alert_system.display_play_ctrl_publish import DisplayPlayCtrlPublisher


def main(args=None):
    package_path = get_package_share_directory("vehicle_voice_alert_system")

    rclpy.init(args=args)
    node = Node("vehicle_voice_alert_system")

    heartbeat = Heartbeat(node)
    ros_service_interface = RosServiceInterface(node)
    parameter_interface = ParameterInterface(node)
    autoware_interface = AutowareInterface(node)
    display_play_ctrl_publish = DisplayPlayCtrlPublisher(node)
    announceController = AnnounceControllerProperty(
        node,
        ros_service_interface,
        parameter_interface,
        autoware_interface,
        display_play_ctrl_publish,
    )

    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("KeyboardInterrupt received. Exiting...")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()   


if __name__ == "__main__":
    main()
