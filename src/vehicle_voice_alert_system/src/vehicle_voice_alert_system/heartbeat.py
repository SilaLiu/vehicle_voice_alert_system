# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# author：Teddy
# time:2023-09-07
# update time: 2024-04-18
# e-mail: teddy@pixmoving.net

from vehicle_voice_alert_system.autoware_diagnostic import AutowareDiagnostic
from diagnostic_msgs.msg import DiagnosticStatus

class Heartbeat:
    def __init__(self, node):
        self._node = node

        self._diagnostic_updater = AutowareDiagnostic().init_updater(
            self._node,
            "/system/voice_alert_system_connection : voirce alert system heartbeat",
            self.handle_heartbeat_diagnostics,
            "none",
        )

    def handle_heartbeat_diagnostics(self, stat):
        stat.summary(DiagnosticStatus.OK, "voirce alert system is working")
        return stat
