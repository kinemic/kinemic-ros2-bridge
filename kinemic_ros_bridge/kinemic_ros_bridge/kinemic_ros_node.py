# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Publisher Node for Publication of Kinemic Gesture Service Events

It is intended to forward messages from the Kinemic Gesture Service to ROS, either by spawning a dedicated Gesture
Service, or by connecting to an already running instance.

You can modify this behaviour by altering the values
USE_BUNDLED_SERVICE
BUNDLED_ENGINE_PORTS
SERVICE_HOST_INFORMATION

If USE_BUNDLED_SERVICE is True, a dedicated Gesture Service is spawned, trying to use a BGLib BLED112 Dongle (preferred)
or the system Bluetooth (fallback) to establish connections to Kinemic Bands.
It will use the Ports defined in BUNDLED_ENGINE_PORTS.

If USE_BUNDLED_SERVICE is False, a connection to the already running Gesture Service instance running on the ports
provided in SERVICE_HOST_INFORMATION will be established.

This node will not handle connection management, but will instead only forward all events to ROS.
To connect to a Kinemic Band, you can e.g. use the Kinemic Remote on the configured Service instance.
(https://repo.kinemic.com/#browse/browse:kinemic-remote-releases)

The Node will publish to the Topic defined in
ROS_TOPIC

Based on the Example Code from
https://github.com/ros2/demos/blob/rolling/demo_nodes_py/demo_nodes_py/topics/talker.py
"""

import asyncio
import threading

import kserviceconnect
import rclpy
from kserviceconnect.eventlistener import ZMQEventListener
from kserviceconnect.library import EnginePorts
from kserviceconnect.library.manager import BundledOptions, BundledServiceProvider
from kserviceconnect.provider import HostInformationProvider
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

USE_BUNDLED_SERVICE = False
BUNDLED_ENGINE_PORTS = (
    EnginePorts.default_service_ports()
)  # Event Publication on Port 9999, RPC on Port 9998
SERVICE_HOST_INFORMATION = kserviceconnect.HostInformation(
    host="host.docker.internal", event_port=9999, rpc_port=9998
)
ROS_TOPIC = "chatter"


class KinemicEventPublisher(Node):
    def __init__(self):
        super().__init__("KinemicPublisher")
        self.publisher_ = self.create_publisher(String, ROS_TOPIC, 10)

        # the Kinemic Gesture Service library utilizes asyncio that is currently not supported in the rclpy event loop
        # therefore we start a dedicated loop in a thread to create a listening task
        self.__loop = asyncio.new_event_loop()
        self.__task = None
        self.__thread = threading.Thread(target=self.__loop.run_forever)
        self.__thread.start()

        self.__loop.call_soon_threadsafe(self.__create_task, self._send_out_gestures())

    async def _send_out_gestures(self):
        async for event in ZMQEventListener():
            msg = String()
            msg.data = event.json()
            self.publisher_.publish(msg)

    def __create_task(self, coroutine):
        self.__task = self.__loop.create_task(coroutine)

    def __del__(self):
        if self.__task is not None:
            self.__task.cancel()

        self.__loop.stop()
        self.__thread.join()


def set_up_kinemic_service_connection(
    bundled_service: bool,
) -> kserviceconnect.HostInformation:
    """Set up the connection to the Kinemic Gesture Service

    :param bundled_service: spawn bundled Gesture Service if True, use provided address on False
    :return: HostInformation to connect to the configured Kinemic Gesture Service
    """

    if bundled_service:
        options = BundledOptions(
            localhost_only=False, use_bglib=True, engine_ports=BUNDLED_ENGINE_PORTS
        )
        success = BundledServiceProvider.load_bundled_service(options)
        if not success:
            options = BundledOptions(
                localhost_only=False, use_bglib=False, engine_ports=BUNDLED_ENGINE_PORTS
            )
            success = BundledServiceProvider.load_bundled_service(options)
            if not success:
                raise RuntimeError("Konnte Kinemic Service nicht starten!")

        host_info = kserviceconnect.HostInformation(
            host="localhost",
            rpc_port=BundledServiceProvider.engine_ports.rep_port,
            event_port=BundledServiceProvider.engine_ports.pub_port,
        )
    else:
        host_info = SERVICE_HOST_INFORMATION
    return host_info


def main(args=None):
    rclpy.init(args=args)

    host_info = set_up_kinemic_service_connection(bundled_service=USE_BUNDLED_SERVICE)
    HostInformationProvider.update_host_info(host_info)

    publisher = KinemicEventPublisher()
    try:
        rclpy.spin(publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        publisher.destroy_node()
        BundledServiceProvider.destroy_bundled_service()

        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
