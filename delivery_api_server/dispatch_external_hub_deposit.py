#!/usr/bin/env python3

# Copyright 2022 Open Source Robotics Foundation, Inc.
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

import sys
from unicodedata import name
import uuid
import argparse
import json

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_fleet_msgs.msg import FleetState
from rmf_fleet_msgs.msg import RobotState
from rmf_task_msgs.msg import ApiRequest


###############################################################################

class TaskRequester(Node):

    def __init__(self, argv=sys.argv):
        super().__init__('task_requester')
        parser = argparse.ArgumentParser()
        parser.add_argument('-F', '--fleet', required=False, default='',
                            type=str, help='Fleet name')
        parser.add_argument('-R', '--robot', required=False, default='',
                            type=str, help='Robot name')
        parser.add_argument('-i', '--ingress', required=True,
                            type=str, help='Ingress waypoint')
        parser.add_argument('-h', '--hub', required=True,
                            type=str, help='Hub Name waypoint')
        parser.add_argument('-e', '--egress', required=True,
                            type=str, help='Egress waypoint')
        parser.add_argument('-st', '--start_time',
                            help='Start time from now in secs, default: 0',
                            type=int, default=0)
        parser.add_argument('-pt', '--priority',
                            help='Priority value for this request',
                            type=int, default=0)
        parser.add_argument("--use_sim_time", action="store_true",
                            help='Use sim time, default: false')

        self.args = parser.parse_args(argv[1:])

        self.is_at_ingress = False

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        self.pub = self.create_publisher(
          ApiRequest, 'task_api_requests', transient_qos)

        self.fleet_state_sub = self.create_subscriber(
            RobotState, 'robot_states', robot_state_cb, 10
        )

        # enable ros sim time
        if self.args.use_sim_time:
            self.get_logger().info("Using Sim Time")
            param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
            self.set_parameters([param])

        # Construct task
        msg = ApiRequest()
        msg.request_id = "store_collect_" + str(uuid.uuid4())
        payload = {}
        if self.args.fleet and self.args.robot:
            payload["type"] = "robot_task_request"
            payload["robot"] = self.args.robot
            payload["fleet"] = self.args.fleet
        else:
            payload["type"] = "dispatch_task_request"
        request = {}
        start_time = 0
        now = self.get_clock().now().to_msg()
        now.sec =  now.sec + start_time
        start_time = now.sec * 1000 + round(now.nanosec/10**6)
        request["unix_millis_earliest_start_time"] = start_time
        # todo(YV): Fill priority after schema is added
        request["category"] = "compose"
        description = {} # task_description_Compose.json
        description["category"] = "external_hub_deposit"
        description["phases"] = []
        activities = []
        activities.append({"category": "go_to_place",  "description": self.args.hub})
        activities.append({"category": "perform_action",  "description": {"unix_millis_action_duration_estimate": 60000, "category": "hub_deposit", "description": {}}})
        activities.append({"category": "go_to_place",  "description": self.args.egress})
        description["phases"].append({"activity":{"category": "sequence", "description":{"activities":activities}}})
        request["description"] = description
        payload["request"] = request
        msg.json_msg = json.dumps(payload)
        print(f"msg: {msg}")
        self.pub.publish(msg)

###############################################################################


def main(argv=sys.argv):
    rclpy.init(args=sys.argv)
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)

    task_requester = TaskRequester(args_without_ros)
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
