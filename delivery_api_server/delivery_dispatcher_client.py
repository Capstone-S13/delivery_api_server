from logging import raiseExceptions

from flask import request, request_started
import rclpy
import time
import json
import uuid
from typing import Tuple

from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter

# Qos
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_task_msgs.msg import ApiRequest, ApiResponse
from rmf_building_map_msgs.srv import GetBuildingMap
from rmf_fleet_msgs.msg import FleetState, RobotMode

# Info for delivery
from delivery_api_server.info import BuildingData, Operation

# This was derived from: https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_panel/rmf_demos_panel/simple_api_server.py

# Which uses the following licence:
"""
    Copyright 2021 Open Source Robotics Foundation, Inc.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
"""

###############################################################################


class DeliveryDispatcherClient(Node):
    def __init__(self):
        super().__init__('simple_api_server')
        api_req_qos_profile = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)
        self.task_api_req_pub = self.create_publisher(
            ApiRequest, '/task_api_requests', api_req_qos_profile)

        self.get_building_map_srv = self.create_client(
            GetBuildingMap, '/get_building_map')

        # to show robot states
        self.fleet_state_subscription = self.create_subscription(
            FleetState, 'fleet_states', self.fleet_state_cb,
            qos_profile=QoSProfile(depth=20))
        self.fleet_states_dict = {}

        # TODO remove this
        sim_time_bool = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([sim_time_bool])
        self.task_states_cache = {}

    def fleet_state_cb(self, msg: FleetState):
        fleet_name = msg.name
        self.fleet_states_dict[fleet_name] = msg.robots

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.1)

    def ros_time(self) -> int:
        return self.get_clock().now().seconds_nanoseconds()[0]

# Request
###############################################################################

    def submit_task_request(self, req_json) -> Tuple[str, str]:
        """
        Task Submission - This function will trigger a ros srv call to the
        dispatcher node, and return a response. Function will return a Task ID

        Args:
            req_json: task description in json format
        Returns:
            task_id, error_msg: if submission failed
        """
        # construct task request json from legacy format
        request_json, err_msg = self.__convert_task_description(req_json)
        if request_json is None:
            self.get_logger().error(err_msg)
            return "", err_msg
        payload = {
            "type": "dispatch_task_request",
            "request": request_json
        }

        msg = ApiRequest()
        msg.request_id = "demos_" + str(uuid.uuid4())
        msg.json_msg = json.dumps(payload)
        self.task_api_req_pub.publish(msg)
        self.get_logger().info(f'Publish task request {msg}')

        # Note: API Response or can wait for response
        # TODO: listen to "/task_api_responses"
        return msg.request_id, ""  # success

    def cancel_task_request(self, task_id) -> bool:
        """
        Cancel Task - This function will trigger a ros srv call to the
        dispatcher node, and return a response.
        """
        print(f"Canceling Task Request! {task_id}")
        payload = {
            "type": "cancel_task_request",
            "task_id": task_id,
            "labels": ["cancellation from simple api server"]
        }
        msg = ApiRequest()
        msg.request_id = "demos_" + str(uuid.uuid4())
        msg.json_msg = json.dumps(payload)
        # self.task_api_req_pub.publish(msg)

        # TODO: check res from "/task_api_responses"
        #   cancellation is not fully tested in "rmf_ros2"
        return False

    # dispatch a delivery order request to internal robot
    def dispatch_order(self, req_json) -> Tuple[str, str]:
        # request_json below refers to the a rmf task_api_request which may not be necessary in our case
        self.get_logger().info("dispatching")
        self.get_logger().info(f"request: {req_json}")
        request_json, err_msg = self.__convert_delivery_order_request(req_json)
        if request_json is None:
            self.get_logger().info(f"request json is empty! :(")
            return "", err_msg
        self.get_logger().info("message converted")
        # submit a delivery order request
        msg = ApiRequest()
        msg.request_id = req_json["task_id"]
        payload = {}
        # TODO: set payload["type"] tp ne "dispatch_task_request" if there is
        # more than one robot
        payload["type"] = "dispatch_task_request"
        payload["fleet"] = BuildingData.internal_fleet_name
        # payload["robot"] = BuildingData.internal_robot
        payload["request"] = request_json
        msg.json_msg = json.dumps(payload)
        self.task_api_req_pub.publish(msg)
        self.get_logger().info("message published")
        return msg, ""

    def receive_robot(self, req_json):
        # send a command to the fleet adapter to accept the robot into the
        # fleet and send a task to collect it from the hub
        request_json, err_msg = self.__convert_receive_robot_request(req_json)
        if request_json is None:
                return "", err_msg

        msg = ApiRequest()
        msg.request_id = req_json["task_id"]
        payload = {}
        # This has to be a robot task request
        # We need that particular robot to collect
        payload["type"] = "robot_task_request"
        payload["fleet"] = BuildingData.external_fleet_name
        payload["robot"] = req_json["robot"]["id"]
        # TODO: Check if building name is equal
        ingress_wp = req_json['ingress_point']['unit']
        # TODO: accept robot into fleet

        payload["request"] = request_json
        msg.json_msg = json.dumps(payload)
        self.task_api_req_pub.publish(msg)
        return msg, ""

        # TODO need to find a way to asynchronously wait for the robot to
        # reach the egress point to inform the system server.

        # IDEA:
        # We can call a rosservice when the robot leaves the fleet.
        # This API Server will be responding to the service request
        # Notifies the System server once the it receives the ROS Service
        # call.

    def eject_robot(self, req_json):
        request_json, err_msg = self.__convert_eject_robot_request(req_json)
        return


# Getters
###############################################################################

    def get_task_status(self):
        """
        Get all task status - This fn will trigger a ros srv call to acquire
        all submitted tasks to dispatcher node. Fn returns an object of tasks
        """
        return list(self.task_states_cache.values())

    def get_robot_states(self):
        """
        This function will return an aggregated list of robot states to the
        front end UI when a ajax GET is requested.
        """
        agg_robot_states = []
        for fleet_name, robot_states in self.fleet_states_dict.items():
            robots = self.__convert_robot_states_msg(fleet_name, robot_states)
            agg_robot_states = agg_robot_states + robots
        return agg_robot_states

    def get_building_map_data(self):
        """
        Get building map data - This fn will trigger a ros srv call to acquire
        building map data. Fn returns an object of tasks
        """
        req = GetBuildingMap.Request()
        try:
            future = self.get_building_map_srv.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            response = future.result()
            if response is None:
                self.get_logger().warn('/get_building_map srv call failed')
            else:
                # Return building map
                map_data = self.__convert_building_map_msg(
                    response.building_map)
                return map_data
        except Exception as e:
            self.get_logger().error(
                'Error! GetBuildingMap Srv failed %r' % (e,))
        return {}  # empty dict

# Setters
###############################################################################

    def set_task_state(self, json_obj):
        """
        set and store the latest task_state.
        """
        state = self.__convert_task_state_msg(json_obj)
        id = state["task_id"]
        self.task_states_cache[id] = state

    def set_delivery_task_state(self, json_obj):
        """
        send the order status to the system server.
        we may want to do this instead of waiting for the
        system server to send a get request.
        """
        status = {}
        return status

# converters
###############################################################################

    def __convert_task_state_msg(self, json_obj):
        """
        convert task_state v2 msg to legacy dashbaord json msg format
        """
        task_state = {}
        task_state["task_id"] = json_obj["booking"]["id"]
        task_state["state"] = json_obj["status"]
        task_state["fleet_name"] = json_obj["assigned_to"]["group"]
        task_state["robot_name"] = json_obj["assigned_to"]["name"]
        task_state["task_type"] = json_obj["category"]
        task_state["priority"] = 0  # TODO

        # Note: all in seconds
        task_state["start_time"] = round(
            json_obj["unix_millis_start_time"]/1000.0, 2)
        task_state["end_time"] = round(
            json_obj["unix_millis_finish_time"]/1000.0, 2)
        task_state["submited_start_time"] = round(
            json_obj["booking"]["unix_millis_earliest_start_time"]/1000.0, 2)

        # \note description should be: e.g. cleaning zone
        if "active" in json_obj:
            active_phase = json_obj["active"]
            task_state["description"] = json_obj["phases"][str(
                active_phase)]["detail"]
        else:
            task_state["description"] = "queued"

        # use start_time, end_time and current time to compute percent
        predicted_duration = task_state["end_time"] - task_state["start_time"]
        task_duration = self.ros_time() - task_state["start_time"]
        percent = task_duration/predicted_duration
        if (percent > 1.0):
            task_state["progress"] = f"100%"
        elif (task_state["state"] == "completed"):
            task_state["progress"] = f"100%"
        elif(percent < 0.0):
            task_state["progress"] = f"0%"
        else:
            percent = int(100.0*percent)
            task_state["progress"] = f"{percent}%"

        done_status_type = [
            "uninitialized", "blocked", "error", "failed",
            "skipped", "canceled", "killed", "completed"]
        if task_state["state"] in done_status_type:
            task_state["done"] = True
        else:
            task_state["done"] = False

        # Hack, change to capital letter for frontend compliance
        task_state["state"] = task_state["state"].title()
        return task_state

    def __get_robot_assignment(self, robot_name):
        assigned_tasks = []
        assigned_task_ids = []
        for _, state in self.task_states_cache.items():
            if state["robot_name"] == robot_name:
                assigned_tasks.append(state)
        assigned_tasks.sort(key=lambda x: x.get('start_time'))
        for task in assigned_tasks:
            assigned_task_ids.append(task["task_id"])
        return assigned_task_ids

    def __convert_robot_states_msg(self, fleet_name, robot_states):
        """
        convert robot states msg to a jsonify-able robot_states
        """
        bots = []
        mode_enum = {
            RobotMode.MODE_IDLE: "Idle-0",
            RobotMode.MODE_CHARGING: "Charging-1",
            RobotMode.MODE_MOVING: "Moving-2",
            RobotMode.MODE_PAUSED: "Paused-3",
            RobotMode.MODE_WAITING: "Waiting-4",
            RobotMode.MODE_EMERGENCY: "Emengency-5",
            RobotMode.MODE_GOING_HOME: "GoingHome-6",
            RobotMode.MODE_DOCKING: "Dock/Clean-7",
            RobotMode.MODE_ADAPTER_ERROR: "AdpterError-8"
        }
        for bot in robot_states:
            state = {}
            state["robot_name"] = bot.name
            state["fleet_name"] = fleet_name
            state["mode"] = mode_enum[bot.mode.mode]
            state["battery_percent"] = bot.battery_percent
            state["location_x"] = bot.location.x
            state["location_y"] = bot.location.y
            state["location_yaw"] = bot.location.yaw
            state["level_name"] = bot.location.level_name
            state["assignments"] = self.__get_robot_assignment(bot.name)
            bots.append(state)
        return bots

    ## convert delivery order request into a task request
    def __convert_delivery_order_request(self, order_json):
        self.get_logger().info("attempting to convert request")
        if (("order" not in order_json) or
            ("id" not in  order_json["order"]) or
            ("unit" not in order_json) or
            ("operation" not in order_json)):
            self.get_logger().info("order request is not complete")
            raise Exception("Key value is incomplete")

        else:
            self.get_logger().info("request is filled!")
            request =  {}
            start_time_sec = 0
            now = self.get_clock().now().to_msg()
            now.sec =  now.sec + start_time_sec
            start_time = now.sec * 1000 + round(now.nanosec/10**6)
            request["unix_millis_earliest_start_time"] = start_time
            request["category"] = "compose"
            description = {} # task_description_Compose.json
            description["category"] = "hub_delivery"
            description["phases"] = []
            activities = []
            # deposit to hub
            if (order_json["operation"]["task"] == Operation.HUB_DEPOSIT.value):
                self.get_logger().info(f"Operation: {Operation.HUB_DEPOSIT}")
                activities.append({"category": "go_to_place",  "description": order_json["unit"]["unit"]})
                collection_activity = {"category": "perform_action",
                "description": {"unix_millis_action_duration_estimate": 60000,
                "category": "unit_collect",
                "description": {"id": order_json["order"]["id"],
                    "company_name": order_json["order"]["company_name"],
                    "description":
                        "" if ("description" not in order_json["order"])
                        else order_json["order"]["description"]}}}
                activities.append(collection_activity)

                activities.append({"category": "go_to_place",  "description": BuildingData.hub})
                hub_activity = {"category": "perform_action",  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "hub_deposit",
                    "description": {"id": order_json["order"]["id"],
                        "company_name": order_json["order"]["company_name"],
                        "description":
                            "" if ("description" not in order_json["order"])
                            else order_json["order"]["description"]}}}
                activities.append(hub_activity)


            # collect from hub
            elif (order_json["operation"]["task"] == Operation.HUB_COLLECT.value):
                self.get_logger().info(f"Operation: {Operation.HUB_COLLECT}")
                activities.append({"category": "go_to_place",  "description": BuildingData.hub})
                hub_activity = {"category": "perform_action",  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "hub_collect",
                    "description": {"id": order_json["order"]["id"],
                        "company_name": order_json["order"]["company_name"],
                        "description":
                            "" if ("description" not in order_json["order"])
                            else order_json["order"]["description"]}}}
                activities.append(hub_activity)

                activities.append({"category": "go_to_place",
                    "description": order_json["unit"]["unit"]})
                unit_activity = {"category": "perform_action", "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "unit_deposit",
                    "description": {"id": order_json["order"]["id"],
                        "company_name": order_json["order"]["company_name"],
                        "description":
                            "" if ("description" not in order_json["order"])
                            else order_json["order"]["description"]}}}
                activities.append(unit_activity)

            activities.append({"category": "go_to_place",  "description": BuildingData.holding_point})
            description["phases"].append({"activity":{"category": "sequence",
                "description":{"activities":activities}}})
            request["description"] = description

            self.get_logger().info(f"rmf task request: {request}")
            return request, ""

    def __convert_receive_robot_request(self, receive_robot_json):
        if (("robot" not in receive_robot_json) or
            ("ingress_point" not in  receive_robot_json) or
            ("egress_point" not in receive_robot_json) or
            ("order" not in receive_robot_json) or
            ("operation" not in receive_robot_json)):
            raise Exception("Key value is incomplete")
        else:
            request = {}
            start_time = 0
            now = self.get_clock().now().to_msg()
            now.sec =  now.sec + start_time
            start_time = now.sec * 1000 + round(now.nanosec/10**6)
            request["unix_millis_earliest_start_time"] = start_time
            request["category"] = "compose"
            description = {} # task_description_Compose.json
            description["category"] = "hub_delivery"
            description["phases"] = []
            activities = []

            if (receive_robot_json["operation"]["task"] == Operation.HUB_COLLECT.value):
                activities.append({"category": "go_to_place",  "description": BuildingData.hub})
                hub_activity = {"category": "perform_action",  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "hub_collect",
                    "description": {"id": receive_robot_json["order"]["id"],
                                    "company":receive_robot_json["order"]["company_name"]}}}
                activities.append(hub_activity)


            elif (receive_robot_json["operation"]["task"] == Operation.HUB_DEPOSIT.value):
                activities.append({"category": "go_to_place",  "description": BuildingData.hub})
                hub_activity = {"category": "perform_action",  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "hub_deposit",
                    "description": {"id": receive_robot_json["order"]["id"],
                                    "company":receive_robot_json["order"]["company_name"]}}}
                activities.append(hub_activity)

            elif (receive_robot_json["operation"]["task"] == Operation.DIRECT_COLLECT.value):
                destination = receive_robot_json["destination"]["unit"]
                activities.append({"category": "go_to_place",  "description": destination})
                collect_activity = {"category": "perform_action",  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "unit_collect",
                    "description": {"id": receive_robot_json["order"]["id"],
                                    "company":receive_robot_json["order"]["company_name"]}}}
                activities.append(collect_activity)

            elif (receive_robot_json["operation"]["task"] == Operation.DIRECT_DEPOSIT.value):
                destination = receive_robot_json["destination"]["unit"]
                activities.append({"category": "go_to_place",  "description": destination})
                deposit_activity = {"category": "perform_action",  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "unit_deposit",
                    "description": {"id": receive_robot_json["order"]["id"],
                                    "company":receive_robot_json["order"]["company_name"]}}}
                activities.append(deposit_activity)

            activities.append({"category": "go_to_place",
                "description": receive_robot_json["egress_point"]["unit"]})
            description["phases"].append({"activity":{"category": "sequence",
                "description":{"activities":activities}}})
            request["description"] = description
            return request, ""

    def __convert_eject_robot_request(self, eject_robot_json):
        request = {}
        start_time = 0
        now = self.get_clock().now().to_msg()
        now.sec =  now.sec + start_time
        start_time = now.sec * 1000 + round(now.nanosec/10**6)
        request["unix_millis_earliest_start_time"] = start_time
        request["category"] = "compose"
        description = {} # task_description_Compose.json
        description["phases"] = []
        activities = []
        egress_point = eject_robot_json["egress_point"]
        activities.append({"category": "go_to_place",  "description": egress_point})

        eject_activity = {"category": "perform_action",  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "hub_collect",
                    "description": {
                        "new_host": eject_robot_json["new_host"]}
                                    }
                        }
        activities.append(eject_activity)

        description["phases"].append({"activity":{"category": "sequence",
                "description":{"activities":activities}}})
        request["description"] = description
        return request

    def __convert_task_description(self, task_json):
        """
        Convert a json task req format to legacy dashboard json msg format
        :note: The 'start time' here is the "Duration" from now.
        """
        # default request fields
        request = {
            "priority": {"type": "binary", "value": 0},
            "labels": ["rmf_demos.simple_api_server"],
            "description": {}
        }
        try:
            if (("task_type" not in task_json) or
                ("start_time" not in task_json) or
                    ("description" not in task_json)):
                raise Exception("Key value is incomplete")

            if ("priority" in task_json):
                priority_val = int(task_json["priority"])
                if (priority_val < 0):
                    raise Exception("Priority value is less than 0")
                request["priority"]["value"] = priority_val

            # Refer to task schemas
            # https://github.com/open-rmf/rmf_ros2/blob/redesign_v2/rmf_fleet_adapter/schemas
            desc = task_json["description"]
            if task_json["task_type"] == "Clean":
                request["category"] = "clean"
                request["description"]["zone"] = desc["cleaning_zone"]
            elif task_json["task_type"] == "Loop":
                request["category"] = "patrol"
                request["description"]["places"] = [
                    desc["start_name"],
                    desc["finish_name"]]
                request["description"]["rounds"] = int(desc["num_loops"])
            elif task_json["task_type"] == "Delivery":
                request["category"] = "delivery"
                request["description"]["pickup"] = {
                    "place": desc["pickup_place_name"],
                    "handler": desc["pickup_dispenser"],
                    "payload": []}
                request["description"]["dropoff"] = {
                    "place": desc["dropoff_place_name"],
                    "handler": desc["dropoff_ingestor"],
                    "payload": []}
            else:
                raise Exception("Invalid TaskType")

            # Calc earliest_start_time, convert "Duration from now(min)"
            # to unix_milli epoch time
            rclpy.spin_once(self, timeout_sec=0.0)
            rostime_now = self.get_clock().now()
            unix_milli_time = round(rostime_now.nanoseconds/1e6)
            unix_milli_time += int(task_json["start_time"]*60)
            request["unix_millis_earliest_start_time"] = unix_milli_time
        except KeyError as ex:
            return None, f"Missing Key value in json body: {ex}"
        except Exception as ex:
            return None, str(ex)
        print("return", request)
        return request, ""

    def __convert_building_map_msg(self, msg):
        map_data = {}

        map_data["name"] = msg.name
        map_data["levels"] = []

        for level in msg.levels:
            level_data = {}
            level_data["name"] = level.name
            level_data["elevation"] = level.elevation

            # TODO: Images, places, doors?
            level_data["nav_graphs"] = \
                [self.__convert_graph_msg(msg) for msg in level.nav_graphs]
            level_data["wall_graph"] = \
                self.__convert_graph_msg(level.wall_graph)

            map_data["levels"].append(level_data)

        return map_data

    def __convert_graph_msg(self, graph_msg):
        graph_data = {}
        graph_data["name"] = graph_msg.name
        graph_data["vertices"] = []
        graph_data["edges"] = []

        for vertex in graph_msg.vertices:
            vertex_data = {}
            vertex_data["x"] = vertex.x
            vertex_data["y"] = vertex.y
            vertex_data["name"] = \
                vertex.name

            vertex_data["params"] = \
                [self.__convert_param_msg(msg) for msg in vertex.params]

            graph_data["vertices"].append(vertex_data)

        for edge in graph_msg.edges:
            edge_data = {}
            edge_data["v1_idx"] = edge.v1_idx
            edge_data["v2_idx"] = edge.v2_idx
            edge_data["edge_type"] = edge.edge_type

            edge_data["params"] = \
                [self.__convert_param_msg(msg) for msg in edge.params]

            graph_data["edges"].append(edge_data)

        return graph_data

    def __convert_param_msg(self, param_msg):
        param_data = {}
        param_data["name"] = param_msg.name

        # TODO: how to handle TYPE_UNDEFINED?
        if param_msg.type == param_msg.TYPE_STRING:
            param_data["type"] = "string"
            param_data["value"] = str(param_msg.value_string)
        elif param_msg.type == param_msg.TYPE_INT:
            param_data["type"] = "int"
            param_data["value"] = str(param_msg.value_int)
        elif param_msg.type == param_msg.TYPE_DOUBLE:
            param_data["type"] = "double"
            param_data["value"] = str(param_msg.value_float)
        elif param_msg.type == param_msg.TYPE_BOOL:
            param_data["type"] = "bool"
            param_data["value"] = str(param_msg.value_bool)
        else:
            param_data["value"] = None

        return param_data
