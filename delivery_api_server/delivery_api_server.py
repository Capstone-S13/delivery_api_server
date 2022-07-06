
"""
The main API Interfaces (default port 8083):
1) HTTP interfaces are:  /order /receive-robot, /cancel_task
2) socketIO broadcast states: /task_status, /robot_states, /ros_time
"""

import sys
import os
from turtle import done
import rclpy
import argparse
import time
import json
import logging
import requests
from http import HTTPStatus
from threading import Thread, Semaphore

from flask import Flask, request, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO, emit, disconnect
import asyncio

from delivery_api_server.info import BuildingData, DeliveryAPIServerData, SystemServerData, Operation, SystemOrderStatus
from delivery_api_server.delivery_dispatcher_client import DeliveryDispatcherClient
from delivery_api_server.rmf_msg_observer import AsyncRmfMsgObserver, RmfMsgType

# This was derived from: https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_panel/rmf_demos_panel/simple_api_server.py

# Which uses the following licence:
"""
    Copyright 2020 Open Source Robotics Foundation, Inc.

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

class DeliveryApiServer():
    def __init__(self, server_ip, port_num, ws_port_num):
        self.app = Flask('deliver_api_server')
        CORS(self.app, origins=r"/*")
        self.socketio = SocketIO(self.app, async_mode='threading')
        self.socketio.init_app(self.app, cors_allowed_origin="*")

        self.server_ip = server_ip
        self.port_num = port_num
        self.ws_port_num = ws_port_num

        self.dispatcher_client = DeliveryDispatcherClient()

        self.order_map = {}
        self.order_map_semaphore = Semaphore()

        # default dashboard
        self.dashboard_config = {"world_name": "EMPTY_DASHBOARD_CONFIG",
                            "valid_task": [],
                            "task": {"Delivery": {}, "Loop": {}, "Clean": {}}
                            }

        # threads
        self.broadcast_thread =Thread(target=self.broadcast_states, args=())
        self.web_server_thread = Thread(target=self.web_server_spin, args=())

        # routings
        @self.app.route('/order', methods=['POST', 'GET'])
        def handle_order():
            # Dispatch robot to collect parcel
            logging.debug("order received")
            print("order received")
            if request.method == "POST":
                msg, err_msg = self.dispatcher_client.dispatch_order(request.json)
                if (msg == ""):
                    return self.app.response_class(status = HTTPStatus.BAD_REQUEST.value)
                return self.app.response_class(status=HTTPStatus.OK.value)
            # Retrieve order status and return it
            elif request.method == "GET":
                if request.args.get('order') is None:
                    logging.debug("No order in get request")
                    return HTTPStatus.BAD_REQUEST
                order = request.args.get('order')
                print(f"order is {order}")
                # get status or order

        @self.app.route('/receive-robot', methods=['POST'])
        def receive_robot():
            print("receive robot request")
            receive_status = self.dispatcher_client.receive_robot(request.json)
            # command rmf fleet adapter to accept robot into fleet and dispatch a task


        @self.app.route('/submit_task', methods=['POST'])
        def submit():
            """REST Call to submit task"""
            task_id, err_msg = self.dispatcher_client.submit_task_request(request.json)
            logging.debug(f" ROS Time: {self.dispatcher_client.ros_time()} | \
                Task Submission: {json.dumps(request.json)}, error: {err_msg}")
            return jsonify({"task_id": task_id, "error_msg": err_msg})


        @self.app.route('/cancel_task', methods=['POST'])
        def cancel():
            cancel_id = request.json['task_id']
            cancel_success = self.dispatcher_client.cancel_task_request(cancel_id)
            logging.debug(f" ROS Time: {self.dispatcher_client.ros_time()} | \
                Cancel Task: {cancel_id}, success: {cancel_success}")
            return jsonify({"success": cancel_success})


        @self.app.route('/task_list', methods=['GET'])
        def status():
            task_status = jsonify(self.dispatcher_client.get_task_status())
            logging.debug(f" ROS Time: {self.dispatcher_client.ros_time()} | \
                Task Status: {json.dumps(task_status.json)}")
            return task_status


        @self.app.route('/robot_list', methods=['GET'])
        def robots():
            robot_status = jsonify(self.dispatcher_client.get_robot_states())
            logging.debug(f" ROS Time: {self.dispatcher_client.ros_time()} | \
                Robot Status: {robot_status}")
            return robot_status


        @self.app.route('/building_map', methods=['GET'])
        def building_map():
            building_map_data = jsonify(self.dispatcher_client.get_building_map_data())
            logging.debug(f" ROS Time: {self.dispatcher_client.ros_time()} | \
                building_map_data: {building_map_data}")
            return building_map_data


        # Note: Get Dashboard Config for each "World", specific to rmf_demos impl
        @self.app.route("/dashboard_config", methods=['GET'])
        def config():
            config = jsonify(self.dashboard_config)
            return config

    def make_listener_thread(self, done_fut: asyncio.Future):
        print("creating listener thread")
        self.done_fut = done_fut
        self.listener_thread = Thread(
            target=self.rmf_listener_spin, args=())

    def web_server_spin(self):
        while rclpy.ok():
            self.dispatcher_client.spin_once()
            time.sleep(0.2)

    def broadcast_states(self):
        """
        Robot_states, tasks_status, and ros_time are being broadcasted
        to frontend UIs via socketIO, periodically (every 2s)
        """
        ns = '/status_updates'
        while rclpy.ok():
            with self.app.test_request_context():
                tasks = self.dispatcher_client.get_task_status()
                robots = self.dispatcher_client.get_robot_states()
                ros_time = self.dispatcher_client.ros_time()
                self.socketio.emit('task_status', tasks, broadcast=True, namespace=ns)
                self.socketio.emit('robot_states', robots, broadcast=True, namespace=ns)
                self.socketio.emit('ros_time', ros_time, broadcast=True, namespace=ns)
                logging.debug(f" ROS Time: {ros_time} | "
                              " tasks: "
                              f"{len(self.dispatcher_client.task_states_cache)}"
                              f" | active robots: {len(robots)}")
            time.sleep(2)

    def fleet_filter(self, json_data):
        if (json_data["assigned_to"]["group"] != BuildingData.internal_fleet_name
            or  json_data["assigned_to"]["name"] != BuildingData.internal_robot):
            self.dispatcher_client.get_logger().info("not our robot!")
            return None
        return json_data

    def update_order_status(self, data):
        status = data["status"]
        url = f"{SystemServerData.server_ip}:{SystemServerData.port_num}/\
            {SystemServerData.order_status_route}"

        requests.post(data=status, url=url)

    def post_order_status(self):
        url = f"http://{SystemServerData.server_ip}:{SystemServerData.port_num}/{SystemServerData.order_status_route}"
        for robot in self.order_map:
            order = {}
            order["orderId"] = self.order_map[robot]["id"]
            if self.order_map[robot]["status"] == "underway":
                order["status"] = SystemOrderStatus.ROBOT_DISPATCHED.value
            elif self.order_map[robot]["status"] == "completed":
                order["status"] = SystemOrderStatus.AT_STORE_HUB.value
            self.dispatcher_client.get_logger().info(f"posting order status {order}")
            requests.post(url=url, json=order)

    def msg_callback(self,msg_type, data):
        self.dispatcher_client.set_task_state(data)
        data["phases"] = {}
        data = self.fleet_filter(data)
        self.dispatcher_client.get_logger().info("rec data")
        # self.dispatcher_client.get_logger()\
        #     .info(f" \nReceived [{msg_type}] :: Data: \n   "
        #     f"{data}")
        robot = data["assigned_to"]["name"]
        if robot not in self.order_map:
            self.dispatcher_client.get_logger().info("adding robot to order map")
            order_status = {}
            order_status["id"] = data["booking"]["id"]
            order_status["status"] = data["status"]
            # self.order_map_semaphore.acquire()
            self.order_map[robot] = order_status
            # self.order_map_semaphore.release()
        else:
            # update status
            self.dispatcher_client.get_logger().info("updating order map status")
            if data["booking"]["id"] == self.order_map[robot]["id"]:
                # self.order_map_semaphore.acquire()
                self.order_map[robot]["status"] = data["status"]
                # self.order_map_semaphore.release
            else:
                # self.order_map_semaphore.acquire()
                self.order_map[robot]["id"] = data["booking"]["id"]
                self.order_map[robot]["status"] = data["status"]
                # self.order_map_semaphore.release()
        self.dispatcher_client.get_logger().info("updating order status")
        self.post_order_status()

    def rmf_listener_spin(self):
        print("creating observer")
        self.observer = AsyncRmfMsgObserver(
            self.msg_callback,
            msg_filters={RmfMsgType.TaskState: []},
            server_url="localhost",
            server_port=int(self.ws_port_num)
        )
        print("spinning observer")
        self.observer.spin(self.done_fut)

    def run_server(self):
        print(f"Starting DELIVERY API Server: {self.server_ip}:{self.port_num}, "
          f"with ws://localhost:{self.ws_port_num}")
        self.app.run(host=self.server_ip, port=self.port_num, debug=True, use_reloader=False)




###############################################################################
def main(args=None):
    server_ip = DeliveryAPIServerData.server_ip
    port_num = DeliveryAPIServerData.port_num
    ws_port_num = DeliveryAPIServerData.ws_port_num
    # logging config
    logging.getLogger('werkzeug').setLevel(logging.ERROR)  # hide logs from flask
    logging.basicConfig(level=logging.DEBUG,
                        format='%(asctime)s %(levelname)s %(message)s',
                        filename='web_server.log',
                        filemode='w')

    rclpy.init()
    delivery_api_server = DeliveryApiServer(server_ip, port_num, ws_port_num)
    delivery_api_server.web_server_thread.start()
    delivery_api_server.broadcast_thread.start()
    print("starting rmf listener")
    done_fut = asyncio.Future()
    delivery_api_server.make_listener_thread(done_fut)
    delivery_api_server.listener_thread.start()
    delivery_api_server.run_server()

    delivery_api_server.dispatcher_client.destroy_node()
    rclpy.shutdown()
    print("shutting down...")


if __name__ == "__main__":
    main(sys.argv)
