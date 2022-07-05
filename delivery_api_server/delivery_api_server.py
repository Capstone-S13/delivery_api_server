
"""
The main API Interfaces (default port 8083):
1) HTTP interfaces are:  /order /receive-robot, /cancel_task
2) socketIO broadcast states: /task_status, /robot_states, /ros_time
"""

import sys
import os
import rclpy
import argparse
import time
import json
import logging
import requests
from http import HTTPStatus
from threading import Thread

from flask import Flask, request, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO, emit, disconnect
import asyncio

from delivery_api_server.info import BuildingData, DeliveryAPIServerData, SystemServerData
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


app = Flask(__name__)
CORS(app, origins=r"/*")

socketio = SocketIO(app, async_mode='threading')
socketio.init_app(app, cors_allowed_origins="*")

rclpy.init(args=None)
dispatcher_client = DeliveryDispatcherClient()

# logging config
logging.getLogger('werkzeug').setLevel(logging.ERROR)  # hide logs from flask
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(levelname)s %(message)s',
                    filename='web_server.log',
                    filemode='w')

# default dashboard
dashboard_config = {"world_name": "EMPTY_DASHBOARD_CONFIG",
                    "valid_task": [],
                    "task": {"Delivery": {}, "Loop": {}, "Clean": {}}
                    }

###############################################################################

@app.route('/order', methods=['POST', 'GET'])
def handle_order():
    # Dispatch robot to collect parcel
    logging.debug("order received")
    print("order received")
    if request.method == "POST":
        msg, err_msg = dispatcher_client.dispatch_order(request.json)
        if (msg == ""):
            return app.response_class(status = HTTPStatus.BAD_REQUEST.value)
        return app.response_class(status=HTTPStatus.OK.value)
    # Retrieve order status and return it
    elif request.method == "GET":
        if request.args.get('order') is None:
            logging.debug("No order in get request")
            return HTTPStatus.BAD_REQUEST
        order = request.args.get('order')
        print(f"order is {order}")
        # get status or order

@app.route('/receive-robot', methods=['POST'])
def receive_robot():
    print("receive robot request")
    receive_status = dispatcher_client.receive_robot(request.json)
    # command rmf fleet adapter to accept robot into fleet and dispatch a task


@app.route('/submit_task', methods=['POST'])
def submit():
    """REST Call to submit task"""
    task_id, err_msg = dispatcher_client.submit_task_request(request.json)
    logging.debug(f" ROS Time: {dispatcher_client.ros_time()} | \
        Task Submission: {json.dumps(request.json)}, error: {err_msg}")
    return jsonify({"task_id": task_id, "error_msg": err_msg})


@app.route('/cancel_task', methods=['POST'])
def cancel():
    cancel_id = request.json['task_id']
    cancel_success = dispatcher_client.cancel_task_request(cancel_id)
    logging.debug(f" ROS Time: {dispatcher_client.ros_time()} | \
        Cancel Task: {cancel_id}, success: {cancel_success}")
    return jsonify({"success": cancel_success})


@app.route('/task_list', methods=['GET'])
def status():
    task_status = jsonify(dispatcher_client.get_task_status())
    logging.debug(f" ROS Time: {dispatcher_client.ros_time()} | \
        Task Status: {json.dumps(task_status.json)}")
    return task_status


@app.route('/robot_list', methods=['GET'])
def robots():
    robot_status = jsonify(dispatcher_client.get_robot_states())
    logging.debug(f" ROS Time: {dispatcher_client.ros_time()} | \
        Robot Status: {robot_status}")
    return robot_status


@app.route('/building_map', methods=['GET'])
def building_map():
    building_map_data = jsonify(dispatcher_client.get_building_map_data())
    logging.debug(f" ROS Time: {dispatcher_client.ros_time()} | \
        building_map_data: {building_map_data}")
    return building_map_data


# Note: Get Dashboard Config for each "World", specific to rmf_demos impl
@app.route("/dashboard_config", methods=['GET'])
def config():
    config = jsonify(dashboard_config)
    return config

###############################################################################


def web_server_spin():
    while rclpy.ok():
        dispatcher_client.spin_once()
        time.sleep(0.2)

# not required to broadcast everything. Perhaps we would want consider process
# the task status and send a patch request of the delivery status and robot
# status periodically so the system server would not have to send a get
# request.
def broadcast_states():
    """
    Robot_states, tasks_status, and ros_time are being broadcasted
    to frontend UIs via socketIO, periodically (every 2s)
    """
    ns = '/status_updates'
    while rclpy.ok():
        with app.test_request_context():
            tasks = dispatcher_client.get_task_status()
            robots = dispatcher_client.get_robot_states()
            ros_time = dispatcher_client.ros_time()
            socketio.emit('task_status', tasks, broadcast=True, namespace=ns)
            socketio.emit('robot_states', robots, broadcast=True, namespace=ns)
            socketio.emit('ros_time', ros_time, broadcast=True, namespace=ns)
            logging.debug(f" ROS Time: {ros_time} | "
                          " tasks: "
                          f"{len(dispatcher_client.task_states_cache)}"
                          f" | active robots: {len(robots)}")
        time.sleep(2)

def fleet_filter(json_data):
    if (json_data["assigned_to"]["group"] != BuildingData.internal_fleet_name
        or  json_data["assigned_to"]["name"] != BuildingData.internal_robot):
        dispatcher_client.get_logger().info("not our robot!")
        return None
    return json_data

def update_order_status(data):
    status = data["status"]
    url = f"{SystemServerData.server_ip}:{SystemServerData.port_num}/\
        {SystemServerData.order_status_route}"

    requests.post(data=status, url=url)

def rmf_state_listener(port_num: str, done_fut: asyncio.Future):
    def msg_callback(msg_type, data):
        dispatcher_client.set_task_state(data)
        data["phases"] = {}
        dispatcher_client.get_logger()\
            .info(f" \nReceived [{msg_type}] :: Data: \n   "
            f"{data}")
        data = fleet_filter(data)

    observer = AsyncRmfMsgObserver(
        msg_callback,
        msg_filters={RmfMsgType.TaskState: []},
        server_url="localhost",
        server_port=int(port_num)
    )
    observer.spin(done_fut)

###############################################################################


def main(args=None):
    server_ip = DeliveryAPIServerData.server_ip
    port_num = DeliveryAPIServerData.port_num
    ws_port_num = DeliveryAPIServerData.ws_port_num

    if "DELIVERY_API_SERVER_IP" in os.environ:
        server_ip = os.environ['DELIVERY_API_SERVER_IP']
        print(f"Set Server IP to: {server_ip}")

    if "DELIVERY_API_SERVER_PORT" in os.environ:
        port_num = int(os.environ['DELIVERY_API_SERVER_PORT'])
        print(f"Set Server port to: {server_ip}:{port_num}")

    if "DELIVERY_WS_SERVER_PORT" in os.environ:
        ws_port_num = int(os.environ['RMF_WS_SERVER_PORT'])
        print(f"Set DELIVERY Websocket port to: localhost:{ws_port_num}")

    if "DASHBOARD_CONFIG_PATH" in os.environ:
        config_path = os.environ['DASHBOARD_CONFIG_PATH']

        if not config_path:
            print(f"WARN! env DASHBOARD_CONFIG_PATH is empty...")
        elif not os.path.exists(config_path):
            raise FileNotFoundError(f"\n File [{config_path}] doesnt exist")
        else:
            try:
                f = open(config_path, 'r')
                global dashboard_config
                dashboard_config = json.load(f)
            except Exception as err:
                print(f"Failed to read [{config_path}] dashboard config file")
                raise err
    else:
        print(f"WARN! env DASHBOARD_CONFIG_PATH is not specified...")

    spin_thread = Thread(target=web_server_spin, args=())
    spin_thread.start()

    broadcast_thread = Thread(target=broadcast_states, args=())
    broadcast_thread.start()

    print("starting rmf listener")
    done_fut = asyncio.Future()
    listener_thread = Thread(
        target=rmf_state_listener, args=(ws_port_num, done_fut))
    listener_thread.start()

    print(f"Starting DELIVERY API Server: {server_ip}:{port_num}, "
          f"with ws://localhost:{ws_port_num}")
    app.run(host=server_ip, port=port_num, debug=True, use_reloader=False)
    dispatcher_client.destroy_node()
    rclpy.shutdown()
    print("shutting down...")
    # done_fut.set_result(True)  # shutdown listner


if __name__ == "__main__":
    main(sys.argv)
