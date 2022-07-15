
from crypt import methods
from threading import Thread
from flask import Flask, json, request
from flask_socketio import SocketIO
import requests
from delivery_api_server.info import SystemServerData, SystemOrderStatus
from http import HTTPStatus
import time


status_dic = {
    0 : "ORDER_SENT",
    1 : "ORDER_RECEIVED",
    2 : "ROBOT_DISPATCHED",
    3 : "AT_STORE_HUB",
    4 : "BETWEEN_HUBS",
    5 : "ARRIVED",
    6 : "DELIVERED",
    7 : "CANCELLED",
    8 : "FAILED"
    }

class TestServer():
    def __init__(self):
        self.app = Flask("minimal sys server")
        self.socketio = SocketIO(self.app, cors_allowed_origin="*")
        self.socketio.init_app(self.app, cors_allowed_origin="*")
        self.current_internal_order = None
        self.waypoints = ["pantry", "coe", "hardware", "lounge"]
        self.send_order_status_thread =\
            Thread(target=self.delay_and_send_order_status, args=())


        @self.app.route('/order', methods=['POST'])
        def handle_order():
            print("received_order")
            if self.validate_order(request.json):
                self.current_internal_order = request.json
                self.send_order_status_thread.start()
                return self.app.response_class(status=HTTPStatus.OK.value)
            return self.app.response_class(status=HTTPStatus.BAD_REQUEST.value)


        @self.app.route('/receive-robot', methods=['POST'])
        def handle_receive_robot():
            print("receive robot")
            try:
                print(f"robot: {request.json['robot']['id']}")
                print(f"egress_point: {request.json['egress_point']['building_name']},\
                        {request.json['egress_point']['unit']}")
                return self.app.response_class(status=HTTPStatus.OK.value)
            except Exception as e:
                print(e)
                return self.app.response_class(status=HTTPStatus.BAD_REQUEST)

        @self.app.route('/eject-robot', methods=['POST'])
        def handle_eject_robot():
            try:
                print("eject robot")
                print(f"orderId: {request.json['orderId']}")
                print(f"status: {status_dic[request.json['status']]}")
                return self.app.response_class(status=HTTPStatus.OK.value)
            except Exception as e:
                print(e)
                return self.app.response_class(status=HTTPStatus.BAD_REQUEST)


    def delay_and_send_order_status(self):
        time.sleep(3)
        self.send_order_in_hub()

    def send_order_in_hub(self):
        url = f'http://{SystemServerData.server_ip}:{SystemServerData.port_num}/{SystemServerData.order_status_route}'
        assert(self.current_internal_order is not None)
        order = {}
        order["orderId"] = self.current_internal_order["order"]["id"]
        order["status"] = SystemOrderStatus.AT_STORE_HUB.value
        resp = requests.post(url=url, json=order)
        return

    def validate_order(self, req_json):



def main():
    print("minimal server is running")
    server_ip = SystemServerData.server_ip
    port_num = SystemServerData.port_num
    app.run(host=server_ip, port=port_num, debug=True, use_reloader=False)