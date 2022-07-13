
from flask import Flask, request
from flask_socketio import SocketIO
import requests
from delivery_api_server.info import SystemServerData, SystemTaskStatus
from http import HTTPStatus



app = Flask("minimal sys server")
socketio = SocketIO(app, cors_allowed_origin="*")
socketio.init_app(app, cors_allowed_origin="*")

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

task_status_dic = {
    0 : "TASK_UNDERWAY",
    1 : "TASK_COMPLETED",
    2 : "TASK_FAILED"
}

@app.route(f'/{SystemServerData.order_status_route}', methods=['POST'])
def handle_order_status():
    print("received order")
    try:
        print(f"task id: {request.json['task_id']}")
        print(f"robot: {request.json['robot']['id']}")
        print(f"status: {status_dic[request.json['status']]}")
        return app.response_class(status=HTTPStatus.OK.value)
    except:
        print("json not in specified format")
        return app.response_class(status=HTTPStatus.BAD_REQUEST)




def main():
    print("minimal server is running")
    server_ip = SystemServerData.server_ip
    port_num = SystemServerData.port_num
    app.run(host=server_ip, port=port_num, debug=True, use_reloader=False)