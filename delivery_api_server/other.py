
from flask import Flask, request
from flask_socketio import SocketIO
import requests
from delivery_api_server.info import SystemServerData, SystemTaskStatus
from http import HTTPStatus
from threading import Thread, Semaphore
import time



app = Flask("minimal sys server")
socketio = SocketIO(app, cors_allowed_origin="*")
socketio.init_app(app, cors_allowed_origin="*")

eject_robot_sem = Semaphore()
eject_robot_json = {}


task_status_dic = {
    0 : "TASK_UNDERWAY",
    1 : "TASK_COMPLETED",
    2 : "TASK_FAILED"
}

@app.route('eject-robot', methods=['POST'])
def handle_eject_robot():
    print("received eject robot")
    try:
        print(f"task id: {request.json['task_id']}")
        print(f"egress_point: {request.json['egress_point']}")
        print(f"robot: {request.json['robot']['id']}")
        print(f"host: {request.json['new_host']['ip']}:{request.json['new_host']['port']}")
        print(f"map: {request.json['map']['pgm']}")
        print(f"map origin: {request.json['map']['origin']}")
        print(f"map negate : {request.json['map']['negate']}")
        print(f"map occupied_thresh: {request.json['map']['occupied_thresh']}")
        print(f"map free_thresh: {request.json['map']['free_thrresh']}")
        print(f"map resolution: {request.json['map']['resolution']}")
        print(f"initial_pose: {request.json['initial_pose']}")
        eject_robot_sem.acquire()
        eject_robot_json = request.json
        eject_robot_sem.release()
        return app.response_class(status=HTTPStatus.OK.value)
    except:
        print("json not in specified format")
        return app.response_class(status=HTTPStatus.BAD_REQUEST)

def loop():
    eject_robot_sem.acquire()
    if eject_robot_json == None:
        eject_robot_sem.release()
        return
    time.sleep(1)
    task_status_json = {}
    task_status_json["robot"] = eject_robot_json["robot"]
    task_status_json["task_id"] = eject_robot_json["task_id"]
    eject_robot_sem.release()
    task_status_json["status"] = SystemTaskStatus.TASK_COMPLETED
    url = f"http://{SystemServerData.server_ip}:{SystemServerData.port_num}{SystemServerData.task_status_route}"
    resp = requests.post(url = url, json=task_status_json)
    return


def main():

    print("other rmf server is running")
    server_ip = 'localhost'
    port_num = 7272
    loop_thread = Thread(target=loop, args=())
    loop_thread.start()
    app.run(host=server_ip, port=port_num, debug=True, use_reloader=False)
    loop_thread.join()


if __name__ == '__main__':
    main()