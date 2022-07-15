import requests
import uuid
import json
from http import HTTPStatus

from delivery_api_server.info import DeliveryAPIServerData, Operation



def main():
    file_path = ''
    f = open(f'{file_path}.pgm', 'rb')
    pgm_map = f.read()
    f.close()
    f = open(f'{file_path}.yaml', 'rb')
    yaml_map = f.read()
    f.close()
    eject_robot_collect_json = {
                        "robot":
                            {
                                "id": "Unodopo2"
                            },

                        "egress_point":
                            {
                                "building_name" : "vovi_city",
                                "unit": "pantry"
                            },

                        "operation":
                            {
                                "task": Operation.HUB_COLLECT.value
                            },
                        "new_host":
                            {
                                "ip": "0.0.0.0",
                                "port": 7000
                            },
                        "map":
                            {
                                "yaml": yaml_map,
                                "pgm" : pgm_map
                            }
                        }

    print(f"receive_robot_json: {eject_robot_collect_json}")
    route = "/receive-robot"
    url = 'http://' +  DeliveryAPIServerData.server_ip + ":" +\
        str(DeliveryAPIServerData.port_num) + route
    response = requests.post(url, json=receive_robot_collect_json)
    assert(response.status_code == HTTPStatus.OK.value )

if __name__ == '__main__':
    main()