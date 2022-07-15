import requests
import uuid
import json
from http import HTTPStatus

from delivery_api_server.info import DeliveryAPIServerData, Operation


def main():
    receive_robot_collect_json = {
                        "task_id" : str(uuid.uuid4()),
                        "robot":
                            {
                                "id": "external1"
                            },

                        "ingress_point":
                            {
                                "building_name" : "vovi_city",
                                "unit": "externaldopo_charger"
                            },
                        "egress_point":
                            {
                                "building_name" : "vovi_city",
                                "unit": "pantry"
                            },
                        "destination":
                            {
                                "building_name" : "vovi_city",
                                "unit": "supplies"
                            },
                        "order":
                            {
                                "company_name": "barg",
                                "id": "barg" + str(uuid.uuid4()),
                                "description" : "a small item"
                            },

                        "operation":
                            {
                                "task": Operation.DIRECT_DEPOSIT.value
                            }
                        }

    print(f"receive_robot_json: {receive_robot_collect_json}")
    route = "/receive-robot"
    url = 'http://' +  DeliveryAPIServerData.server_ip + ":" +\
        str(DeliveryAPIServerData.port_num) + route
    response = requests.post(url, json=receive_robot_collect_json)
    assert(response.status_code == HTTPStatus.OK.value )

if __name__ == '__main__':
    main()