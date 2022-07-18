from webbrowser import Opera
import requests
import uuid
import json
from http import HTTPStatus

from delivery_api_server.info import DeliveryAPIServerData, Operation


def main():
    go_to_json = {
                        "task_id" : f'barg_{str(uuid.uuid4())}',
                        "robot" :
                            {
                                "id" : "external1"
                            },

                        "destination":
                            {
                                "building_name" : "vovi_city",
                                "unit": "pantry"
                            },

                        "operation":
                            {
                                "task": Operation.GO_TO_PLACE.value
                            }
                        }

    print(f"hub_collect_json: {go_to_json}")
    route = "/external-task"
    url = 'http://' +  DeliveryAPIServerData.server_ip + ":" +\
        str(DeliveryAPIServerData.port_num) + route
    response = requests.post(url, json=go_to_json)
    assert(response.status_code == HTTPStatus.OK.value )

if __name__ == '__main__':
    main()