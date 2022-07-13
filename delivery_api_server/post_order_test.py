from webbrowser import Opera
import requests
import uuid
import json
from http import HTTPStatus

from delivery_api_server.info import DeliveryAPIServerData, Operation


def test_post():
    hub_collect_json = {
                        "task_id" : str(uuid.uuid4()),
                        "order":
                            {
                                "company_name": "barg",
                                "id": "barg" + str(uuid.uuid4()),
                                "description" : "a small item"
                            },

                        "unit":
                            {
                                "building_name" : "vovi_city",
                                "unit": "pantry"
                            },

                        "operation":
                            {
                                "task": Operation.HUB_DEPOSIT.value
                            }
                        }

    print(f"hub_collect_json: {hub_collect_json}")
    route = "/order"
    url = 'http://' +  DeliveryAPIServerData.server_ip + ":" +\
        str(DeliveryAPIServerData.port_num) + route
    response = requests.post(url, json=hub_collect_json)
    assert(response.status_code == HTTPStatus.OK.value )

if __name__ == '__main__':
    test_post()