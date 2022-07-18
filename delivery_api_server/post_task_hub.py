from webbrowser import Opera
import requests
import uuid
import json
from http import HTTPStatus

from delivery_api_server.info import DeliveryAPIServerData, Operation


def main():
    hub_collect_json = {
                        "task_id" : f'{str(uuid.uuid4())}',
                        "order":
                            {
                                "company_name": "barg",
                                "id": "barg" + str(uuid.uuid4()),
                                "description" : "a small item"
                            },

                        "operation":
                            {
                                "task": Operation.HUB_DEPOSIT.value
                            }
                        }

    print(f"hub_collect_json: {hub_collect_json}")
    route = "/internal-task"
    url = 'http://' +  DeliveryAPIServerData.server_ip + ":" +\
        str(DeliveryAPIServerData.port_num) + route
    response = requests.post(url, json=hub_collect_json)
    assert(response.status_code == HTTPStatus.OK.value )

if __name__ == '__main__':
    main()