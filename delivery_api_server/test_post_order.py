from ast import Del
import requests
import uuid
from delivery_api_server.info import DeliveryAPIServerData
import json

def main():
    hub_collect_json = {
                        "order":
                            {
                                "company_name": "barg",
                                "id": "barg" + str(uuid.uuid4())
                            },

                        "unit":
                            {
                                "building_name" : "vovi_city",
                                "unit": "pantry"
                            },

                        "operation":
                            {
                                "task": 1
                            }
                        }

    print(f"hub_collect_json: {hub_collect_json}")
    route = "/order"
    url = 'http://' +  DeliveryAPIServerData.server_ip + ":" + str(DeliveryAPIServerData.port_num) + route
    print(f"url: {url}")
    order = requests.post(url, json=hub_collect_json)
    print(order)
if __name__ == '__main__':
    main()