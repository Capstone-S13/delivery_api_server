import requests
import uuid
from delivery_api_server.info import DeliveryAPIServerData
import json

def main():
    route = "/order"
    url = 'http://' +  DeliveryAPIServerData.server_ip + ":" + str(DeliveryAPIServerData.port_num) + route
    print(f"url: {url}")
    order = requests.get(url)
    print(order)

if __name__ == '__main__':
    main()