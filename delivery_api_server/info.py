# info to include:
# delivery hub's waypoint on the rmf waypoint
# the mapping to the store address and shop waypoint in rmf (if needed) (we can standardise the two)
# the waypoint(s) for the indoor delivery robot to go to after making a hub delivery
# delivery company's fleet name
# delivery company's robot
# system server uri


from dataclasses import dataclass, field
from typing import List
from enum import Enum

class SystemOrderStatus(Enum):
    ORDER_SENT = 0
    ORDER_RECEIVED = 1
    ROBOT_DISPATCHED = 2
    AT_STORE_HUB = 3
    BETWEEN_HUBS = 4
    AT_DEST_HUB = 5
    ARRIVED = 6
    DELIVERED = 7
    CANCELLED = 8
    FAILED = 9

class SystemTaskStatus(Enum):
    TASK_UNDERWAY = 0
    TASK_COMPLETED = 1
    TASK_FAILED = 2


class Operation(Enum):
    HUB_DEPOSIT = 0
    HUB_COLLECT = 1


@dataclass
class SystemServerData:
    server_ip: str = "localhost"
    port_num: str = "8888"

    # routings
    order_status_route: str = "order-status"

@dataclass
class BuildingData:
    building_name: str = "vovi_city"
    units: List[str] = field(default_factory=list)
    ingress_points: List = field(default_factory=["e_1", "e_2"])
    egress_points: List[str] = field(default_factory=["e_1", "e_2"])
    hub: str = "coe"
    holding_point: str ="lounge"
    internal_fleet_name: str = "Unodopo"
    external_fleet_name: str = "External"
    internal_robot: str = "Unodopo1"
    external_robot: str = ""
    # TODO: Implpement a list of robots


@dataclass
class DeliveryAPIServerData:
    server_ip: str = "0.0.0.0"
    port_num: int = 7171
    ws_port_num: int = 7878

