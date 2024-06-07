from enum import Enum

class AutopilotMode(Enum):
    Disabled = 0
    Hold_Best_Sail = 1
    Hold_Heading = 2
    Hold_Heading_And_Best_Sail = 3
    Waypoint_Mission = 4