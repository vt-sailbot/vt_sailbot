import requests
import pandas as pd
import time
import json

TELEMETRY_SERVER_URL = 'http://107.23.136.207:8082/'

def main():
    """
    Just a little script that sends the waypoints specified in desired_waypoints.csv to the server :)
    """
    parameters = json.dumps(json.load(open("desired_parameters.json")))
    
    print(requests.post(TELEMETRY_SERVER_URL + "api/wp/one", json={"w": parameters}).text)

if __name__ == "__main__": main()