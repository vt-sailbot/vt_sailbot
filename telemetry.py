import requests
import json
import time
import os

TELEMETRY_SERVER_URL = 'http://107.23.136.207:8082/'

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

    
while True:
    
    telemetry_list = json.loads(requests.get(TELEMETRY_SERVER_URL + "api/latest").text)
    telemetry = telemetry_list["s1"].split("; ")
    gps_lat_lon = telemetry[0].split(", ")
    next_destination_lat_lon = telemetry[5].split(", ")
    
    clear_screen()
    
    print(f"GPS Latitude: {gps_lat_lon[0]}, GPS Longitude: {gps_lat_lon[1]}")
    print(f"Speed Over Ground (m/s): {telemetry[1]}")
    print(f"Heading: {telemetry[2]}")
    print(f"Wind Speed (m/s): {telemetry[3]}, Wind Direction: {telemetry[4]}")
    print(f"Next Waypoint Latitude: {next_destination_lat_lon[0]}, Next Waypoint Longitude: {next_destination_lat_lon[1]}")
    print(f"Desired Mast Angle: {telemetry[6]}")
    print(f"Desired Rudder Angle: {telemetry[7]}")
    
    # telemetry_string = "hi there"
    # print(requests.post(url= TELEMETRY_SERVER_URL + "api/", json={"s1": telemetry_string, "s2": "1"}).text)
    
    time.sleep(0.1)