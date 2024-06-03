import geopy.distance
import requests
import json
import time
import os
import sys
import datetime
import cv2
import geopy
import matplotlib.pyplot as plt

from renderer import CV2DRenderer
from utils import *

RUN_WITH_SAILOR_STANDARDS = False
DEGREE_SIGN = u'\N{DEGREE SIGN}'
TELEMETRY_SERVER_URL = 'http://107.23.136.207:8082/'

pid_data_file = None
telemetry_file = None
telemetry_start_time = time.time()

# MAP_BOUNDS = [[-25, -50], [100, 75]]
MAP_BOUNDS = [[-75, -75], [75, 75]]
BUOYS = [
    [42.8449667, -70.9772667],
    [42.8447667, -70.9771167],
    [42.8449167, -70.9761667],
    [42.8455167, -70.9765333]
]


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

def move_terminal_cursor(y, x):
    """
        Please look here as to how to interpret this abomination:
        https://stackoverflow.com/questions/54630766/how-can-move-terminal-cursor-in-python
    """
    print("\033[%d;%dH" % (y, x))


def hide_terminal_cursor():
    # if os.name == 'nt':
    #     ci = _CursorInfo()
    #     handle = ctypes.windll.kernel32.GetStdHandle(-11)
    #     ctypes.windll.kernel32.GetConsoleCursorInfo(handle, ctypes.byref(ci))
    #     ci.visible = False
    #     ctypes.windll.kernel32.SetConsoleCursorInfo(handle, ctypes.byref(ci))
    if os.name == 'posix':
        sys.stdout.write("\033[?25l")
        sys.stdout.flush()

def show_terminal_cursor():
    # if os.name == 'nt':
    #     ci = _CursorInfo()
    #     handle = ctypes.windll.kernel32.GetStdHandle(-11)
    #     ctypes.windll.kernel32.GetConsoleCursorInfo(handle, ctypes.byref(ci))
    #     ci.visible = True
    #     ctypes.windll.kernel32.SetConsoleCursorInfo(handle, ctypes.byref(ci))
    if os.name == 'posix':
        sys.stdout.write("\033[?25h")
        sys.stdout.flush()



def get_telemetry() -> dict:
    """
    Should return a dictionary with the following keys:
        position as a latitude, longitude tuple
        state as a string
        speed as a float in m/s
        bearing in degrees
        heading in degrees
        true_wind_speed in m/s
        true_wind_angle in degrees
        apparent_wind_speed in m/s
        apparent_wind_angle in degrees
        mast_angle in degrees
        rudder_angle in degrees
        current_waypoint as a latitude, longitude tuple
        current_route as a list of latitude, longitude tuples
    """
    telemetry = json.loads((requests.get(TELEMETRY_SERVER_URL + "api/latest").text))
    return json.loads(telemetry["s1"])


def update_telemetry_text(telemetry: dict):
    global telemetry_file, telemetry_start_time
        
    # Convert to the units that the sailors are happy with
    heading_cw_north = (90 - telemetry["heading"]) % 360        # ccw from true east -> cw from true north
    bearing_cw_north = (90 - telemetry["bearing"]) % 360        # ccw from true east -> cw from true north
    apparent_wind_angle_cw_centerline_from = (180 - telemetry["apparent_wind_angle"]) % 360         # ccw centerline measuring the direction the wind is blowing towards -> cw centerline measuring the direction the wind is blowing from
    apparent_wind_speed_knots = 1.94384 * telemetry["apparent_wind_speed"]                          # m/s -> knots
    true_wind_angle_cw_centerline_from =  (180 - telemetry["true_wind_angle"]) % 360                # ccw centerline measuring the direction the wind is blowing towards -> cw centerline measuring the direction the wind is blowing from
    true_wind_speed_knots = 1.94384 * telemetry["true_wind_speed"]                                  # m/s -> knots
    boat_speed_knots = 1.94384 * telemetry["speed"]                               # m/s -> knots
    
    current_waypoint_index = telemetry["current_waypoint_index"]
    
    if telemetry["current_route"]:
        distance_to_next_waypoint = geopy.distance.geodesic(telemetry["current_route"][current_waypoint_index], telemetry["position"])
    else:
        distance_to_next_waypoint = geopy.distance.Distance(0.)
    
    if RUN_WITH_SAILOR_STANDARDS:
        speed_unit = "kts"
        heading = heading_cw_north
        bearing = bearing_cw_north
        apparent_wind_angle = apparent_wind_angle_cw_centerline_from
        apparent_wind_speed = apparent_wind_speed_knots
        true_wind_angle = true_wind_angle_cw_centerline_from
        true_wind_speed = true_wind_speed_knots
        boat_speed = boat_speed_knots
        
    else:
        speed_unit = "m/s"
        heading = telemetry["heading"]
        bearing = telemetry["bearing"]
        apparent_wind_angle = telemetry["apparent_wind_angle"]
        apparent_wind_speed = telemetry["apparent_wind_speed"]
        true_wind_angle  = telemetry["true_wind_angle"]
        true_wind_speed = telemetry["true_wind_speed"]
        boat_speed = telemetry["speed"]
        
        
    # Get Formatted Time
    time_since_startup = (time.time() - telemetry_start_time)
    time_since_startup = datetime.time(
        hour=int((time_since_startup // 3600) % 24), 
        minute=int(time_since_startup // 60) % 60, 
        second=int(time_since_startup) % 60, 
        microsecond=int(time_since_startup * 10**6) % 10**6
    )
    time_since_startup_str = time_since_startup.strftime('%H:%M:%S.{:02.0f}').format(time_since_startup.microsecond/10000.0)
    
    real_life_date_time = datetime.datetime.now()
    real_life_date_time_str = real_life_date_time.strftime('%m-%d-%Y %H:%M:%S.{:02.0f}').format(real_life_date_time.microsecond/10000.0)
    # Construct String to Display to Command Line
    string_to_show = ""
    string_to_show += f"Time Today: {real_life_date_time_str}                                                                                                  \n"
    string_to_show += f"Time Since Start Up: {time_since_startup_str}                                                                                          \n"
    string_to_show += f"GPS Latitude: {telemetry['position'][0]:.8f}, GPS Longitude: {telemetry['position'][1]:.8f}                                            \n"
    string_to_show += f"Autopilot Mode: {telemetry['state']}                                                                                                   \n"
    string_to_show += f"Speed Over Ground: {boat_speed:.2f} {speed_unit}                                                                                       \n"
    string_to_show += f"Target Heading: {bearing:.2f}{DEGREE_SIGN}                                                                                             \n"
    string_to_show += f"Heading: {heading:.2f}{DEGREE_SIGN}                                                                                                    \n"
    string_to_show += f"True Wind Speed: {true_wind_speed:.2f} {speed_unit}, True Wind Angle {true_wind_angle:.2f}{DEGREE_SIGN}                                \n"
    string_to_show += f"Apparent Wind Speed: {apparent_wind_speed:.2f} {speed_unit}, Apparent Wind Angle: {apparent_wind_angle:.2f}{DEGREE_SIGN}               \n"
    string_to_show += f"Target Mast Angle: {telemetry['mast_angle']:.2f}{DEGREE_SIGN}                                                                          \n"
    string_to_show += f"Target Rudder Angle: {telemetry['rudder_angle']:.2f}{DEGREE_SIGN}                                                                      \n"
    string_to_show += f"Current Waypoint Index: {current_waypoint_index}                                                                                       \n"
    string_to_show += f"Distance to next waypoint: {distance_to_next_waypoint.m:.2f} meters                                                                        \n"
    string_to_show += "                                                                                                                                        \n"
    
    string_to_show += f"Parameters:                                                                                                                            \n"
    string_to_show += f"------------------------------------                                                                                                   \n"
    for param_name, param_value in telemetry["parameters"].items():
        string_to_show += f"{param_name}: {param_value}                                                 \n"
    
    string_to_show += "                                                                                                                                        \n"
    string_to_show += f"Current Route:                                                                                                                         \n"
    string_to_show += f"------------------------------------                                                                                                   \n"
    for index, waypoint in enumerate(telemetry["current_route"]):
        string_to_show += f"Waypoint {index} Latitude: {waypoint[0]:.8f}, Waypoint {index} Longitude: {waypoint[1]:.8f}                                        \n"
    
    
    trailing_white_space = ""
    for i in range(6 - len(telemetry["current_route"])):
        trailing_white_space += "                                                                                                                                      \n"
    
    # Display String and Write to Telemetry File
    move_terminal_cursor(0, 0)
    print(string_to_show + trailing_white_space)
    
    telemetry_file.write(string_to_show)

    

def display_image(img):
    cv2.imshow("Ground Station GUI", img)
    cv2.waitKey(1)
        
def update_telemetry_gui(renderer: CV2DRenderer, telemetry: dict):
    local_y, local_x = 0, 0
    absolute_wind_angle = telemetry["true_wind_angle"] + telemetry["heading"]
    mast_dir_fix = -1 if 0 < telemetry["true_wind_angle"] < 180 else 1
    
    TWS = telemetry["true_wind_speed"]
    TWA = telemetry["true_wind_angle"]
    AWS = telemetry["apparent_wind_speed"]
    AWA = telemetry["apparent_wind_angle"]
    
    true_wind_vector = np.array([TWS * np.cos(np.deg2rad(TWA-90)), TWS * np.sin(np.deg2rad(TWA-90))])
    apparent_wind_vector = np.array([AWS * np.cos(np.deg2rad(AWA-90)), AWS * np.sin(np.deg2rad(AWA-90))])
    velocity_vector = (true_wind_vector - apparent_wind_vector)
    
    gui_state = State()
    gui_state["p_boat"] = np.array([local_x, local_y, 0])
    gui_state["dt_p_boat"] = velocity_vector
    gui_state["theta_boat"] = np.array([0, 0, np.deg2rad(telemetry["heading"])])
    gui_state["dt_theta_boat"] = np.array([0, 0, 0])
    gui_state["theta_rudder"] = np.array([0, 0, 0])
    gui_state["dt_theta_rudder"] = np.array([0, 0, 0])
    gui_state["theta_sail"] = np.array([mast_dir_fix * np.deg2rad(telemetry["mast_angle"]), 0, 0])
    gui_state["dt_theta_sail"] = np.array([0, 0, 0])
    gui_state["apparent_wind"] = telemetry["apparent_wind_angle"]
    gui_state["wind"] = np.array([telemetry["true_wind_speed"] * np.cos(np.deg2rad(absolute_wind_angle)), telemetry["true_wind_speed"] * np.sin(np.deg2rad(absolute_wind_angle))])
    gui_state["water"] = np.array([0, 0])
    gui_state["buoys"] = np.array(BUOYS)
    gui_state["cur_waypoint"] = telemetry["current_waypoint_index"]
    
    waypoints = []
    for waypoint in telemetry["current_route"]:
        local_y, local_x, _ = navpy.lla2ned(waypoint[0], waypoint[1], 0, telemetry["position"][0], telemetry["position"][1], 0)
        local_x = np.clip(local_x, MAP_BOUNDS[0][0], MAP_BOUNDS[1][0])
        local_y = np.clip(local_y, MAP_BOUNDS[0][1], MAP_BOUNDS[1][1])
        
        waypoints.append((local_x, local_y))
    gui_state["waypoints"] = np.array(waypoints)
    
    
    buoys = []
    for buoy in BUOYS:
        # print(buoy[0], buoy[1])
        # print(telemetry["position"][0], telemetry["position"][1])
        local_y, local_x, _ = navpy.lla2ned(buoy[0], buoy[1], 0, telemetry["position"][0], telemetry["position"][1], 0)
        # print(local_x, local_y)
        local_x = np.clip(local_x, MAP_BOUNDS[0][0], MAP_BOUNDS[1][0])
        local_y = np.clip(local_y, MAP_BOUNDS[0][1], MAP_BOUNDS[1][1])
        
        buoys.append((local_x, local_y))
    gui_state["buoys"] = np.array(buoys)

    display_image(renderer.render(gui_state))


def update_heading_pid_graph(telemetry):
    global pid_data_file

    heading = telemetry["heading"]
    desired_heading = telemetry["bearing"]
    current_time = time.time() - telemetry_start_time
    
    pid_data_file.write(f"{current_time},{heading},{desired_heading}\n")




def main():
    global telemetry_file, pid_data_file, renderer
    # if os.name == 'nt':
    #     import msvcrt
    #     import ctypes

    #     class _CursorInfo(ctypes.Structure):
    #         _fields_ = [("size", ctypes.c_int),
    #                     ("visible", ctypes.c_byte)]
    
    clear_screen()
    hide_terminal_cursor()
    telemetry = get_telemetry()
    
    map_bounds = np.array(MAP_BOUNDS)
    renderer = CV2DRenderer()
    renderer.setup(map_bounds)
    
    telemetry_file = open("./telemetry_log.txt", "a")
    pid_data_file = open("./pid_data.csv", "w")
    
    pid_data_file.write("time,heading,desired_heading\n")
    
    plt.ion()
    
    
    while True:
        telemetry = get_telemetry()
        update_telemetry_text(telemetry)
        update_telemetry_gui(renderer, telemetry)
        update_heading_pid_graph(telemetry)
        
        
        time.sleep(0.05)
    
    
if __name__ == "__main__": 
    try:
        main()
    finally:
        clear_screen()
        show_terminal_cursor()
        telemetry_file.close()
        
