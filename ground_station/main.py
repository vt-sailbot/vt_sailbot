import requests
import json
import time
import os
import sys
import datetime
import cv2

from renderer import CV2DRenderer
from utils import *


DEGREE_SIGN = u'\N{DEGREE SIGN}'
TELEMETRY_SERVER_URL = 'http://107.23.136.207:8082/'

telemetry_file = None
telemetry_start_time = time.time()


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

def move_terminal_cursor(y, x):
    """
        Please look here as to how to interpret this abomination:
        https://stackoverflow.com/questions/54630766/how-can-move-terminal-cursor-in-python
    """
    print("\033[%d;%dH" % (y, x))


def hide_terminal_cursor():
    if os.name == 'nt':
        ci = _CursorInfo()
        handle = ctypes.windll.kernel32.GetStdHandle(-11)
        ctypes.windll.kernel32.GetConsoleCursorInfo(handle, ctypes.byref(ci))
        ci.visible = False
        ctypes.windll.kernel32.SetConsoleCursorInfo(handle, ctypes.byref(ci))
    elif os.name == 'posix':
        sys.stdout.write("\033[?25l")
        sys.stdout.flush()

def show_terminal_cursor():
    if os.name == 'nt':
        ci = _CursorInfo()
        handle = ctypes.windll.kernel32.GetStdHandle(-11)
        ctypes.windll.kernel32.GetConsoleCursorInfo(handle, ctypes.byref(ci))
        ci.visible = True
        ctypes.windll.kernel32.SetConsoleCursorInfo(handle, ctypes.byref(ci))
    elif os.name == 'posix':
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
    string_to_show += f"Time Today: {real_life_date_time_str}                                                                                                     \n"
    string_to_show += f"Time Since Start Up: {time_since_startup_str}                                                                                             \n"
    string_to_show += f"GPS Latitude: {telemetry['position'][0]}, GPS Longitude: {telemetry['position'][1]}                                                       \n"
    string_to_show += f"Current State: {telemetry['state']}                                                                                                       \n"
    string_to_show += f"Speed Over Ground (knots): {(str(telemetry['speed']))}                                                                                   \n"
    string_to_show += f"Target Heading: {str(telemetry['bearing'])  + DEGREE_SIGN}                                                                                \n"
    string_to_show += f"Heading: {str(telemetry['heading']) + DEGREE_SIGN}                                                                                        \n"
    string_to_show += f"True Wind Speed (knots): {telemetry['true_wind_speed']}, True Wind Angle {str(telemetry['true_wind_angle']) + DEGREE_SIGN}                                                     \n"
    string_to_show += f"Apparent Wind Speed (knots): {telemetry['apparent_wind_speed']}, Apparent Wind Angle: {str(telemetry['apparent_wind_angle']) + DEGREE_SIGN}                                 \n"
    string_to_show += f"Target Mast Angle: {str(telemetry['mast_angle']) + DEGREE_SIGN}                                                                           \n"
    string_to_show += f"Target Rudder Angle: {str(telemetry['rudder_angle']) + DEGREE_SIGN}                                                                       \n"
    string_to_show += f"Current Waypoint Latitude: {telemetry['current_waypoint'][0]}, Current Waypoint Latitude: {telemetry['current_waypoint'][1]}                                                   \n"
    string_to_show += "                                                                                                                                           \n"
    string_to_show += f"Current Route:                                                                                                                            \n"
    string_to_show += f"------------------------------------                                                                                                      \n"
    for index, waypoint in enumerate(telemetry["current_route"]):
        string_to_show += f"Waypoint {index} Latitude: {waypoint[0]}, Waypoint {index} Longitude: {waypoint[1]}                                              \n"
    string_to_show += "\n\n\n"
    
    
    # Display String and Write to Telemetry File
    move_terminal_cursor(0, 0)
    print(string_to_show)
    telemetry_file.write(string_to_show)

    

def display_image(img):
    cv2.imshow("Simulation Real Time", img)
    cv2.waitKey(1)
        
def update_telemetry_gui(renderer: CV2DRenderer, reference_lat_lon, telemetry: dict):
    
    local_y, local_x, _ = navpy.lla2ned(telemetry["position"][0], telemetry["position"][1], 0, reference_lat_lon[0], reference_lat_lon[1], 0)
    absolute_wind_angle = telemetry["true_wind_angle"] + telemetry["heading"]
    mast_dir_fix = -1 if 0 < telemetry["true_wind_angle"] < 180 else 1
    
    gui_state = State()
    gui_state["p_boat"] = np.array([local_x, local_y, 0])
    gui_state["dt_p_boat"] = np.array([0, 0, 0])
    gui_state["theta_boat"] = np.array([0, 0, np.deg2rad(telemetry["heading"])])
    gui_state["dt_theta_boat"] = np.array([0, 0, 0])
    gui_state["theta_rudder"] = np.array([0, 0, 0])
    gui_state["dt_theta_rudder"] = np.array([0, 0, 0])
    gui_state["theta_sail"] = np.array([mast_dir_fix * np.deg2rad(telemetry["mast_angle"]), 0, 0])
    gui_state["dt_theta_sail"] = np.array([0, 0, 0])
    gui_state["wind"] = np.array([telemetry["true_wind_speed"] * np.cos(np.deg2rad(absolute_wind_angle)), telemetry["true_wind_speed"] * np.sin(np.deg2rad(absolute_wind_angle))])
    gui_state["water"] = np.array([0, 0])
    
    waypoints = []
    for waypoint in telemetry["current_route"]:
        local_y, local_x, _ = navpy.lla2ned(waypoint[0], waypoint[1], 0, reference_lat_lon[0], reference_lat_lon[1], 0)
        waypoints.append((local_x, local_y))
        
    display_image(renderer.render(gui_state, waypoints))


def main():
    global telemetry_file, renderer
    if os.name == 'nt':
        import msvcrt
        import ctypes

        class _CursorInfo(ctypes.Structure):
            _fields_ = [("size", ctypes.c_int),
                        ("visible", ctypes.c_byte)]
    
    clear_screen()
    hide_terminal_cursor()
    telemetry = get_telemetry()
    starting_position_lat_lon = telemetry["position"]
    
    map_bounds = np.array([[0, -10], [20, 10]])
    renderer = CV2DRenderer()
    renderer.setup(map_bounds)
    
    telemetry_file = open("./telemetry_log.txt", "a")
    
    
    while True:
        telemetry = get_telemetry()
        update_telemetry_text(telemetry)
        update_telemetry_gui(renderer, starting_position_lat_lon, telemetry)
        
        time.sleep(0.05)
    
    
if __name__ == "__main__": 
    try:
        main()
    finally:
        # clear_screen()
        show_terminal_cursor()
        telemetry_file.close()
        