import requests
import json
import time
import os
import sys
import math

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



def update_telemetry_gui():
    global telemetry_file, telemetry_start_time
    
    telemetry_list = json.loads(requests.get(TELEMETRY_SERVER_URL + "api/latest").text)
    telemetry = telemetry_list["s1"].split("; ")
    
    gps_lat_lon = telemetry[0].split(", ")
    cur_waypoint_lat_lon = telemetry[9].split(", ")
    
    current_route = []
    for waypoint in telemetry[10:]:
        if waypoint == "": continue     # sanity check don't remove this was causing issues
        current_route.append(tuple(waypoint.split(", ")))
    
    
    string_to_show = ""
    string_to_show += f"Time: {time.time() - telemetry_start_time}                                                                       \n"
    string_to_show += f"GPS Latitude: {gps_lat_lon[0]}, GPS Longitude: {gps_lat_lon[1]}                                                  \n"
    string_to_show += f"Current State: {telemetry[1]}                                                                                    \n"
    string_to_show += f"Speed Over Ground (m/s): {telemetry[2]}                                                                          \n"
    string_to_show += f"Bearing: {telemetry[3]}                                                                                          \n"
    string_to_show += f"Heading: {telemetry[4]}                                                                                          \n"
    string_to_show += f"Wind Speed (m/s): {telemetry[5]}, Wind Direction: {telemetry[6]}                                                 \n"
    string_to_show += f"Target Mast Angle: {telemetry[7]}                                                                                \n"
    string_to_show += f"Target Rudder Angle: {telemetry[8]}                                                                              \n"
    string_to_show += f"Current Waypoint Latitude: {cur_waypoint_lat_lon[0]}, Current Waypoint Latitude: {cur_waypoint_lat_lon[1]}       \n"
    
    string_to_show += "\n"
    string_to_show += f"Current Route:                                                                                                   \n"
    string_to_show += f"------------------------------------                                                                             \n"
    for index, waypoint in enumerate(current_route):
        string_to_show += f"Waypoint {index} Latitude: {waypoint[0]}, Waypoint {index} Longitude: {waypoint[1]}                          \n"
    string_to_show += "\n\n\n"
    
    move_terminal_cursor(0, 0)
    print(string_to_show)
    
    telemetry_file.write(string_to_show)
    

def main():
    global telemetry_file
    if os.name == 'nt':
        import msvcrt
        import ctypes

        class _CursorInfo(ctypes.Structure):
            _fields_ = [("size", ctypes.c_int),
                        ("visible", ctypes.c_byte)]
    
    clear_screen()
    hide_terminal_cursor()
    
    telemetry_file = open("./telemetry_log.txt", "a")

    
    while True:
        update_telemetry_gui()
        time.sleep(0.05)
    
    
if __name__ == "__main__": 
    try:
        main()
    finally:
        clear_screen()
        show_terminal_cursor()
        telemetry_file.close()
        