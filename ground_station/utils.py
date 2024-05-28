import functools
import numpy as np
import os
import time
import atexit
from typing import TypedDict
import navpy


def latlon_to_local(lat, lon, ref_lat, ref_lon):
    North, East, Down = navpy.lla2ned(lat, lon, 0, ref_lat, ref_lon, 0)
    return np.array([North, East])


def rgba(color, alpha, background=np.array([255, 255, 255])):
    return tuple((1 - alpha) * background + alpha * np.array(color))


def angle_to_vec(angle):
    return np.array([np.cos(angle), np.sin(angle)])


def rotate_vector(vector: np.ndarray, angle: float):
    assert vector.shape == (2,)
    rot = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])
    return rot @ vector
    
    
class State(TypedDict):
    p_boat: np.ndarray[3]
    dt_p_boat: np.ndarray[3]
    theta_boat: np.ndarray[3]
    dt_theta_boat: np.ndarray[3]
    theta_rudder: np.ndarray[1]
    dt_theta_rudder: np.ndarray[1]
    theta_sail: np.ndarray[1]
    dt_theta_sail: np.ndarray[1]
    wind: np.ndarray[2]
    water: np.ndarray[2]
    waypoints: np.ndarray
    
    
    