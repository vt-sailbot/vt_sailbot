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


# @functools.lru_cache(maxsize=1)
# def is_profiling():
#     return os.getenv('PROFILING') is not None


# @functools.lru_cache(maxsize=1)
# def is_profiling_all():
#     return os.getenv('PROFILING') == 'all'


# def profiling(func, prefix=''):
#     stats = {'count': 0, 'first_call_time': None, 'duration': 0}

#     def flush():
#         freq = stats['count'] / (time.time() - stats['first_call_time'])
#         mean_duration = stats['duration'] / stats['count']
#         print(f'[{prefix + func.__name__}]\tcount: {stats["count"]}\tfreq: {freq:.2f}/s\tduration: ~{mean_duration / 1e-3:.2f}ms')

#     def wrapper(*args, **kwargs):
#         if not is_profiling():
#             return func(*args, **kwargs)

#         # register flush function to be called at exit
#         if stats['first_call_time'] is None:
#             stats['first_call_time'] = time.time()
#             atexit.register(flush)

#         stats['count'] += 1
#         t0 = time.time()
#         res = func(*args, **kwargs)
#         t1 = time.time()
#         stats['duration'] += t1 - t0
#         return res
#     return wrapper

# class ProfilingMeta(type):
#     def __new__(cls, name, bases, attrs):
#         for key, value in attrs.items():
#             if callable(value) and (not '__' in key or is_profiling_all()):
#                 attrs[key] = profiling(value, prefix=f'{name}.')
#         return super(ProfilingMeta, cls).__new__(cls, name, bases, attrs)
    
    
    
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