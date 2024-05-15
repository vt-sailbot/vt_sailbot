from utils import *
from typing import List
import cv2
import numpy as np
from pydantic.utils import deep_update



BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (200, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 120, 0)
CYAN = (0, 255, 255)


    
class RendererState():
    def __init__(self, state: State):
        # heading angle
        self.theta_boat = state["theta_boat"][2]
        self.dt_theta_boat = np.abs(state["dt_theta_boat"][2]) * \
            angle_to_vec(self.theta_boat +
                         np.sign(state["dt_theta_boat"][2]) * np.pi / 2)  # is either -90 or 90 degrees

        # position
        self.p_boat = np.array([state["p_boat"][0], state["p_boat"][1]])
        self.dt_p_boat = rotate_vector(
            np.array([state["dt_p_boat"][0], state["dt_p_boat"][1]]),
            self.theta_boat)

        # rudder
        self.theta_rudder = np.pi + self.theta_boat + state["theta_rudder"][0]
        self.dt_rudder = np.abs(state["dt_theta_rudder"][0]) * \
            angle_to_vec(self.theta_rudder +
                         np.sign(state["dt_theta_rudder"][0]) * np.pi / 2)  # is either -90 or 90 degrees

        # sail
        self.theta_sail = np.pi + self.theta_boat + state["theta_sail"][0]
        self.dt_sail = np.abs(state["dt_theta_sail"][0]) * \
            angle_to_vec(self.theta_sail
                         + np.sign(state["dt_theta_sail"][0]) * np.pi / 2)  # is either -90 or 90 degrees

        # wind
        self.wind = state["wind"]

        # water
        self.water = state["water"]


class CV2DRenderer():
    def __init__(self, size=512, padding=30, vector_scale=10, style={}):
        self.size = size
        self.padding = padding
        self.vector_scale = vector_scale
        self.map_bounds = None
        self.center = None

        self.style = {
            "background": WHITE,
            "border": {
                "color": rgba(BLACK, .2),
                "width": 1,
            },
            "boat": {
                "color": rgba(BLACK, .3),
                "spike_coef": 2,
                "size": 10,
                "phi": np.deg2rad(40),
                "dt_p": {
                    "color": GREEN,
                    "width": 1,
                },
                "dt_theta": {
                    "color": RED,
                    "width": 1,
                },
                "center": {
                    "color": WHITE,
                    "radius": 2,
                }
            },
            "rudder": {
                "color": rgba(BLACK, .3),
                "width": 2,
                "height": 5,
                "dt_theta": {
                    "color": RED,
                    "width": 1,
                },
            },
            "sail": {
                "color": rgba(BLACK, .7),
                "width": 2,
                "height": 15,
                "dt_theta": {
                    "color": RED,
                    "width": 1,
                },
            },
            "wind": {
                "color": rgba(BLUE, .5),
                "width": 2,
            },
            "water": {
                "color": rgba(CYAN, .5),
                "width": 2,
            },
        }
        self.style = deep_update(self.style, style)

    def _create_empty_img(self):
        bg = np.array(self.style["background"])
        img = bg[None, None, :] + np.zeros((self.size, self.size, 3))
        return img.astype(np.uint8)

    def _scale_to_fit_in_img(self, x):
        return x / (self.map_bounds[1] - self.map_bounds[0]).max() * (self.size - 2 * self.padding)

    def _translate_and_scale_to_fit_in_map(self, x):
        return self._scale_to_fit_in_img(x - self.map_bounds[0]) + self.padding

    def _transform_state_to_fit_in_img(self, state: RendererState):
        # translate and scale positions
        state.p_boat = self._translate_and_scale_to_fit_in_map(state.p_boat)

        # scale vectors
        state.dt_p_boat = self._scale_to_fit_in_img(state.dt_p_boat)
        state.dt_theta_boat = self._scale_to_fit_in_img(state.dt_theta_boat)
        state.dt_rudder = self._scale_to_fit_in_img(state.dt_rudder)
        state.dt_sail = self._scale_to_fit_in_img(state.dt_sail)
        state.wind = self._scale_to_fit_in_img(state.wind)
        state.water = self._scale_to_fit_in_img(state.water)

    def _draw_borders(self, img: np.ndarray):
        borders = self._translate_and_scale_to_fit_in_map(
            self.map_bounds).astype(int)
        cv2.rectangle(img,
                      tuple(borders[0]),
                      tuple(borders[1]),
                      self.style["border"]["color"],
                      self.style["border"]["width"],
                      lineType=cv2.LINE_AA)

    def _draw_wind(self, img: np.ndarray, state: RendererState):
        img_center = np.array([self.size, self.size]) / 2
        cv2.arrowedLine(img,
                        tuple(img_center.astype(int)),
                        tuple((img_center + state.wind *
                              self.vector_scale).astype(int)),
                        self.style["wind"]["color"],
                        self.style["wind"]["width"],
                        tipLength=0.2,
                        line_type=cv2.LINE_AA)

    def _draw_water(self, img: np.ndarray, state: RendererState):
        img_center = np.array([self.size, self.size]) / 2
        cv2.arrowedLine(img,
                        tuple(img_center.astype(int)),
                        tuple((img_center + state.water *
                              self.vector_scale).astype(int)),
                        self.style["water"]["color"],
                        self.style["water"]["width"],
                        tipLength=0.2,
                        line_type=cv2.LINE_AA)

    def _draw_boat(self, img: np.ndarray, state: RendererState):
        boat_size = self.style["boat"]["size"]
        phi = self.style["boat"]["phi"]
        spike_coeff = self.style["boat"]["spike_coef"]
        sailboat_pts = np.array([
            [state.p_boat + angle_to_vec(state.theta_boat + phi) * boat_size],
            [state.p_boat + angle_to_vec(state.theta_boat +
                                       (np.pi - phi)) * boat_size],
            [state.p_boat + angle_to_vec(state.theta_boat +
                                       (np.pi + phi)) * boat_size],
            [state.p_boat + angle_to_vec(state.theta_boat - phi) * boat_size],
            [state.p_boat + angle_to_vec(state.theta_boat)
             * spike_coeff * boat_size]
        ], dtype=int)
        cv2.fillConvexPoly(img,
                           sailboat_pts,
                           self.style["boat"]["color"],
                           lineType=cv2.LINE_AA)

    def _draw_sail(self, img: np.ndarray, state: RendererState):
        sail_height = self.style["sail"]["height"]
        sail_start = state.p_boat
        sail_end = sail_start + angle_to_vec(state.theta_sail) * sail_height
        cv2.line(img,
                 tuple(sail_start.astype(int)),
                 tuple(sail_end.astype(int)),
                 self.style["sail"]["color"],
                 self.style["sail"]["width"],
                 lineType=cv2.LINE_AA)

    def _draw_rudder(self, img: np.ndarray, state: RendererState):
        rudder_height = self.style["rudder"]["height"]
        boat_phi = self.style["boat"]["phi"]
        boat_size = self.style["boat"]["size"]
        back_of_boat = state.p_boat + \
            angle_to_vec(np.pi + state.theta_boat) * \
            np.cos(boat_phi) * boat_size
        rudder_start = back_of_boat
        rudder_end = rudder_start + \
            angle_to_vec(state.theta_rudder) * rudder_height
        cv2.line(img,
                 tuple(rudder_start.astype(int)),
                 tuple(rudder_end.astype(int)),
                 self.style["rudder"]["color"],
                 self.style["rudder"]["width"],
                 lineType=cv2.LINE_AA)

    def _draw_boat_pos_velocity(self, img: np.ndarray, state: RendererState):
        state.dt_p_boat
        dt_p_boat_start = state.p_boat
        dt_p_boat_end = dt_p_boat_start + state.dt_p_boat * self.vector_scale
        cv2.arrowedLine(img,
                        tuple(dt_p_boat_start.astype(int)),
                        tuple(dt_p_boat_end.astype(int)),
                        self.style["boat"]["dt_p"]["color"],
                        self.style["boat"]["dt_p"]["width"],
                        tipLength=.2,
                        line_type=cv2.LINE_AA)

    def _draw_boat_heading_velocity(self, img: np.ndarray, state: RendererState):
        spike_coeff = self.style["boat"]["spike_coef"]
        boat_size = self.style["boat"]["size"]
        front_of_boat = state.p_boat + \
            angle_to_vec(state.theta_boat) * spike_coeff * boat_size
        state.dt_p_boat
        dt_theta_boat_start = front_of_boat
        dt_theta_boat_end = dt_theta_boat_start + state.dt_theta_boat * self.vector_scale
        cv2.arrowedLine(img,
                        tuple(dt_theta_boat_start.astype(int)),
                        tuple(dt_theta_boat_end.astype(int)),
                        self.style["boat"]["dt_theta"]["color"],
                        self.style["boat"]["dt_theta"]["width"],
                        tipLength=.2,
                        line_type=cv2.LINE_AA)

    def _draw_rudder_velocity(self, img: np.ndarray, state: RendererState):
        boat_phi = self.style["boat"]["phi"]
        boat_size = self.style["boat"]["size"]
        rudder_height = self.style["rudder"]["height"]
        back_of_boat = state.p_boat + \
            angle_to_vec(np.pi + state.theta_boat) * \
            np.cos(boat_phi) * boat_size
        dt_rudder_start = back_of_boat + \
            angle_to_vec(state.theta_rudder) * rudder_height
        dt_rudder_end = dt_rudder_start + state.dt_rudder
        cv2.arrowedLine(img,
                        tuple(dt_rudder_start.astype(int)),
                        tuple(dt_rudder_end.astype(int)),
                        self.style["rudder"]["dt_theta"]["color"],
                        self.style["rudder"]["dt_theta"]["width"],
                        tipLength=.2,
                        line_type=cv2.LINE_AA)

    def _draw_sail_velocity(self, img: np.ndarray, state: RendererState):
        sail_height = self.style["sail"]["height"]
        dt_sail_start = state.p_boat + \
            angle_to_vec(state.theta_sail) * sail_height
        dt_sail_end = dt_sail_start + state.dt_sail
        cv2.arrowedLine(img,
                        tuple(dt_sail_start.astype(int)),
                        tuple(dt_sail_end.astype(int)),
                        self.style["sail"]["dt_theta"]["color"],
                        self.style["sail"]["dt_theta"]["width"],
                        tipLength=.2,
                        line_type=cv2.LINE_AA)

    def _draw_boat_center(self, img: np.ndarray, state: RendererState):
        cv2.circle(img,
                   tuple(state.p_boat.astype(int)),
                   self.style["boat"]["center"]["radius"],
                   self.style["boat"]["center"]["color"],
                   -1)

    def _draw_waypoint(self, img: np.ndarray, x_position, y_position):
        cv2.circle(img, (x_position, y_position), radius=1, color=(255, 0, 0), thickness=-1)
        
    def get_render_mode(self) -> str:
        return 'rgb_array'

    def get_render_modes(self) -> List[str]:
        return ['rgb_array']

    def setup(self, map_bounds):
        self.map_bounds = map_bounds[:, 0:2]  # ignore z axis
        self.center = (self.map_bounds[0] + self.map_bounds[1]) / 2

    def render(self, state, local_waypoints: tuple, draw_extra_fct=None):
        """
        Args:
            state (State): A State object (from utils.py) that represents the boat's current state (stuff like position, speed, wind, etc).
            local_waypoints (list[tuples]): A list of all of the desired waypoints to draw in local coordinates.

        Returns:
            np.ndarray: the image to be displayed
        """
        
        assert (self.map_bounds is not None
                and self.center is not None), "Please call setup() first."

        img = self._create_empty_img()

        # prepare state
        state = RendererState(state)
        self._transform_state_to_fit_in_img(state)

        # draw extra stuff
        if draw_extra_fct is not None:
            draw_extra_fct(img, state)

        # draw map
        self._draw_borders(img)
        self._draw_wind(img, state)
        self._draw_water(img, state)
        self._draw_boat(img, state)
        self._draw_boat_heading_velocity(img, state)
        self._draw_boat_pos_velocity(img, state)
        self._draw_rudder(img, state)
        self._draw_rudder_velocity(img, state)
        self._draw_sail(img, state)
        self._draw_sail_velocity(img, state)
        self._draw_boat_center(img, state)

        for (x, y) in local_waypoints:
            self._draw_waypoint(img, x, y)
            
        # flip vertically
        img = img[::-1, :, :]

        return img
