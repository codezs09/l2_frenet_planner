import numpy as np
import os
import sys

sys.path.append("build/")
import FotWrapper
_to_frenet_initial_conditions = FotWrapper.to_frenet_initial_conditions
_run_fot = FotWrapper.run_fot

# from ctypes import c_double, c_int, POINTER, Structure, CDLL, byref

# try:
#     from py_cpp_struct import FrenetInitialConditions, FrenetHyperparameters, \
#         FrenetReturnValues
# except:
#     from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory \
#         .py_cpp_struct import FrenetInitialConditions, FrenetHyperparameters, \
#          FrenetReturnValues

# try:
#     cdll = CDLL("build/libFrenetOptimalTrajectory.so")
# except:
#     cdll = CDLL("{}/dependencies/frenet_optimal_trajectory_planner/"
#                 "build/libFrenetOptimalTrajectory.so".format(
#                     os.getenv("PYLOT_HOME")))

# _c_double_p = POINTER(c_double)

# # func / return type declarations for C++ run_fot
# _run_fot = cdll.run_fot
# _run_fot.argtypes = (
#     POINTER(FrenetInitialConditions),
#     POINTER(FrenetHyperparameters),
#     POINTER(FrenetReturnValues),
# )
# _run_fot.restype = None

# # func / return type declarations for C++ to_frenet_initial_conditions
# _to_frenet_initial_conditions = cdll.to_frenet_initial_conditions
# _to_frenet_initial_conditions.restype = None
# _to_frenet_initial_conditions.argtypes = (c_double, c_double, c_double,
#                                           c_double, c_double, c_double,
#                                           _c_double_p, _c_double_p, c_int,
#                                           _c_double_p)


def _parse_hyperparameters(hp):
    ans = FotWrapper.FrenetHyperparameters()
    ans.max_speed = hp["max_speed"]
    ans.max_accel = hp["max_accel"]
    ans.max_curvature = hp["max_curvature"]
    ans.max_road_width_l = hp["max_road_width_l"]
    ans.max_road_width_r = hp["max_road_width_r"]
    ans.d_road_w = hp["d_road_w"]
    ans.dt = hp["dt"]
    ans.maxt = hp["maxt"]
    ans.mint = hp["mint"]
    ans.d_t_s = hp["d_t_s"]
    ans.n_s_sample = hp["n_s_sample"]
    ans.obstacle_clearance = hp["obstacle_clearance"]
    ans.kd = hp["kd"]
    ans.kv = hp["kv"]
    ans.ka = hp["ka"]
    ans.kj = hp["kj"]
    ans.kt = hp["kt"]
    ans.ko = hp["ko"]
    ans.klat = hp["klat"]
    ans.klon = hp["klon"]
    ans.num_threads = hp["num_threads"]
    return ans


def run_fot(initial_conditions, hyperparameters):
    """ Return the frenet optimal trajectory given initial conditions in
    cartesian space.

    Args:
        initial_conditions (dict): dict containing the following items
            ps (float): previous longitudinal position
            target_speed (float): target speed [m/s]
            pos (np.ndarray([float, float])): initial position in global coord
            vel (np.ndarray([float, float])): initial velocity [m/s]
            wp (np.ndarray([float, float])): list of global waypoints
            obs (np.ndarray([float, float, float, float])): list of obstacles
                as: [lower left x, lower left y, upper right x, upper right y]

        hyperparameters (dict): a dict of optional hyperparameters
            max_speed (float): maximum speed [m/s]
            max_accel (float): maximum acceleration [m/s^2]
            max_curvature (float): maximum curvature [1/m]
            max_road_width_l (float): maximum road width to the left [m]
            max_road_width_r (float): maximum road width to the right [m]
            d_road_w (float): road width sampling discretization [m]
            dt (float): time sampling discretization [s]
            maxt (float): max prediction horizon [s]
            mint (float): min prediction horizon [s]
            d_t_s (float): target speed sampling discretization [m/s]
            n_s_sample (float): sampling number of target speed
            obstacle_clearance (float): obstacle radius [m]
            kd (float): positional deviation cost
            kv (float): velocity cost
            ka (float): acceleration cost
            kj (float): jerk cost
            kt (float): time cost
            ko (float): dist to obstacle cost
            klat (float): lateral cost
            klon (float): longitudinal cost

    Returns:
        result_x (np.ndarray(float)): x positions of fot, if it exists
        result_y (np.ndarray(float)): y positions of fot, if it exists
        speeds (np.ndarray(float)): speeds of fot, if it exists
        ix (np.ndarray(float)): spline x of fot, if it exists
        iy (np.ndarray(float)): spline y of fot, if it exists
        iyaw (np.ndarray(float)): spline yaws of fot, if it exists
        d (np.ndarray(float)): lateral offset of fot, if it exists
        s (np.ndarray(float)): longitudinal offset of fot, if it exists
        speeds_x (np.ndarray(float)): x speeds of fot, if it exists
        speeds_y (np.ndarray(float)): y speeds of fot, if it exists
        params (dict): next frenet coordinates, if they exist
        costs (dict): costs of best frenet path, if it exists
        success (bool): whether a fot was found or not
    """
    # parse initial conditions and convert to frenet coordinates
    fot_initial_conditions, misc = to_frenet_initial_conditions(
        initial_conditions)

    # parse hyper parameters
    fot_hp = _parse_hyperparameters(hyperparameters)

    # initialize return values
    fot_rv = FotWrapper.FrenetReturnValues()

    # run the planner
    _run_fot(fot_initial_conditions, fot_hp, fot_rv)

    path_length = fot_rv.path_length
    x_path = np.array(fot_rv.x_path)
    y_path = np.array(fot_rv.y_path)
    speeds = np.array(fot_rv.speeds)
    ix = np.array(fot_rv.ix)
    iy = np.array(fot_rv.iy)
    iyaw = np.array(fot_rv.iyaw)
    d = np.array(fot_rv.d)
    s = np.array(fot_rv.s)
    speeds_x = np.array(fot_rv.speeds_x)
    speeds_y = np.array(fot_rv.speeds_y)
    params = {
        "s": fot_rv.params[0],
        "s_d": fot_rv.params[1],
        "d": fot_rv.params[2],
        "d_d": fot_rv.params[3],
        "d_dd": fot_rv.params[4],
    }
    costs = {
        "c_lateral_deviation": fot_rv.costs[0],
        "c_lateral_velocity": fot_rv.costs[1],
        "c_lateral_acceleration": fot_rv.costs[2],
        "c_lateral_jerk": fot_rv.costs[3],
        "c_lateral": fot_rv.costs[4],
        "c_longitudinal_acceleration": fot_rv.costs[5],
        "c_longitudinal_jerk": fot_rv.costs[6],
        "c_time_taken": fot_rv.costs[7],
        "c_end_speed_deviation": fot_rv.costs[8],
        "c_longitudinal": fot_rv.costs[9],
        "c_inv_dist_to_obstacles": fot_rv.costs[10],
        "cf": fot_rv.costs[11],
    }

    success = fot_rv.success

    return x_path, y_path, speeds, ix, iy, iyaw, d, s, \
           speeds_x, speeds_y, params, costs, success


def to_frenet_initial_conditions(initial_conditions):
    """ Convert the cartesian initial conditions into frenet initial conditions.

    Args:
        initial_conditions (dict): dictionary containing
            ps (float): previous longitudinal position
            target_speed (float): target speed [m/s]
            pos (np.ndarray([float, float])): initial position in global coord
            vel (np.ndarray([float, float])): initial velocity [m/s]
            wp (np.ndarray([float, float])): list of global waypoints
            obs (np.ndarray([float, float, float, float])): list of obstacles
                as: [lower left x, lower left y, upper right x, upper right y]
    Returns:
        FrenetInitialConditions, dictionary for debugging
    """
    # parse the initial conditions
    ps = initial_conditions['ps']
    pos = initial_conditions['pos']
    vel = initial_conditions['vel']
    wp = initial_conditions['wp']
    obs = initial_conditions['obs']
    target_speed = initial_conditions['target_speed']
    if obs.shape[0] == 0:
        obs = np.empty((0, 4))
    x = pos[0].item()
    y = pos[1].item()
    vx = vel[0].item()
    vy = vel[1].item()
    wx = wp[:, 0]
    wy = wp[:, 1]
    o_llx = np.copy(obs[:, 0])
    o_lly = np.copy(obs[:, 1])
    o_urx = np.copy(obs[:, 2])
    o_ury = np.copy(obs[:, 3])
    forward_speed = np.hypot(vx, vy).item()

    # construct return array and convert initial conditions
    misc = np.zeros(5)
    _to_frenet_initial_conditions(ps, x, y,
                                  vx, vy,
                                  forward_speed,
                                  wx,
                                  wy,
                                  misc)

    # return the FrenetInitialConditions structure
    ans = FotWrapper.FrenetInitialConditions()
    ans.s0 = misc[0]
    ans.c_speed = misc[1]
    ans.c_d = misc[2]
    ans.c_d_d = misc[3]
    ans.c_d_dd = misc[4]
    ans.target_speed = target_speed
    ans.wx = wx
    ans.wy = wy
    ans.o_llx = o_llx
    ans.o_lly = o_lly
    ans.o_urx = o_urx
    ans.o_ury = o_ury
    return ans, misc


#############################################################
# For Anytime Planner Implementation
#############################################################
def query_anytime_planner_path(fot_planner, return_rv_object=False):
    # return value should be separate for each path
    fot_rv = FrenetReturnValues(0)
    fot_planner.get_path(fot_rv)

    x_path = np.array([fot_rv.x_path[i] for i in range(fot_rv.path_length)])
    y_path = np.array([fot_rv.y_path[i] for i in range(fot_rv.path_length)])
    speeds = np.array([fot_rv.speeds[i] for i in range(fot_rv.path_length)])
    ix = np.array([fot_rv.ix[i] for i in range(fot_rv.path_length)])
    iy = np.array([fot_rv.iy[i] for i in range(fot_rv.path_length)])
    iyaw = np.array([fot_rv.iyaw[i] for i in range(fot_rv.path_length)])
    d = np.array([fot_rv.d[i] for i in range(fot_rv.path_length)])
    s = np.array([fot_rv.s[i] for i in range(fot_rv.path_length)])
    speeds_x = np.array([fot_rv.speeds_x[i] for i in range(fot_rv.path_length)])
    speeds_y = np.array([fot_rv.speeds_y[i] for i in range(fot_rv.path_length)])
    params = {
        "s": fot_rv.params[0],
        "s_d": fot_rv.params[1],
        "d": fot_rv.params[2],
        "d_d": fot_rv.params[3],
        "d_dd": fot_rv.params[4],
    }
    costs = {
        "c_lateral_deviation": fot_rv.costs[0],
        "c_lateral_velocity": fot_rv.costs[1],
        "c_lateral_acceleration": fot_rv.costs[2],
        "c_lateral_jerk": fot_rv.costs[3],
        "c_lateral": fot_rv.costs[4],
        "c_longitudinal_acceleration": fot_rv.costs[5],
        "c_longitudinal_jerk": fot_rv.costs[6],
        "c_time_taken": fot_rv.costs[7],
        "c_end_speed_deviation": fot_rv.costs[8],
        "c_longitudinal": fot_rv.costs[9],
        "c_inv_dist_to_obstacles": fot_rv.costs[10],
        "cf": fot_rv.costs[11],
    }

    success = fot_rv.success
    if return_rv_object:
        return x_path, y_path, speeds, ix, iy, iyaw, d, s, \
            speeds_x, speeds_y, params, costs, success, fot_rv
    else:
        return x_path, y_path, speeds, ix, iy, iyaw, d, s, \
            speeds_x, speeds_y, params, costs, success
