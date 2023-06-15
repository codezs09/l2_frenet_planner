import fot_wrapper
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import argparse
from pathlib import Path
import os
import json


# Run fot planner
def fot(show_animation=False,
        show_info=False,
        num_threads=0,
        save_frame=False,
        scene_path=""):
    if not scene_path or os.path.exists(scene_path): 
        raise ValueError("Scene path not provided")
    with open(scene_path, 'r') as f:
        conds = json.load(f)

    initial_conditions = {
        'timestamp': 0.0,   # simulation time stamp
        'ps': conds['s0'],
        'target_speed': conds['target_speed'],
        'pos': np.array(conds['pos']),
        'vel': np.array(conds['vel']),
        'wp': np.array(conds['wp']),
        'obs': np.array(conds['obs'])
    }

    hyperparameters = {
        "max_speed": 25.0,
        "max_accel": 15.0,
        "max_curvature": 15.0,
        "max_road_width_l": 5.0,
        "max_road_width_r": 5.0,
        "d_road_w": 0.5,
        "dt": 0.1,
        "maxt": 5.0,
        "mint": 2.0,
        "d_t_s": 0.5,
        "n_s_sample": 2.0,
        "obstacle_clearance": 0.1,
        "kd": 1.0,
        "kv": 0.1,
        "ka": 0.1,
        "kj": 0.1,
        "kt": 0.1,
        "ko": 0.1,
        "klat": 1.0,
        "klon": 1.0,
        "num_threads": num_threads,  # set 0 to avoid using threaded algorithm
    }

    # static elements of planner
    wx = initial_conditions['wp'][:, 0]
    wy = initial_conditions['wp'][:, 1]

    # simulation config
    sim_loop = 200
    area = 40
    total_runtime = 0
    runtime_list = []
    for i in range(sim_loop):
        # run FOT and keep time
        print("Iteration: {}".format(i))
        start_time = time.time()
        result_x, result_y, speeds, ix, iy, iyaw, d, s, speeds_x, \
            speeds_y, misc, costs, success = \
            fot_wrapper.run_fot(initial_conditions, hyperparameters)
        end_time = time.time() - start_time
        print("Time taken: {}".format(end_time))
        total_runtime += end_time
        runtime_list.append(end_time)

        # reconstruct initial_conditions
        if success:
            initial_conditions['timestamp'] += hyperparameters['dt']
            initial_conditions['pos'] = np.array([result_x[1], result_y[1]])
            initial_conditions['vel'] = np.array([speeds_x[1], speeds_y[1]])
            initial_conditions['ps'] = misc['s']
            if initial_conditions['obs']['pose_coord'] == 'Cartesian':
                initial_conditions['obs']['pose']
            if show_info:
                print(costs)
        else:
            print(f"Failed unexpectedly at iteration {i}, simulation time {initial_conditions['timestamp']}.")
            break

        # break if near goal
        if np.hypot(result_x[1] - wx[-1], result_y[1] - wy[-1]) <= 3.0:
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None])
            plt.plot(wx, wy)
            ax = plt.gca()
            for o in obs:
                rect = patch.Rectangle((o[0], o[1]), o[2] - o[0], o[3] - o[1])
                ax.add_patch(rect)
            plt.plot(result_x[1:], result_y[1:], "-or")
            plt.plot(result_x[1], result_y[1], "vc")
            plt.xlim(result_x[1] - area, result_x[1] + area)
            plt.ylim(result_y[1] - area, result_y[1] + area)
            plt.xlabel("X axis")
            plt.ylabel("Y axis")
            plt.title(f"Timestamp {initial_conditions['timestamp']:.1f}, v[m/s]:" +
                      str(np.linalg.norm(initial_conditions['vel']))[0:4])
            plt.grid(True)
            if save_frame:
                Path("img/frames").mkdir(parents=True, exist_ok=True)
                plt.savefig("img/frames/{}.jpg".format(i))
            plt.pause(0.1)

    print("Finish")

    print("======================= SUMMARY ========================")
    print("Total time for {} iterations taken: {}".format(i, total_runtime))
    print("Average time per iteration: {}".format(total_runtime / i))
    print("Max time per iteration: {}".format(max(runtime_list)))

    return runtime_list


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--display",
        action="store_true",
        help="show animation, ensure you have X11 forwarding server open")
    parser.add_argument("-v",
                        "--verbose",
                        action="store_true",
                        help="verbose mode, show all state info")
    parser.add_argument("-s",
                        "--save",
                        action="store_true",
                        help="save each frame of simulation")
    parser.add_argument("-t",
                        "--thread",
                        type=int,
                        default=0,
                        help="set number of threads to run with")
    parser.add_argument("-p",
                        "--scene_path",
                        type=str,
                        default="/home/sheng/Projects/l2_frenet_planner/config/scene/"
                                "one_lane_slow_down.json",
                        help="set path to load scene json file")
    args = parser.parse_args()

    # run planner with args passed in
    fot(args.display, args.verbose, args.thread, args.save, args.scene_path)
