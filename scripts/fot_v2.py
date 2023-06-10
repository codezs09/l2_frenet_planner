
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
from pathlib import Path
import argparse
import os 
import sys
import subprocess

from load_data import load_data

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD_DIR = os.path.join(PROJECT_DIR, "build")
CONFIG_DIR = os.path.join(PROJECT_DIR, "config")
CPP_EXECUTABLE_PATH = os.path.join(BUILD_DIR, "FrenetOptimalTrajectoryTest")
DATA_LOG_PATH = os.path.join(BUILD_DIR, "data.bin")

def parse_arguments(): 
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene_path', type=str, 
                        default=os.path.join(CONFIG_DIR, "scenes/one_lane_slow_down.json"))
    parser.add_argument('--hyper_path', type=str, 
                        default=os.path.join(CONFIG_DIR, "hyperparameters.json"))
    parser.add_argument('--store_data', action='store_true', 
                        default=True)
    parser.add_argument('--data_path', type=str, 
                        default=DATA_LOG_PATH)
    parser.add_argument('--save_frame', action='store_true',
                        default=True)

    args = parser.parse_args()
    return args 

def run_cpp_executable(args):
    command = [CPP_EXECUTABLE_PATH,
               "--scene_path={}".format(args.scene_path),
               "--hyper_path={}".format(args.hyper_path),
               "--store_data={}".format(str(args.store_data).lower()),
               "--data_path={}".format(args.data_path)]
    subprocess.run(command)

def post_process(args):
    # visualization
    if args.store_data:
        area = 10
        data_frames = load_data(args.data_path)
        # visualize data
        for i, frame in enumerate(data_frames):
            plt.cla()
            plt.plot(frame.wx, frame.wy)
            ax = plt.gca()

            for obstacle in frame.obstacles:
                corners = obstacle.predict_boxes[0].corners
                polygon = patch.Polygon(corners, linewidth=1, edgecolor='k', facecolor='none')
                ax.add_patch(polygon)
            plt.plot(frame.best_frenet_path.x, frame.best_frenet_path.y, "-or")
            ego_x = frame.ego_car.pose.x
            ego_y = frame.ego_car.pose.y
            ego_yaw = frame.ego_car.pose.yaw
            plt.plot(ego_x, ego_y, "vc")
            plt.xlim(ego_x - area, ego_x + area)
            plt.ylim(ego_y - area, ego_y + area)
            plt.xlabel("X [m]")
            plt.ylabel("Y [m]")
            plt.title(f"Timestamp {frame.timestamp}, v[m/s]:" + \
                      str(np.linalg.norm((frame.ego_car.twist.vx)))[:4])
            plt.grid(True)
            if args.save_frame:
                Path("img/frames").mkdir(parents=True, exist_ok=True)
                plt.savefig("img/frames/{}.jpg".format(i))
            plt.pause(0.1)

if __name__=="__main__":
    args = parse_arguments()
    run_cpp_executable(args)
    post_process(args)