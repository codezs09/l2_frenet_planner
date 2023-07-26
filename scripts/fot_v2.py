
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
from pathlib import Path
import argparse
import os 
import sys
import subprocess

from load_data import load_data
from utils.geometry import rotate, pose_to_box
from make_gif import make_gif

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD_DIR = os.path.join(PROJECT_DIR, "build")
CONFIG_DIR = os.path.join(PROJECT_DIR, "config")
CPP_EXECUTABLE_PATH = os.path.join(BUILD_DIR, "FrenetOptimalTrajectoryTest")
DATA_LOG_PATH = os.path.join(BUILD_DIR, "data.bin")

def round_to_tenth(x):
    return round(x * 10.0) / 10.0

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
    parser.add_argument('--save_gif', action='store_true',
                        default=True)
    # debug fields
    parser.add_argument('--skip_fot', action='store_true', default=False,
                        help='skip running FOT and only do post-processing on data.bin')
    parser.add_argument('--cost_frame', type=int, default=None,
                        help='print path costs of a specific frame if provided')
    parser.add_argument('--cost_lane', type=int, default=None,
                        help='print candidate path costs of a specific lane if provided')
    parser.add_argument('--local_planning', action='store_true', default=True,
                        help='enable local planning')
    
    args = parser.parse_args()
    return args 

def run_cpp_executable(args):
    command = [CPP_EXECUTABLE_PATH,
               "--scene_path={}".format(args.scene_path),
               "--hyper_path={}".format(args.hyper_path),
               "--store_data={}".format(str(args.store_data).lower()),
               "--data_path={}".format(args.data_path),
               "--local_planning={}".format(str(args.local_planning).lower())]
    command_str = " ".join(command)
    print("Running command: {}".format(command_str))
    try:
        # Use subprocess.run with check=True to raise an exception if the command fails
        # Use subprocess.PIPE to capture the output
        # result = subprocess.run(command, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        result = subprocess.run(command_str, shell=True, check=True)
        # print(result.stdout.decode())  # Print the command's output
    except subprocess.CalledProcessError as e:
        print(f"Command failed with error {e.returncode}")
        # print(e.output.decode())
        sys.exit(e.returncode)  # Terminate the program

def plot_frames(data_frames, args):
    if args.save_frame or args.save_gif:
        Path("img/frames").mkdir(parents=True, exist_ok=True)
        # remove all files in img/frames
        frames_dir = os.path.join(PROJECT_DIR, "img/frames")
        for file in os.listdir(frames_dir):
            os.remove(os.path.join(frames_dir, file))
        
        Path("img/frames_local").mkdir(parents=True, exist_ok=True)
        # remove all files in img/frames_local
        frames_dir = os.path.join(PROJECT_DIR, "img/frames_local")
        for file in os.listdir(frames_dir):
            os.remove(os.path.join(frames_dir, file))

    # visualization
    area = 20
    # visualize data
    for i, frame in enumerate(data_frames):
        plt.figure(1)
        plt.cla()
        # plot lanes
        for lane in frame.lanes:
            wp = lane.wp
            left_boundary = lane.left_boundary
            right_boundary = lane.right_boundary
            plt.plot(left_boundary[0], left_boundary[1], "-k")
            plt.plot(right_boundary[0], right_boundary[1], "-k")
        ax = plt.gca()

        # plot obstacles
        for obstacle in frame.obstacles:
            corners = obstacle.predict_boxes[0].corners
            polygon = patch.Polygon(corners, linewidth=1, edgecolor='b', facecolor='none')
            ax.add_patch(polygon)
            for j in range(1, len(obstacle.predict_boxes), 10):
                corners = obstacle.predict_boxes[j].corners
                polygon = patch.Polygon(corners, linewidth=1, edgecolor='b', facecolor='none',
                                        linestyle='--')
                ax.add_patch(polygon)

        # plot ego car
        ego_x = frame.ego_car.pose.x
        ego_y = frame.ego_car.pose.y
        ego_corners = pose_to_box(frame.ego_car.pose, frame.ego_car.length, frame.ego_car.width)
        ego_polygon = patch.Polygon(ego_corners, linewidth=1, edgecolor='g', facecolor='none')
        ax.add_patch(ego_polygon)
        plt.plot(ego_x, ego_y, "vc")

        # plot frenet paths
        plt.plot(frame.best_frenet_path.x, frame.best_frenet_path.y, "-or")
        # print("frame.frenet_paths number: ", len(frame.frenet_paths))
        for frenet_path in frame.frenet_paths:
            plt.plot(frenet_path.x, frenet_path.y, "-")

        plt.axis('equal')
        plt.xlim(ego_x - 0.5*area, ego_x + 2.5*area)
        plt.ylim(ego_y - 0.5*area, ego_y + 2.5*area)
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.title(f"Global: Timestamp {frame.timestamp: .1f}, v[m/s]:" + \
                    str(np.linalg.norm((frame.ego_car.twist.vx)))[:4])
        plt.grid(True)
        if args.save_frame or args.save_gif:
            plt.savefig("img/frames/{}.jpg".format(i))

        # plot objects in local coordinate frame
        plt.figure(2)
        plt.cla()
        # plot lanes
        for key,value in frame.wp_lanes_local.items():
            wp = value 
            plt.plot(wp[0], wp[1], "--k")
        ax = plt.gca()

        # plot obstacles
        for obstacle in frame.obstacles_local:
            corners = obstacle.predict_boxes[0].corners
            polygon = patch.Polygon(corners, linewidth=1, edgecolor='b', facecolor='none')
            ax.add_patch(polygon)
            for j in range(1, len(obstacle.predict_boxes), 10):
                corners = obstacle.predict_boxes[j].corners
                polygon = patch.Polygon(corners, linewidth=1, edgecolor='b', facecolor='none',
                                        linestyle='--')
                ax.add_patch(polygon)

        # plot ego car
        ego_x = frame.planning_init_point_local.pose.x
        ego_y = frame.planning_init_point_local.pose.y
        ego_corners = pose_to_box(frame.planning_init_point_local.pose, 
                                    frame.planning_init_point_local.length, 
                                    frame.planning_init_point_local.width)
        ego_polygon = patch.Polygon(ego_corners, linewidth=1, edgecolor='g', facecolor='none')
        ax.add_patch(ego_polygon)
        plt.plot(ego_x, ego_y, "vc")

        # plot frenet paths
        plt.plot(frame.best_frenet_path_local.x, frame.best_frenet_path_local.y, "-or")
        # print("frame.frenet_paths number: ", len(frame.frenet_paths))
        for frenet_path_local in frame.frenet_paths_local:
            plt.plot(frenet_path_local.x, frenet_path_local.y, "-")
        # # plot candiate path at d offset as well
        # for fps_per_lane in frame.frenet_paths_local_all:
        #     for fp in fps_per_lane:
        #         plt.plot(fp.x, fp.y, "k--")

        plt.axis('equal')
        plt.xlim(ego_x - 0.5*area, ego_x + 2.5*area)
        plt.ylim(ego_y - 0.5*area, ego_y + 2.5*area)
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.title(f"Local: Timestamp {frame.timestamp: .1f}, v[m/s]:" + \
                    str(np.linalg.norm((frame.ego_car.twist.vx)))[:4])
        plt.grid(True)
        if args.save_frame or args.save_gif:
            plt.savefig("img/frames_local/{}.jpg".format(i))

        plt.pause(0.1)

def print_frame_cost(data_frames, frame_idx, lane_idx = None):
    '''
    Compare costs by lane if lane_idx not provided. 
    Compare costs by paths per d offset if lane_idx provided.
    '''
    frame = data_frames[frame_idx]

    costs_data = {
        'lane_ids': [],
        'd_offsets': [],
        'costs_total': [],
        'costs_lateral': [],
        'costs_longitudinal': [],
        'costs_inv_dist_to_obstacles': [],
        'costs_lane_change': []
    }

    if lane_idx is None:
        for frenet_path in frame.frenet_paths:
            plt.plot(frenet_path.x, frenet_path.y, "-")
            costs_data['lane_ids'].append(frenet_path.lane_id)
            costs_data['costs_total'].append(frenet_path.cf)
            costs_data['costs_lateral'].append(frenet_path.c_lateral)
            costs_data['costs_longitudinal'].append(frenet_path.c_longitudinal)
            costs_data['costs_inv_dist_to_obstacles'].append(frenet_path.c_inv_dist_to_obstacles)
            costs_data['costs_lane_change'].append(frenet_path.c_lane_change)
    else:
        for fp in frame.frenet_paths_local_all[lane_idx]:
            costs_data['lane_ids'].append(fp.lane_id)
            costs_data['d_offsets'].append(round_to_tenth(fp.d[-1]))
            costs_data['costs_total'].append(fp.cf)
            costs_data['costs_lateral'].append(fp.c_lateral)
            costs_data['costs_longitudinal'].append(fp.c_longitudinal)
            costs_data['costs_inv_dist_to_obstacles'].append(fp.c_inv_dist_to_obstacles)
            costs_data['costs_lane_change'].append(fp.c_lane_change)

    # print costs
    def row_str(key):
        # return " ".join(["{:.2f}".format(x) for x in row])
        return key + "\t" + "\t".join([f"{x:.1f}" for x in costs_data[key]]) + "\n"
    print("\nCosts at frame {}".format(frame_idx))

    print(row_str('lane_ids') + \
        row_str('d_offsets') + \
        row_str('costs_total') + \
        row_str('costs_lateral') + \
        row_str('costs_longitudinal') + \
        row_str('costs_inv_dist_to_obstacles') + \
        row_str('costs_lane_change'))

def post_process(args):
    if args.store_data:
        data_frames = load_data(args.data_path)

        plot_frames(data_frames, args)

        if args.cost_frame is not None:
            print_frame_cost(data_frames, args.cost_frame, args.cost_lane)
        

if __name__=="__main__":
    args = parse_arguments()
    
    if not args.skip_fot:
        run_cpp_executable(args)
    else:
        print("Skipping FOT and only doing post-processing on data.bin")

    post_process(args)
    if args.save_gif:
        make_gif()