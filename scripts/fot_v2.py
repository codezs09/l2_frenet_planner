
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
from pathlib import Path
import argparse
import os 
import sys
import subprocess
import json
from collections import defaultdict
import copy

from load_data import load_data
from utils.geometry import rotate, pose_to_box
from make_gif import make_gif

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD_DIR = os.path.join(PROJECT_DIR, "build")
CONFIG_DIR = os.path.join(PROJECT_DIR, "config")
CPP_EXECUTABLE_PATH = os.path.join(BUILD_DIR, "FrenetOptimalTrajectoryTest")
DATA_LOG_PATH = os.path.join(BUILD_DIR, "data.bin")

LONMODE = ["", "Following", "VelocityKeeping"]

def round_to_tenth(x):
    return round(x * 10.0) / 10.0

def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def parse_arguments(): 
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene_path', type=str, 
                        default=os.path.join(CONFIG_DIR, "scenes/two_lanes.json"))
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
                                        linestyle='--', alpha=0.5)
                ax.add_patch(polygon)

        # plot ego car
        ego_x = frame.ego_car.pose.x
        ego_y = frame.ego_car.pose.y
        ego_yaw = frame.ego_car.pose.yaw
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
        plt.ylim(ego_y - 1.5*area, ego_y + 1.5*area)
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.title(f"Global: Timestamp {frame.timestamp: .1f}, v[m/s]:" + \
                    f"{frame.ego_car.twist.vx:.2f}" + \
                    " " + LONMODE[int(frame.best_frenet_path.lon_mode)])
        plt.grid(True)
        if args.save_frame or args.save_gif:
            plt.savefig("img/frames/{}.jpg".format(i))

        # plot objects in local coordinate frame
        plt.figure(2)
        plt.cla()
        # plot lanes
        # for key,value in frame.wp_lanes_local.items():
        #     wp = value 
        #     plt.plot(wp[0], wp[1], "--k")
        for lane in frame.lanes:
            left_boundary_local = copy.deepcopy(lane.left_boundary)
            right_boundary_local = copy.deepcopy(lane.right_boundary)
            # convert w.r.t. ego car
            for j in range(len(left_boundary_local[0])):
                pt = [left_boundary_local[0][j] - ego_x, left_boundary_local[1][j] - ego_y]
                pt_local = rotate(pt, -ego_yaw)
                left_boundary_local[0][j] = pt_local[0]
                left_boundary_local[1][j] = pt_local[1]
            for j in range(len(right_boundary_local[0])):
                pt = [right_boundary_local[0][j] - ego_x, right_boundary_local[1][j] - ego_y]
                pt_local = rotate(pt, -ego_yaw)
                right_boundary_local[0][j] = pt_local[0]
                right_boundary_local[1][j] = pt_local[1]
            plt.plot(left_boundary_local[0], left_boundary_local[1], "-k")
            plt.plot(right_boundary_local[0], right_boundary_local[1], "-k")
        ax = plt.gca()

        # plot obstacles
        for obstacle in frame.obstacles_local:
            corners = obstacle.predict_boxes[0].corners
            polygon = patch.Polygon(corners, linewidth=1, edgecolor='b', facecolor='none')
            ax.add_patch(polygon)
            for j in range(1, len(obstacle.predict_boxes), 10):
                corners = obstacle.predict_boxes[j].corners
                polygon = patch.Polygon(corners, linewidth=1, edgecolor='b', facecolor='none',
                                        linestyle='--', alpha=0.5)
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
        # plot candiate path at d offset as well
        for fps_per_lane in frame.frenet_paths_local_all:
            for fp in fps_per_lane:
                plt.plot(fp.x, fp.y, "b--")

        plt.axis('equal')
        plt.xlim(ego_x - 0.5*area, ego_x + 2.5*area)
        plt.ylim(ego_y - 0.5*area, ego_y + 1.0*area)
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.title(f"Local: Timestamp {frame.timestamp: .1f}, v[m/s]:" + \
                  f"{frame.ego_car.twist.vx:.2f}" + \
                    " " + LONMODE[int(frame.best_frenet_path.lon_mode)])
        plt.grid(True)
        if args.save_frame or args.save_gif:
            plt.savefig("img/frames_local/{}.jpg".format(i))

        # plt.pause(0.1)

def print_frame_cost(data_frames, frame_idx, lane_idx = None):
    '''
    Compare costs by lane if lane_idx not provided. 
    Compare costs by paths per d offset if lane_idx provided.
    '''
    frame = data_frames[frame_idx]

    costs_data = defaultdict(list)

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
        plt.figure()
        plt.suptitle('frenet sd plot @ frame=' + str(frame_idx) + ' lane=' + str(lane_idx))

        # load json file args.hyper_path
        with open(args.hyper_path) as f:
            hp = json.load(f)

        for fp in frame.frenet_paths_local_all[lane_idx]:
            costs_data['lane_ids'].append(fp.lane_id)
            costs_data['d_offsets'].append(round_to_tenth(fp.d[-1]))
            costs_data['costs_total'].append(fp.cf)
            costs_data['costs_lateral'].append(fp.c_lateral)
            costs_data['costs_longitudinal'].append(fp.c_longitudinal)
            costs_data['costs_inv_dist_to_obstacles'].append(fp.c_inv_dist_to_obstacles)
            costs_data['costs_lane_change'].append(fp.c_lane_change)

            # lon costs
            costs_data["costs_lon_a"].append(hp['ka'] * fp.c_longitudinal_acceleration)
            costs_data["costs_lon_j"].append(hp['kj'] * fp.c_longitudinal_jerk)
            costs_data["costs_lon_t"].append(hp['kt'] * fp.c_time_taken)
            costs_data["costs_end_v"].append(hp['k_es'] * fp.c_end_speed_deviation)
            costs_data["costs_end_s"].append(hp['k_es'] * fp.c_end_s_deviation)
            costs_data["costs_eff"].append(hp['k_ef'] * fp.c_efficiency)

            # lat costs
            costs_data["costs_lat_dev"].append(hp['kd'] * fp.c_lateral_deviation)
            costs_data["costs_end_d"].append(hp['k_ed'] * fp.c_end_d_deviation)
            costs_data["costs_lat_v"].append(hp['kv'] * fp.c_lateral_velocity)
            costs_data["costs_lat_a"].append(hp['ka'] * fp.c_lateral_acceleration)
            costs_data["costs_lat_j"].append(hp['kj'] * fp.c_lateral_jerk)
    
            ax1 = plt.subplot(331)
            plt.plot(fp.t, fp.s)
            plt.xlabel('t')
            plt.ylabel('s')
            plt.grid(True)

            plt.subplot(334, sharex=ax1)
            plt.plot(fp.t, fp.s_d)
            plt.xlabel('t')
            plt.ylabel('s_d')
            plt.grid(True)

            plt.subplot(337, sharex=ax1)
            plt.plot(fp.t, fp.s_dd)
            plt.xlabel('t')
            plt.ylabel('s_dd')
            plt.grid(True)
            
            plt.subplot(332, sharex=ax1)
            plt.plot(fp.t, fp.d)
            plt.xlabel('t')
            plt.ylabel('d')
            plt.grid(True)

            plt.subplot(335, sharex=ax1)
            plt.plot(fp.t, fp.d_d)
            plt.xlabel('t')
            plt.ylabel('d_d')
            plt.grid(True)

            plt.subplot(338, sharex=ax1)
            plt.plot(fp.t, fp.d_dd)
            plt.xlabel('t')
            plt.ylabel('d_dd')
            plt.grid(True)

            plt.subplot(333)
            plt.plot(fp.s, fp.d)
            plt.xlabel('s')
            plt.ylabel('d')
            plt.grid(True)

            # this is wrong for short paths
            yaw = wrap_angle(np.arctan2(fp.d_d, fp.s_d))
            plt.subplot(336)
            plt.plot(fp.t, yaw)
            plt.xlabel('t')
            plt.ylabel('yaw')
            plt.grid(True)

        bf = frame.best_frenet_path
        ax1 = plt.subplot(331)
        plt.plot(bf.t, bf.s, 'b--')
        plt.subplot(334, sharex=ax1)
        plt.plot(bf.t, bf.s_d, 'b--')
        plt.subplot(337, sharex=ax1)
        plt.plot(bf.t, bf.s_dd, 'b--')
        plt.subplot(332, sharex=ax1)
        plt.plot(bf.t, bf.d, 'b--')
        plt.subplot(335, sharex=ax1)
        plt.plot(bf.t, bf.d_d, 'b--')
        plt.subplot(338, sharex=ax1)
        plt.plot(bf.t, bf.d_dd, 'b--')
        plt.subplot(333)
        plt.plot(bf.s, bf.d, 'b--')
        plt.subplot(336)
        plt.plot(bf.t, wrap_angle(np.arctan2(bf.d_d, bf.s_d)), 'b--')


    # print costs
    def row_str(key):
        # return " ".join(["{:.2f}".format(x) for x in row])
        return key + "\t" + "\t".join([f"{x:.1f}" for x in costs_data[key]]) + "\n"

    def print_cost_in_seq(cost_seq):
        s = ""
        for cost_name in cost_seq:
            if cost_name in costs_data:
                s += row_str(cost_name)
            else:
                s += cost_name + "\t" + "\t".join(["-"] * len(costs_data['lane_ids'])) + "\n"
        print(s)


    total_cost_seq = [
        'lane_ids', 
        'd_offsets',
        'costs_total',
        'costs_lateral',
        'costs_longitudinal',
        'costs_inv_dist_to_obstacles',
        'costs_lane_change'
    ]
    print("\nCosts at frame {}".format(frame_idx))
    print_cost_in_seq(total_cost_seq)

    lon_cost_seq = [
        "costs_lon_a", 
        "costs_lon_j",
        "costs_lon_t",
        "costs_end_v",
        "costs_end_s",
        "costs_eff"
    ]
    print("\nLongitudinal costs at frame {}".format(frame_idx))
    print_cost_in_seq(lon_cost_seq)

    lat_cost_seq = [
        "costs_lat_dev",
        "costs_end_d",
        "costs_lat_v",
        "costs_lat_a",
        "costs_lat_j"
    ]
    print_cost_in_seq(lat_cost_seq)


def plot_states(data_frames):
    timestamp = [df.timestamp for df in data_frames]
    speed_meas = [df.speed_meas for df in data_frames]
    yaw_rate_meas = [df.yaw_rate_meas for df in data_frames]
    pose_change_est_x = [df.pose_change_est.x for df in data_frames]
    pose_change_est_y = [df.pose_change_est.y for df in data_frames]
    pose_change_est_yaw = [df.pose_change_est.yaw for df in data_frames]
    planning_init_point_wrt_last_frame_x = [df.planning_init_point_wrt_last_frame.x for df in data_frames]
    planning_init_point_wrt_last_frame_y = [df.planning_init_point_wrt_last_frame.y for df in data_frames]
    planning_init_point_wrt_last_frame_yaw = [df.planning_init_point_wrt_last_frame.yaw for df in data_frames]
    planning_init_point_local_x = [df.planning_init_point_local.pose.x for df in data_frames]
    planning_init_point_local_y = [df.planning_init_point_local.pose.y for df in data_frames]
    planning_init_point_local_yaw = [df.planning_init_point_local.pose.yaw for df in data_frames]
    
    pose_change_act_x = [0]
    pose_change_act_y = [0]
    pose_change_act_yaw = [0]
    for i in range(len(data_frames) - 1):
        pose_change_act_x.append(data_frames[i+1].ego_car.pose.x - data_frames[i].ego_car.pose.x)
        pose_change_act_y.append(data_frames[i+1].ego_car.pose.y - data_frames[i].ego_car.pose.y)
        pose_change_act_yaw.append(data_frames[i+1].ego_car.pose.yaw - data_frames[i].ego_car.pose.yaw)

    speed_act = [df.ego_car.twist.vx for df in data_frames]
    yaw_rate_act = [df.ego_car.twist.yaw_rate for df in data_frames]


    # sensor measurement vs actual states
    plt.figure(figsize=(12,4.5))
    plt.suptitle("Sensor measurement vs actual states")
    ax1 = plt.subplot(221)
    plt.plot(timestamp[1:], speed_meas[1:], 'b-x', label='Measure',ms=4)
    plt.plot(timestamp, speed_act, 'r', label='Actual')
    plt.xlabel('Time [s]')
    plt.ylabel(r'$v$ [m/s]')
    plt.legend()
    plt.grid(True)
    plt.subplots_adjust(hspace=0.3)
    
    plt.subplot(223, sharex=ax1)
    plt.plot(timestamp[1:], np.rad2deg(yaw_rate_meas[1:]), 'b-x', label='Measure', ms=4)
    plt.plot(timestamp, np.rad2deg(yaw_rate_act), 'r', label='Actual')
    plt.xlabel('Time [s]')
    plt.ylabel(r'$\dot{\theta}$ [deg/s]')
    plt.legend()
    plt.grid(True)
    plt.subplots_adjust(hspace=0.3)

    plt.subplot(222, sharex=ax1)
    plt.plot(timestamp[1:], np.array(speed_meas[1:]) - np.array(speed_act[1:]), 'b-x',ms=4)
    plt.xlabel('Time [s]')
    plt.ylabel(r'Meas. err   $v$ [m/s]')
    plt.grid(True)
    plt.subplots_adjust(hspace=0.3)

    plt.subplot(224, sharex=ax1)
    plt.plot(timestamp, np.rad2deg(np.array(yaw_rate_meas) - np.array(yaw_rate_act)), 'b-x',ms=4)
    plt.xlabel('Time [s]')
    plt.ylabel(r'Meas. err   $\dot{\theta}$ [deg/s]')
    plt.grid(True)
    plt.subplots_adjust(hspace=0.3)

    if args.save_frame or args.save_gif:
        plt.savefig("img/plots/sensor_err.jpg")



    # correlation plots of estimated pose change between frames
    plt.figure(figsize=(16, 4))
    plt.suptitle("Correlation plots of pose change estimate")
    ax1 = plt.subplot(131)
    plt.plot(planning_init_point_wrt_last_frame_x, pose_change_est_x, 'r.',alpha=0.3,ms=8)
    plt.xlabel(r'$\Delta x$ [m]')
    plt.ylabel(r'Est. $\Delta x$ [m]')
    plt.grid(True)
    plt.axis('equal')
    # ax1.set_aspect('equal', 'box')
    plt.subplots_adjust(wspace=0.35)

    ax2 = plt.subplot(132)
    plt.plot(planning_init_point_wrt_last_frame_y, pose_change_est_y, 'r.',alpha=0.3,ms=8)
    plt.xlabel(r'$\Delta y$ [m]')
    plt.ylabel(r'Est. $\Delta y$ [m]')
    plt.grid(True)
    plt.axis('equal')
    # ax2.set_aspect('equal', 'box')
    plt.subplots_adjust(wspace=0.35)

    ax3 = plt.subplot(133)
    plt.plot(np.rad2deg(planning_init_point_wrt_last_frame_yaw), np.rad2deg(pose_change_est_yaw), 'r.',alpha=0.3,ms=8)
    plt.xlabel(r'$\Delta\theta$ [deg]')
    plt.ylabel(r'Est. $\Delta\theta$ [deg]')
    plt.grid(True)
    plt.axis('equal')
    # ax3.set_aspect('equal', 'box')
    plt.subplots_adjust(wspace=0.35)

    if args.save_frame or args.save_gif:
        plt.savefig("img/plots/correlate_plot.jpg")


    # box plot of sensor reading errors plus estimated pose change errors
    plt.figure(figsize=(8, 6))
    plt.suptitle("Box plot of sensor reading errors plus estimated pose change errors")
    ax1 = plt.subplot(231)
    y = np.array(speed_meas[1:]) - np.array(speed_act[1:])
    plt.boxplot(y,medianprops=dict(color='blue',linewidth=1.5))
    x = np.random.normal(1, 0.04, size=len(y))    
    plt.plot(x, y, 'r.', alpha=0.2)
    plt.ylabel(r'Meas. err  $v$ [m/s]')
    plt.subplots_adjust(wspace=0.8)
    plt.xticks([])

    plt.subplot(232)
    y = np.rad2deg(np.array(yaw_rate_meas) - np.array(yaw_rate_act))
    plt.boxplot(y,medianprops=dict(color='blue',linewidth=1.5))
    x = np.random.normal(1, 0.04, size=len(y))    
    plt.plot(x, y, 'r.', alpha=0.2)
    plt.ylabel(r'Meas. err   $\dot{\theta}$ [deg/s]')
    plt.subplots_adjust(wspace=0.8)
    plt.xticks([])
    
    plt.subplot(234)
    y = planning_init_point_local_x[1:]
    plt.boxplot(y,medianprops=dict(color='blue',linewidth=1.5))    
    x = np.random.normal(1, 0.04, size=len(y))    
    plt.plot(x, y, 'r.', alpha=0.2)
    plt.ylabel(r'Est. err  $\Delta x$ [m]')
    plt.subplots_adjust(wspace=0.8)
    plt.xticks([])

    plt.subplot(235)
    y= planning_init_point_local_y
    plt.boxplot(y,medianprops=dict(color='blue',linewidth=1.5))
    x = np.random.normal(1, 0.04, size=len(y))    
    plt.plot(x, y, 'r.', alpha=0.2)
    plt.ylabel(r'Est. err   $\Delta y$ [m]')
    plt.subplots_adjust(wspace=0.8)
    plt.xticks([])

    plt.subplot(236)
    y = np.rad2deg(planning_init_point_local_yaw)
    plt.boxplot(y,medianprops=dict(color='blue',linewidth=1.5))
    x = np.random.normal(1, 0.04, size=len(y))    
    plt.plot(x, y, 'r.', alpha=0.2)
    plt.ylabel(r'Est. err   $\Delta\theta$ [deg]')
    plt.subplots_adjust(wspace=0.8)
    plt.xticks([])

    if args.save_frame or args.save_gif:
        plt.savefig("img/plots/err_box_plot.jpg")
    

    # pose change estimation vs changes of actual states
    plt.figure(figsize=(12,6.5))
    plt.suptitle("Pose change estimation vs changes of actual states\nunder local coordinate system")
    ax1 = plt.subplot(321)
    plt.plot(timestamp, pose_change_est_x, 'b-x',label='Estimation',ms=4)
    plt.plot(timestamp, planning_init_point_wrt_last_frame_x, 'r', label='Actual')
    # plt.plot(timestamp, pose_change_act_x, label='actual')  # 不是一个坐标系下的， 不能对比
    plt.xlabel('Time [s]')
    plt.ylabel(r'$\Delta x$ [m]')
    plt.legend()
    plt.grid(True)
    plt.subplots_adjust(hspace=0.3)

    plt.subplot(323, sharex=ax1)
    plt.plot(timestamp, pose_change_est_y, 'b-x',label='Estimation',ms=4)
    plt.plot(timestamp, planning_init_point_wrt_last_frame_y, 'r', label='Actual')
    # plt.plot(timestamp, pose_change_act_y, label='actual')
    plt.xlabel('Time [s]')
    plt.ylabel(r'$\Delta y$ [m]')
    plt.legend()
    plt.grid(True)
    plt.subplots_adjust(hspace=0.3)

    plt.subplot(325, sharex=ax1)
    plt.plot(timestamp, np.rad2deg(pose_change_est_yaw), 'b-x',label='Estimation',ms=4)
    plt.plot(timestamp, np.rad2deg(planning_init_point_wrt_last_frame_yaw), 'r', label='Actual')
    # plt.plot(timestamp, np.rad2deg(pose_change_act_yaw), label='actual')
    plt.xlabel('Time [s]')
    plt.ylabel(r'$\Delta \theta$ [deg]')
    plt.legend()
    plt.grid(True)
    plt.subplots_adjust(hspace=0.3)

    plt.subplot(322, sharex=ax1)
    plt.plot(timestamp, planning_init_point_local_x, 'b-x',ms=4)
    # plt.plot(timestamp, np.array(pose_change_est_x) - np.array(pose_change_act_x), label='error') # 不是一个坐标系下的， 不能对比
    plt.xlabel('Time [s]')
    plt.ylabel(r'Est. err   $\Delta x$ [m]')
    plt.grid(True)
    plt.subplots_adjust(hspace=0.3)

    plt.subplot(324, sharex=ax1)
    plt.plot(timestamp, planning_init_point_local_y, 'b-x',ms=4)
    # plt.plot(timestamp, np.array(pose_change_est_y) - np.array(pose_change_act_y), label='error') # 不是一个坐标系下的， 不能对比
    plt.xlabel('Time [s]')
    plt.ylabel(r'Est. err   $\Delta y$ [m]')
    plt.grid(True)
    plt.subplots_adjust(hspace=0.3)

    plt.subplot(326, sharex=ax1)
    plt.plot(timestamp, np.rad2deg(planning_init_point_local_yaw), 'b-x',ms=4)
    # plt.plot(timestamp, np.rad2deg(np.array(pose_change_est_yaw) - np.array(pose_change_act_yaw)), label='error')# 不是一个坐标系下的， 不能对比
    plt.xlabel('Time [s]')
    plt.ylabel(r'Est. err   $\Delta\theta$ [deg]')
    plt.grid(True)
    plt.subplots_adjust(hspace=0.3)

    if args.save_frame or args.save_gif:
        plt.savefig("img/plots/pose_change.jpg")
    

    # ego car state
    plt.figure()
    plt.suptitle("Ego car state")
    ax1 = plt.subplot(331)
    plt.plot(timestamp, [df.ego_car.pose.x for df in data_frames])
    plt.xlabel('Time [s]')
    plt.ylabel('x [m]')
    plt.grid()

    plt.subplot(332, sharex=ax1)
    plt.plot(timestamp, [df.ego_car.pose.y for df in data_frames])
    plt.xlabel('Time [s]')
    plt.ylabel('y [m]')
    plt.grid()

    plt.subplot(333, sharex=ax1)
    plt.plot(timestamp, np.rad2deg([df.ego_car.pose.yaw for df in data_frames]))
    plt.xlabel('Time [s]')
    plt.ylabel('yaw [deg]')
    plt.grid()

    plt.subplot(334, sharex=ax1)
    plt.plot(timestamp, [df.ego_car.twist.vx for df in data_frames])
    plt.xlabel('Time [s]')
    plt.ylabel('vx [m/s]')
    plt.grid()

    plt.subplot(335, sharex=ax1)
    plt.plot(timestamp, [df.ego_car.twist.vy for df in data_frames])
    plt.xlabel('Time [s]')
    plt.ylabel('vy [m/s]')
    plt.grid()

    plt.subplot(336, sharex=ax1)
    plt.plot(timestamp, np.rad2deg([df.ego_car.twist.yaw_rate for df in data_frames]))
    plt.xlabel('Time [s]')
    plt.ylabel('yaw rate [deg/s]')
    plt.grid()

    plt.subplot(337, sharex=ax1)
    plt.plot(timestamp, [df.ego_car.accel.ax for df in data_frames])
    plt.xlabel('Time [s]')
    plt.ylabel('ax [m/s^2]')
    plt.grid()

    plt.subplot(338, sharex=ax1)
    plt.plot(timestamp, [df.ego_car.accel.ay for df in data_frames])
    plt.xlabel('Time [s]')
    plt.ylabel('ay [m/s^2]')
    plt.grid()

    plt.subplot(339, sharex=ax1)
    plt.plot(timestamp, np.rad2deg([df.ego_car.accel.yaw_accel for df in data_frames]))
    plt.xlabel('Time [s]')
    plt.ylabel('yaw accel [deg/s^2]')
    plt.grid()


def post_process(args):
    if args.store_data:
        data_frames = load_data(args.data_path)

        if not args.skip_fot or args.save_gif:
            plot_frames(data_frames, args)

        if args.cost_frame is not None:
            print_frame_cost(data_frames, args.cost_frame, args.cost_lane)
        
        plot_states(data_frames)
        plt.show()

if __name__=="__main__":
    args = parse_arguments()
    
    if not args.skip_fot:
        run_cpp_executable(args)
    else:
        print("Skipping FOT and only doing post-processing on data.bin")

    post_process(args)
    if args.save_gif:
        make_gif()