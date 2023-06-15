from collections import namedtuple
import msgpack

'''
Load data in msgpack format into python classes. 
The data format and deserialization are provided below. 
'''

Point = namedtuple('Point', ['x', 'y'])
Box = namedtuple('Box', ['corners'])
Pose = namedtuple('Pose', ['x', 'y', 'yaw'])
Twist = namedtuple('Twist', ['vx', 'vy', 'yaw_rate'])
Accel = namedtuple('Accel', ['ax', 'ay', 'yaw_accel'])
Obstacle = namedtuple('Obstacle', ['length', 'width', 'obstacle_clearance',
                                   'pose', 'twist', 'predict_boxes'])
Car = namedtuple('Car', ['pose', 'twist', 'accel', 'length', 'width'])
FrenetPath = namedtuple('FrenetPath', ['t', 'd', 'd_d', 'd_dd', 'd_ddd', 's', 's_d', 's_dd', 's_ddd',
                                        'x', 'y', 'yaw', 'ds', 'c', 'c_lateral_deviation',
                                        'c_lateral_velocity', 'c_lateral_acceleration',
                                        'c_lateral_jerk', 'c_lateral', 'c_longitudinal_acceleration',
                                        'c_longitudinal_jerk', 'c_time_taken', 'c_end_speed_deviation',
                                        'c_longitudinal', 'c_inv_dist_to_obstacles', 'cf'])
DataFrame = namedtuple('DataFrame', ['timestamp', 'ego_car', 'best_frenet_path',
                                     'wx', 'wy', 'obstacles', 'frenet_paths'])

def load_data(file_name): 
    with open(file_name, 'rb') as f:
        binary_data = f.read()

    unpacker = msgpack.Unpacker(raw=False)
    unpacker.feed(binary_data)
    data_frames_data = unpacker.unpack()
    # print(data_frames_data[53][:-1])
    # print(len(data_frames_data))
    # print(len(data_frames_data[79]))

    # conver the unpacked data into python classes
    data_frames = []
    for frame_data in data_frames_data:
        timestamp, ego_car_data, best_frenet_path_data, wx, wy, obstacles_data, frenet_paths_data = frame_data
        ego_car_pose_data, ego_car_twist_data, ego_car_accel_data, ego_car_length, ego_car_width = ego_car_data
        ego_car_pose = Pose(*ego_car_pose_data)
        ego_car_twist = Twist(*ego_car_twist_data)
        ego_car_accel = Accel(*ego_car_accel_data)
        ego_car = Car(ego_car_pose, ego_car_twist, ego_car_accel, ego_car_length, ego_car_width)

        best_frenet_path = FrenetPath(*best_frenet_path_data)

        obstacles = []
        for obstacle_data in obstacles_data:
            obstacle_length, obstacle_width, obstacle_clearance, obstacle_pose_data, obstacle_twist_data, obstacle_predict_boxes_data = obstacle_data
            obstacle_pose = Pose(*obstacle_pose_data)
            obstacle_twist = Twist(*obstacle_twist_data)            
            # print(obstacle_predict_boxes_data[0][0])  # debug
            obstacle_predict_boxes = [Box([Point(*corner) for corner in box[0]]) for box in obstacle_predict_boxes_data]
            obstacle = Obstacle(obstacle_length, obstacle_width, obstacle_clearance, obstacle_pose, obstacle_twist, obstacle_predict_boxes)
            obstacles.append(obstacle)

        frenet_paths = []
        for frenet_path_data in frenet_paths_data:
            frenet_path = FrenetPath(*frenet_path_data)
            frenet_paths.append(frenet_path)

        data_frame = DataFrame(timestamp, ego_car, best_frenet_path, wx, wy, obstacles, frenet_paths)
        data_frames.append(data_frame)
    
    return data_frames

if __name__=='__main__':
    # test
    data_frames = load_data('/home/sheng/Projects/l2_frenet_planner/build/data.bin')  
    print(data_frames[0].obstacles[0].predict_boxes[0])
    print(data_frames[79].ego_car)
    print(data_frames[53].best_frenet_path)
    print(data_frames[53].best_frenet_path.cf)