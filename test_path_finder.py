import numpy as np
import argparse
import path_finder

def get_config():
    parser = argparse.ArgumentParser()
    parser.add_argument('--obstacle-map-path', '-p', help='path of obstacle map', type=str, default=None)
    parser.add_argument('--start-point', '-s', help='path of obstacle map', type=list, default=[249, 249])
    parser.add_argument('--end-point', '-e', help='path of obstacle map', type=list, default=[25, 25])
    arg = parser.parse_args()
    return arg

def made_obstacle_map(map_size=250, block_size=20):
    obstacle_map = np.zeros([map_size, map_size], dtype=np.int32)
    obstacle_map[50:150, 50:(50+block_size)] = 1
    obstacle_map[50:(50+block_size), 50:200] = 1
    return obstacle_map

if __name__ == '__main__':
    args = get_config()
    if args.obstacle_map_path is None:
        obstacle_map = made_obstacle_map()
    else:
        obstacle_map = np.load(args.obstacle_map_path, allow_pickle=True)

    start_point = np.array(args.start_point)
    end_point = np.array(args.end_point)

    path = path_finder.find(obstacle_map, start_point, end_point)
    path_splits = path.path_splits
    print(path_splits)
