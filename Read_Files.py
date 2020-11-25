import re
import numpy as np
from scipy.spatial.distance import cdist
from Routes import RouteBuilder, RouteBuilderForSoloman
from collections import Counter


def read_single_soloman(filename):
    with open(filename, 'r') as file_to_read:
        pos = []
        while True:
            lines = file_to_read.readline()  # 整行读取数据
            if not lines:
                break
                pass
            if re.match(r'\s*[0-9]', lines) is not None:
                lines = lines.strip()
                lines = lines.split()
                lines = list(map(int, lines))
                pos.append(lines)  # 添加新读取的数据
            pass
    pass
    info = pos[0]
    mat = pos[1:]
    return info, mat


def get_dis_mat(cor):
    return cdist(cor, cor)


def resolve_soloman(info, mat):
    vehicle_num, capacity = info
    mat = np.array(mat)
    dis_mat = get_dis_mat(mat[:, 1:3])
    demand = mat[:, 3]
    t_win = mat[:, 4:6]
    t_ser = mat[:, 6]
    return vehicle_num, capacity, dis_mat, demand, t_win, t_ser


def resolve_station_inventory(station_inventory_path='.\station_inventory.npy'):
    station_inventory = np.load(station_inventory_path)

    for bike_num in station_inventory:
        bike_counter = Counter()
        bl = 0
        for i in bike_num:
            bike_counter[bl] = i
            bl += 10
        yield bike_counter


def resolve_instancesBRP(filepath='1Bari30.txt'):
    with open(filepath, 'r') as file_to_read:
        pos = []
        while True:
            lines = file_to_read.readline()  # 整行读取数据
            if not lines:
                break
                pass
            if re.match(r'\s*[0-9]', lines) is not None:
                lines = lines.strip()
                lines = lines.split()
                lines = list(map(resolve_str_2_int, lines))
                pos.append(lines)  # 添加新读取的数据
            pass
    pass
    return pos[0][0], np.array(pos[1]), np.array(pos[2]), np.array(pos[3]), pos[4][0], np.array(pos[5:])


def resolve_str_2_int(x):
    if x == '1e+009' or x == '1E+009':
        return 1000000
    else:
        return int(x)


if __name__ == '__main__':
    file_path = r'.\solomon_25\R211.txt'
    _info, _mat = read_single_soloman(file_path)
    _vehicle_num, _capacity, _dis_mat, _demand, _t_win, _t_ser = resolve_soloman(_info, _mat)
    _dis_mat = np.around(_dis_mat, 1)
    _vehicle_num = 6
    routeBuilder = RouteBuilderForSoloman(_dis_mat, _dis_mat)
    routeBuilder.add_empty_route([0] * _vehicle_num, [0] * _vehicle_num, [0] * _vehicle_num,
                                 [_t_win[0, 1]] * _vehicle_num, [_capacity] * _vehicle_num)
    routeBuilder.add_tasks(list(range(1, 26)), _t_win[1:, 0], _t_win[1:, 1], _demand[1:], _t_ser[1:], [1] * 25)
    routeBuilder.build_initial_solution()
    print(routeBuilder.get_feasibility(routeBuilder.routes, range(_vehicle_num)))
    routeBuilder.print_sol()
    print(routeBuilder.get_sol_schedule())
    print(routeBuilder.best_feas_obj)
    routeBuilder.multiple_neighborhood_search()
    routeBuilder.print_sol()
    print(routeBuilder.get_sol_schedule())
    print(routeBuilder.best_feas_obj)
    '''
    print(routeBuilder.best_feas_sol)
    print('------------------')
    routeBuilder.multiple_neighborhood_search()
    routeBuilder.print_sol()
    print(routeBuilder.evaluate_solution(routeBuilder.best_feas_sol))
    print(routeBuilder.best_feas_sol)
    print('++++++++++++++++++')
    '''
    print('finished')
