from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import csv
import re
import numpy as np


def resolve_self_created_case(filename):
    with open(filename, 'r') as file_to_read:
        pos = []
        while True:
            lines = file_to_read.readline()  # 整行读取数据
            if not lines:
                break
                pass
            if re.match(r'\s*[0-9]', lines) is not None:
                lines = lines.strip()
                lines = lines.split(',')
                lines = list(map(float, lines))
                pos.append(lines)  # 添加新读取的数据
            pass
    pass

    return pos


filepath = r'.\RC201.csv'
pos = resolve_self_created_case(filepath)
_station_num, _vehicle_num, _capacity, H = map(int, pos[0])
_c_ij = np.array(pos[1:_station_num + 2])
_t_ij = _c_ij / 1
_nodes_info = pos[_station_num + 2:]
_nodes, _t_lower_bound, _t_upper_bound, _demand, _t_ser, _w_i, _H, _loss = zip(*_nodes_info)
_nodes = list(map(int, _nodes))
_t_lower_bound = list(map(int, _t_lower_bound))
_t_upper_bound = list(map(int, _t_upper_bound))
_demand = list(map(int, _demand))
_t_ser = list(map(int, _t_ser))
_w_i = list(map(int, _w_i))
_H = list(map(int, _H))

distance_matrix = _c_ij * 10
time_matrix = _t_ij * 10
distance_matrix = distance_matrix.astype(int)
time_matrix = time_matrix.astype(int)

data = {}
data['vehicle_capacities'] = [_capacity] * _vehicle_num
data['num_vehicles'] = _vehicle_num
data['depot'] = 0

demands = [0] + _demand
point2Add = max(sum(demands), 0)
point2Add = int(point2Add / _capacity + 3) * _capacity

demands = demands + [-1] * point2Add

data['demands'] = demands

new_distance_matrix = np.zeros([len(distance_matrix) + point2Add, len(distance_matrix) + point2Add])
new_distance_matrix[:len(distance_matrix), :len(distance_matrix)] = distance_matrix

_length = len(distance_matrix)
new_distance_matrix[_length:, :_length] = new_distance_matrix[0, :_length]
new_distance_matrix[:_length, _length:] = new_distance_matrix[:_length, 0].reshape(_length, 1)

data['distance_matrix'] = (new_distance_matrix.astype(int)).tolist()
time_matrix = new_distance_matrix
data['time_matrix'] = time_matrix.tolist()  # 时间矩阵，单位秒

battery_time_matrix = np.zeros(len(demands))
for i in range(1, len(new_distance_matrix)):
    if i < _length - 1:
        battery_time_matrix[i] = _t_ser[i]
    else:
        battery_time_matrix[i] = 0
battery_time_matrix = battery_time_matrix * 10
battery_time_matrix = battery_time_matrix.astype(int)
data['battery_time_matrix'] = battery_time_matrix.tolist()


def print_solution(data, manager, routing, assignment, finalRoute):
    # Display dropped nodes.
    dropped_nodes = 'Dropped nodes:'
    for node in range(routing.Size()):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if assignment.Value(routing.NextVar(node)) == node:
            dropped_nodes += ' {}'.format(manager.IndexToNode(node))
    print(dropped_nodes)
    """Prints assignment on console."""
    total_distance = 0
    total_load = 0
    for vehicle_id in range(data['num_vehicles']):
        finalRoute.append([])
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        route_time = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            if node_index > _station_num and finalRoute[-1][-1] != -1:
                finalRoute[-1].append(-1)
            elif 0 < node_index <= _station_num:
                finalRoute[-1].append(node_index + _station_num + 1)
            elif node_index == 0:
                finalRoute[-1].append(0)
            route_load += data['demands'][node_index]
            plan_output += ' {0} Load({1}) Time({2}) -> '.format(node_index, route_load, route_time)
            previous_index = index
            previous_node_index = manager.IndexToNode(previous_index)
            index = assignment.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
            route_time += data['time_matrix'][previous_node_index][manager.IndexToNode(index)] + \
                          data['battery_time_matrix'][manager.IndexToNode(index)]
            # plan_output += 'Time({0}) ->'.format(route_time)
        plan_output += ' {0} Load({1}) Time({2})\n'.format(manager.IndexToNode(index),
                                                           route_load, route_time)
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        plan_output += 'Load of the route: {}\n'.format(route_load)
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
        finalRoute[-1].append(0)
    print('Total distance of all routes: {}m'.format(total_distance))
    print('Total load of all routes: {}'.format(total_load))
    print(finalRoute)


manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                       data['num_vehicles'], data['depot'])

routing = pywrapcp.RoutingModel(manager)


def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data['distance_matrix'][from_node][to_node]


transit_callback_index = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)


def demand_callback(from_index):
    from_node = manager.IndexToNode(from_index)
    return data['demands'][from_node]


demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
capacity = 'Capacity'
routing.AddDimensionWithVehicleCapacity(
    demand_callback_index,
    0,  # null capacity slack
    data['vehicle_capacities'],  # vehicle maximum capacities
    True,  # start cumul to zero
    capacity)

penalty = 10000
for node in range(1, len(data['distance_matrix'])):
    if node <= _station_num:
        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)
    else:
        routing.AddDisjunction([manager.NodeToIndex(node)], 0)

capacity_dimension = routing.GetDimensionOrDie(capacity)
for location_idx in range(len(data['demands'])):
    index = manager.NodeToIndex(location_idx)
    capacity_dimension.CumulVar(index).SetRange(max(0, -demands[location_idx]),
                                                _capacity - max(demands[location_idx], 0))


def time_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data['time_matrix'][from_node][to_node] + data['battery_time_matrix'][from_node]


time_callback_index = routing.RegisterTransitCallback(time_callback)
time = 'Time'
routing.AddDimension(
    time_callback_index,
    0,  # 没有时间缓冲
    H * 10,  # 最多排一小时
    True,  # 以0开始
    time)

search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
# Solve the problem.
assignment = routing.SolveWithParameters(search_parameters)

finalRoute = []
# Print solution on console.
if assignment:
    print_solution(data, manager, routing, assignment, finalRoute)
else:
    print('No results')

print('finished')
