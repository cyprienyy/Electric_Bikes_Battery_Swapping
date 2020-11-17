from __future__ import print_function
import random
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def RGCP(q_i, c_ij, capacity, a=1):
    vertexes = list(range(1, len(q_i)))
    routes = list()
    routes.append([0])
    route = routes[-1]
    load = 0
    load_min = 0
    load_max = 0
    m = 1
    vertex = vertexes.pop(random.randint(0, len(vertexes) - 1))
    route.append(vertex)
    load = load + q_i[vertex]
    load_min = min(load_min, load)
    load_max = max(load_max, load)
    while vertexes:
        RCL = []
        for i, vertex in enumerate(vertexes):
            if load + capacity - load_max < -min(q_i[vertex], 0) or load - load_min > capacity - max(q_i[vertex], 0):
                continue
            else:
                RCL.append((vertex, c_ij[route[-1], vertex], i))
        if RCL:
            RCL.sort(key=lambda x: x[1])
            vertex = RCL[random.randint(0, min(a - 1, len(RCL) - 1))]
            route.append(vertex[0])
            load = load + q_i[vertex[0]]
            load_min = min(load_min, load)
            load_max = max(load_max, load)
            del vertexes[vertex[2]]
        else:
            route.append(0)
            routes.append([0])
            route = routes[-1]
            m = m + 1
            load = 0
            load_min = 0
            load_max = 0
            vertex = vertexes.pop(random.randint(0, len(vertexes) - 1))
            route.append(vertex)
            load = load + q_i[vertex]
            load_min = min(load_min, load)
            load_max = max(load_max, load)
    if routes[-1][-1] != 0:
        routes[-1].append(0)
    return routes


def get_tsp_route(c_ij):
    data = {}
    data['distance_matrix'] = c_ij
    data['num_vehicles'] = 1
    data['depot'] = 0
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        return print_solution(manager, routing, solution)


def print_solution(manager, routing, solution):
    index = routing.Start(0)
    plan_output = []
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output.append(manager.IndexToNode(index))
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output.append(manager.IndexToNode(index))
    return plan_output
