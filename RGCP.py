import random


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
