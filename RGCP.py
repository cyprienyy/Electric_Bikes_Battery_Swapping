import random


def RGCP(q_i, c_ij, capacity, a=1):
    vertexes = list(range(1, len(q_i)))
    routes = list()
    routes.append([0])
    route = routes[-1]
    load = 0
    m = 1
    vertex = vertexes.pop(random.randint(0, len(vertexes) - 1))
    route.append(vertex)
    load = load + q_i[vertex]
    while vertexes:
        RCL = []
        for i, vertex in enumerate(vertexes):
            if -capacity <= q_i[vertex] + load <= capacity:
                RCL.append((vertex, c_ij[route[-1], vertex], i))
        if RCL:
            RCL.sort(key=lambda x: x[1])
            vertex = RCL[random.randint(0, min(a - 1, len(RCL) - 1))]
            route.append(vertex[0])
            load = load + q_i[vertex[0]]
            del vertexes[vertex[2]]
        else:
            route.append(0)
            routes.append([0])
            route = routes[-1]
            m = m + 1
            load = 0
            vertex = vertexes.pop(random.randint(0, len(vertexes) - 1))
            route.append(vertex)
            load = load + q_i[vertex]
    if routes[-1][-1] != 0:
        routes[-1].append(0)
    return routes
