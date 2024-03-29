import numpy as np
import random

# 定义宏观变量。
satellite = -1
upper_limit = 10000
lower_limit = 0

# 与Multiple Neighborhood Search相关宏观变量。
maxIter = 100
maxNbIter = 15
maxNonImpIter = 30
maxNbNonImpIter = 7
INF_FACTOR = 1.2
INF_STEP_UP = 0.05
INF_STEP_DOWN = 0.03
INF_LOWER_BOUND = 1
# neighborhoods = ['node_relocation']
neighborhoods = ['node_relocation',
                 'inter_routes_2opt',
                 'intra_route_2opt',
                 'nodes_swap'
                 ]
random.shuffle(neighborhoods)


class Task:
    # def __init__(self, location, start_time, end_time, task_demand, service_time):
    def __init__(self, location, start_time, end_time, task_demand, service_time, w_i, H=3600, loss=0):
        self.location = location
        self.startTime = start_time
        self.endTime = end_time
        self.demand = task_demand
        # self.onRoute = None
        self.serviceTime = service_time
        self.w_i = w_i
        self.loss = loss
        self.H = H
        # self.r_mrt = self.endTime - self.startTime


class RouteBuilder:
    def __init__(self, distance_matrix, time_matrix):
        self.c_ij = distance_matrix
        self.t_ij = time_matrix

        self.fixedKeys = set()
        self.activeKeys = set()
        self.key2Task = {}
        self.routes = []
        self.fixedRoutes = []
        self.routes_info = []

        self.banList = []
        self.best_feas_sol = None
        self.best_feas_obj = None
        self.last_sol_obj = None
        self.infFactor = 1.1
        self.unassigned_tasks = set()

        self.primary_sol = []

    @classmethod
    def copy_routes(cls, a):
        return [r.copy() for r in a]

    def add_empty_route(self, heads, tails, time_lbs, time_ubs, capacities):
        for head, tail, time_lb, time_ub, capacity in zip(heads, tails, time_lbs, time_ubs, capacities):
            self.routes.append([head, tail])
            self.fixedRoutes.append([head])
            self.routes_info.append(
                {'current_time': time_lb, 'time_ub': time_ub, 'current_load': 0, 'capacity': capacity})
            self.primary_sol.append([])

    def add_tasks(self, *args):
        if self.fixedKeys:
            start_key = max(max(self.fixedKeys), self.c_ij.shape[0])
        else:
            start_key = self.c_ij.shape[0]

        for _task_param in zip(*args):
            start_key = start_key + 1
            task = Task(*_task_param)
            self.activeKeys.add(start_key)
            self.key2Task[start_key] = task

    def trans_key_to_station(self, key):
        """
        把任务的key翻译成对应的节点。
        """
        if key < 0:
            return 0
        elif key < self.c_ij.shape[0]:
            return key
        else:
            return self.key2Task[key].location

    def evaluate_solution(self, routes, tour_id):
        """
        To do:
        改掉evaluate_solution_by_time_func_of_task()。
        """
        return self.evaluate_solution_by_total_distance(routes) + self.evaluate_solution_by_time_func_of_task(routes,
                                                                                                              tour_id)

    def evaluate_solution_by_total_distance(self, routes):
        """
        以所有路径总长度之和作为目标函数，返回routes的Obj。
        返回舍入到一位小数后的值。
        """
        total_dis = 0
        for r in routes:
            for i in range(0, len(r) - 1):
                total_dis = total_dis + self.c_ij[self.trans_key_to_station(r[i]), self.trans_key_to_station(r[i + 1])]
        return round(total_dis, 1)

    def evaluate_solution_by_time_func_of_task(self, routes, tour_id):
        """
        计算出routes中每个任务的到达时间，然后依据func计算每个任务的目标函数值。
        然后相加，作为routes的Obj。
        To do：
        是否增加舍入。
        """
        total_ob = 0
        for i, r in enumerate(routes):
            current_time = self.routes_info[tour_id[i]]['current_time'] - self.get_service_time(r[0])
            for j in range(0, len(r) - 1):
                current_time = current_time + self.get_service_time(r[j]) + self.t_ij[
                    self.trans_key_to_station(r[j]), self.trans_key_to_station(r[j + 1])]
                total_ob = total_ob + self.linear_response_time_cost_of_task(r[j + 1], current_time)
        return total_ob

    def linear_response_time_cost_of_task(self, task_key, task_arrive_time):
        if task_key >= self.c_ij.shape[0]:
            _task = self.key2Task[task_key]
            if task_arrive_time > _task.endTime and _task.loss != 0:
                return round(_task.loss * (task_arrive_time - _task.endTime) / (_task.H - _task.endTime), 1)
            else:
                return 0
        else:
            return 0

    '''
    def quadratic_response_time_cost_of_task(self, task_key, task_arrive_time):
        """
        以二次函数计算task的response time导致的惩罚。
        To do:
        是否加上超时产生的penalty
        """
        if task_key >= self.c_ij.shape[0]:
            _task = self.key2Task[task_key]
            _t_i = task_arrive_time - _task.startTime
            return _task.w_i * (_t_i / _task.r_mrt) ** 2
        else:
            return 0

    def linear_response_time_cost_of_task(self, task_key, task_arrive_time):
        """
        以线性函数计算task的response time导致的惩罚
        To do：
        是否加上超时产生的penalty
        """
        if task_key >= self.c_ij.shape[0]:
            _task = self.key2Task[task_key]
            _t_i = task_arrive_time - _task.startTime
            return _task.w_i / _task.r_mrt * (min(_t_i, _task.r_mrt) + 2 * max(0, _t_i - _task.r_mrt))
        else:
            return 0
    '''

    def get_feasibility(self, routes, tour_id):
        """
        用于检查路径是否feasible，总是传入一个路径列表,输出对应长度的feasibility列表。
        """
        return [all(f_col) for f_col in
                zip(self.get_load_feasibility(routes, tour_id),
                    self.get_period_upper_bound_feasibility(routes, tour_id))]

    def get_period_upper_bound_feasibility(self, routes, tour_id):
        t_feas = []
        for i, r in enumerate(routes):
            period = self.routes_info[tour_id[i]]['time_ub']
            total_time = self.routes_info[tour_id[i]]['current_time']
            for j in range(0, len(r) - 1):
                total_time = total_time + self.t_ij[
                    self.trans_key_to_station(r[j]), self.trans_key_to_station(r[j + 1])]
                if r[j + 1] > self.c_ij.shape[0]:
                    total_time = total_time + self.key2Task[r[j + 1]].serviceTime
            if total_time > period:
                t_feas.append(False)
            else:
                t_feas.append(True)
        return t_feas

    def get_time_window_feasibility(self, routes, tour_id):
        t_feas = []
        for i, r in enumerate(routes):
            t_f = True
            period = self.routes_info[tour_id[i]]['time_ub']
            s_jr = np.zeros(len(r))
            w_jr = np.zeros(len(r))
            s_jr[0] = self.routes_info[tour_id[i]]['current_time'] - self.get_service_time(r[0])
            for j in range(1, len(r)):
                s_jr[j] = s_jr[j - 1] + self.t_ij[
                    self.trans_key_to_station(r[j - 1]), self.trans_key_to_station(r[j])] + self.get_service_time(
                    r[j - 1])
                if s_jr[j] <= self.get_tw_lb(r[j]):
                    w_jr[j] = self.get_tw_lb(r[j]) - s_jr[j]
                    s_jr[j] = self.get_tw_lb(r[j])
                else:
                    w_jr[j] = 0
            for j in range(1, len(r)):
                if s_jr[j] > self.get_tw_ub(r[j]):
                    t_f = False
            if s_jr[-1] > period:
                t_f = False
            t_feas.append(t_f)
        return t_feas

    def get_tw_ub(self, task_key):
        if task_key < self.c_ij.shape[0]:
            return upper_limit
        else:
            return self.key2Task[task_key].endTime

    def get_tw_lb(self, task_key):
        if task_key < self.c_ij.shape[0]:
            return lower_limit
        else:
            return self.key2Task[task_key].startTime

    def get_service_time(self, task_key):
        if task_key < 0:
            return 0
        elif task_key < self.c_ij.shape[0]:
            return 0
        else:
            return self.key2Task[task_key].serviceTime

    def get_load_feasibility(self, routes, tour_id):
        l_feas = []
        for j, r in enumerate(routes):
            l_f = True
            capacity = self.routes_info[tour_id[j]]['capacity']
            load = self.routes_info[tour_id[j]]['current_load']
            for i in r[1:]:
                if i < self.c_ij.shape[0]:
                    load = 0
                else:
                    load = load + self.key2Task[i].demand
                if load > capacity:
                    l_f = False
            l_feas.append(l_f)
        return l_feas

    def get_feasibility_for_solution_construction(self, routes, tour_id):
        return self.get_period_upper_bound_feasibility(routes, tour_id)

    def build_initial_solution(self):
        self.banList = []
        unassigned_nodes = self.activeKeys.copy()
        add_move = None
        num_tour = 0
        sol = self.copy_routes(self.routes)
        while num_tour < len(sol) and unassigned_nodes:
            insertion_allowed = True
            while insertion_allowed and unassigned_nodes:
                best_sol = float('inf')
                move = None
                for i in unassigned_nodes:
                    for j in range(1, len(sol[num_tour])):
                        sigma_1 = sol[num_tour][:j]
                        sigma_2 = sol[num_tour][j:]
                        obj = self.evaluate_solution([sigma_1 + [i] + sigma_2], [num_tour])
                        f = all(self.get_feasibility([sigma_1 + [i] + sigma_2], [num_tour]))
                        if obj < best_sol and f:
                            move = (i, j)
                            best_sol = obj
                if move:
                    i = move[0]
                    j = move[1]
                    sigma_1 = sol[num_tour][:j]
                    sigma_2 = sol[num_tour][j:]
                    sol[num_tour] = sigma_1 + [i] + sigma_2
                    unassigned_nodes.remove(move[0])
                    add_move = None
                else:
                    if add_move:
                        j = add_move[1]
                        sigma_1 = sol[num_tour][:j]
                        sigma_2 = sol[num_tour][j + 1:]
                        sol[num_tour] = sigma_1 + sigma_2
                        add_move = None
                        insertion_allowed = False
                    else:
                        best_add_sol = float('inf')
                        for j in range(1, len(sol[num_tour])):
                            if sol[num_tour][j - 1] != (satellite and 0) and sol[num_tour][j] != (satellite and 0):
                                sigma_1 = sol[num_tour][:j]
                                sigma_2 = sol[num_tour][j:]
                                add_obj = self.evaluate_solution([sigma_1 + [satellite] + sigma_2], [num_tour])
                                # To do:
                                # 这里需要合理选择
                                add_f = all(
                                    self.get_feasibility_for_solution_construction([sigma_1 + [satellite] + sigma_2],
                                                                                   [num_tour]))
                                if add_obj < best_add_sol and add_f:
                                    add_move = (satellite, j)
                                    best_add_sol = add_obj
                        if add_move is None:
                            j = len(sol[num_tour]) - 1
                            sigma_1 = sol[num_tour][:j]
                            sigma_2 = sol[num_tour][j:]
                            add_f = all(
                                self.get_feasibility_for_solution_construction([sigma_1 + [satellite] + sigma_2],
                                                                               [num_tour]))
                            if add_f:
                                add_move = (satellite, j)
                        if add_move:
                            i = add_move[0]
                            j = add_move[1]
                            sigma_1 = sol[num_tour][:j]
                            sigma_2 = sol[num_tour][j:]
                            sol[num_tour] = sigma_1 + [i] + sigma_2
                        else:
                            insertion_allowed = False
            num_tour = num_tour + 1
        self.unassigned_tasks = unassigned_nodes
        self.routes = self.copy_routes(sol)
        self.banList.append(self.evaluate_solution(self.routes, list(range(len(self.routes)))))
        self.best_feas_sol = self.copy_routes(self.routes)
        self.best_feas_obj = self.evaluate_solution(self.best_feas_sol, list(range(len(self.routes))))
        self.last_sol_obj = self.best_feas_obj

    def pass_initial_solution(self, routes):
        self.routes = self.copy_routes(routes)
        self.banList = []
        self.banList.append(self.evaluate_solution(self.routes, list(range(len(self.routes)))))
        self.best_feas_sol = self.copy_routes(self.routes)
        self.best_feas_obj = self.evaluate_solution(self.best_feas_sol, list(range(len(self.routes))))
        self.last_sol_obj = self.best_feas_obj

    def parallel_insertion(self):
        self.banList = []
        unassigned_nodes = list(self.activeKeys)
        unassigned_nodes.sort(key=lambda x: self.key2Task[x].w_i, reverse=True)
        add_move = None
        num_tour = 0
        sol = self.copy_routes(self.routes)
        while num_tour < len(sol) and unassigned_nodes:
            insertion_allowed = True
            while insertion_allowed and unassigned_nodes:
                best_sol = float('inf')
                move = None
                for i in unassigned_nodes[:1]:
                    for j in range(1, len(sol[num_tour])):
                        sigma_1 = sol[num_tour][:j]
                        sigma_2 = sol[num_tour][j:]
                        obj = self.evaluate_solution([sigma_1 + [i] + sigma_2], [num_tour])
                        f = all(self.get_feasibility([sigma_1 + [i] + sigma_2], [num_tour]))
                        if obj < best_sol and f:
                            move = (i, j)
                            best_sol = obj
                if move:
                    i = move[0]
                    j = move[1]
                    sigma_1 = sol[num_tour][:j]
                    sigma_2 = sol[num_tour][j:]
                    sol[num_tour] = sigma_1 + [i] + sigma_2
                    unassigned_nodes.remove(move[0])
                    add_move = None
                else:
                    if add_move:
                        j = add_move[1]
                        sigma_1 = sol[num_tour][:j]
                        sigma_2 = sol[num_tour][j + 1:]
                        sol[num_tour] = sigma_1 + sigma_2
                        add_move = None
                        insertion_allowed = False
                    else:
                        best_add_sol = float('inf')
                        for j in range(1, len(sol[num_tour])):
                            if sol[num_tour][j - 1] != (satellite and 0) and sol[num_tour][j] != (satellite and 0):
                                sigma_1 = sol[num_tour][:j]
                                sigma_2 = sol[num_tour][j:]
                                add_obj = self.evaluate_solution([sigma_1 + [satellite] + sigma_2], [num_tour])
                                # To do:
                                # 这里需要合理选择
                                add_f = all(
                                    self.get_feasibility_for_solution_construction([sigma_1 + [satellite] + sigma_2],
                                                                                   [num_tour]))
                                if add_obj < best_add_sol and add_f:
                                    add_move = (satellite, j)
                                    best_add_sol = add_obj
                        if add_move:
                            i = add_move[0]
                            j = add_move[1]
                            sigma_1 = sol[num_tour][:j]
                            sigma_2 = sol[num_tour][j:]
                            sol[num_tour] = sigma_1 + [i] + sigma_2
                        else:
                            insertion_allowed = False
            num_tour = num_tour + 1
        self.routes = self.copy_routes(sol)
        self.banList.append(self.evaluate_solution(self.routes, list(range(len(self.routes)))))
        self.best_feas_sol = self.copy_routes(self.routes)
        self.best_feas_obj = self.evaluate_solution(self.best_feas_sol, list(range(len(self.routes))))
        self.last_sol_obj = self.best_feas_obj

    def add_unassigned_tasks(self):
        unassigned_nodes = self.unassigned_tasks.copy()
        unassigned_nodes_num = len(unassigned_nodes)
        add_move = None
        num_tour = 0
        sol = self.copy_routes(self.routes)
        while num_tour < len(sol) and unassigned_nodes:
            insertion_allowed = True
            while insertion_allowed and unassigned_nodes:
                best_sol = float('inf')
                move = None
                for i in unassigned_nodes:
                    for j in range(1, len(sol[num_tour])):
                        sigma_1 = sol[num_tour][:j]
                        sigma_2 = sol[num_tour][j:]
                        obj = self.evaluate_solution([sigma_1 + [i] + sigma_2], [num_tour])
                        f = all(self.get_feasibility([sigma_1 + [i] + sigma_2], [num_tour]))
                        if obj < best_sol and f:
                            move = (i, j)
                            best_sol = obj
                if move:
                    i = move[0]
                    j = move[1]
                    sigma_1 = sol[num_tour][:j]
                    sigma_2 = sol[num_tour][j:]
                    sol[num_tour] = sigma_1 + [i] + sigma_2
                    unassigned_nodes.remove(move[0])
                    add_move = None
                else:
                    if add_move:
                        j = add_move[1]
                        sigma_1 = sol[num_tour][:j]
                        sigma_2 = sol[num_tour][j + 1:]
                        sol[num_tour] = sigma_1 + sigma_2
                        add_move = None
                        insertion_allowed = False
                    else:
                        best_add_sol = float('inf')
                        for j in range(1, len(sol[num_tour])):
                            if sol[num_tour][j - 1] != (satellite and 0) and sol[num_tour][j] != (satellite and 0):
                                sigma_1 = sol[num_tour][:j]
                                sigma_2 = sol[num_tour][j:]
                                add_obj = self.evaluate_solution([sigma_1 + [satellite] + sigma_2], [num_tour])
                                # To do:
                                # 这里需要合理选择
                                add_f = all(
                                    self.get_feasibility_for_solution_construction([sigma_1 + [satellite] + sigma_2],
                                                                                   [num_tour]))
                                if add_obj < best_add_sol and add_f:
                                    add_move = (satellite, j)
                                    best_add_sol = add_obj
                        if add_move is None:
                            j = len(sol[num_tour]) - 1
                            sigma_1 = sol[num_tour][:j]
                            sigma_2 = sol[num_tour][j:]
                            add_f = all(
                                self.get_feasibility_for_solution_construction([sigma_1 + [satellite] + sigma_2],
                                                                               [num_tour]))
                            if add_f:
                                add_move = (satellite, j)
                        if add_move:
                            i = add_move[0]
                            j = add_move[1]
                            sigma_1 = sol[num_tour][:j]
                            sigma_2 = sol[num_tour][j:]
                            sol[num_tour] = sigma_1 + [i] + sigma_2
                        else:
                            insertion_allowed = False
            num_tour = num_tour + 1
        self.unassigned_tasks = unassigned_nodes.copy()
        if len(unassigned_nodes) < unassigned_nodes_num:
            self.routes = self.copy_routes(sol)
            self.banList = []
            self.banList.append(self.evaluate_solution(self.routes, list(range(len(self.routes)))))
            self.best_feas_sol = self.copy_routes(self.routes)
            self.best_feas_obj = self.evaluate_solution(self.best_feas_sol, list(range(len(self.routes))))
            self.last_sol_obj = self.best_feas_obj
            return True
        else:
            return False

    # 关于infFactor
    def reset_inf_factor(self, init_val=INF_FACTOR):
        self.infFactor = init_val

    def update_inf_factor(self, symbol, inf_step_up=INF_STEP_UP, inf_step_down=INF_STEP_DOWN):
        if symbol > 0:
            self.infFactor = self.infFactor + inf_step_up
        else:
            self.infFactor = self.infFactor - inf_step_down
        if self.infFactor < INF_LOWER_BOUND:
            self.infFactor = INF_LOWER_BOUND

    def node_relocation(self):
        routes = self.copy_routes(self.routes)
        S_sol = self.evaluate_solution(routes, range(len(routes)))
        r_feas = self.get_feasibility(routes, range(len(routes)))
        bestSol = float('inf')
        saveMove = None
        savef = False

        total_try = 0
        for i, route_i in enumerate(routes):
            total_try += len(route_i) - 1
        do_or_not = [0] * (total_try - 10) + [1] * 10

        for i in range(len(routes)):
            route_i = routes[i]
            S_i = self.evaluate_solution([route_i], [i])
            for k in range(1, len(route_i) - 1):
                sigma_1 = route_i[0:k]
                sigma_2 = route_i[k:k + 1]
                sigma_3 = route_i[k + 1:]
                random.shuffle(do_or_not)
                count = -1
                for j in range(len(routes)):
                    route_j = routes[j]
                    S_j = self.evaluate_solution([route_j], [j])
                    for l in range(len(route_j) - 1):
                        count += 1
                        if do_or_not[count] == 0:
                            continue
                        sigma_4 = route_j[0:l + 1]
                        sigma_5 = route_j[l + 1:]
                        if i != j:
                            obj = self.evaluate_solution([sigma_1 + sigma_3], [i])
                            obj = obj + self.evaluate_solution([sigma_4 + sigma_2 + sigma_5], [j])
                            obj = obj + S_sol - S_i - S_j
                            obj = round(obj, 1)
                            r_feas_temp = r_feas.copy()
                            r_feas_update = self.get_feasibility([sigma_1 + sigma_3, sigma_4 + sigma_2 + sigma_5],
                                                                 [i, j])
                            r_feas_temp[i] = r_feas_update[0]
                            r_feas_temp[j] = r_feas_update[1]
                            f = all(r_feas_temp)
                        else:
                            if l < k:
                                # sigma_5.remove(route_i[k])
                                sigma_5.pop(k - l - 1)
                            else:
                                # sigma_4.remove(route_i[k])
                                sigma_4.pop(k)
                            obj = self.evaluate_solution([sigma_4 + sigma_2 + sigma_5], [i])
                            obj = obj + S_sol - S_i
                            obj = round(obj, 1)
                            r_feas_temp = r_feas.copy()
                            r_feas_update = self.get_feasibility([sigma_4 + sigma_2 + sigma_5], [i])
                            r_feas_temp[i] = r_feas_update[0]
                            f = all(r_feas_temp)
                        if obj not in self.banList:
                            if not f:
                                obj = obj * self.infFactor
                            if obj < bestSol:
                                bestSol = obj
                                saveMove = (i, k, j, l)
                                savef = f
                if count != len(do_or_not) - 1:
                    raise Exception('relocation_random_error')

        if saveMove:
            i = saveMove[0]
            k = saveMove[1]
            j = saveMove[2]
            l = saveMove[3]
            sigma_1 = routes[i][0:k]
            sigma_2 = routes[i][k:k + 1]
            sigma_3 = routes[i][k + 1:]
            sigma_4 = routes[j][0:l + 1]
            sigma_5 = routes[j][l + 1:]
            if i != j:
                routes[i] = sigma_1 + sigma_3
                routes[j] = sigma_4 + sigma_2 + sigma_5
            else:
                if l < k:
                    # sigma_5.remove(routes[i][k])
                    sigma_5.pop(k - l - 1)
                else:
                    # sigma_4.remove(routes[i][k])
                    sigma_4.pop(k)
                routes[i] = sigma_4 + sigma_2 + sigma_5
        return routes, savef, bestSol < self.last_sol_obj

    def nodes_swap(self):
        routes = self.copy_routes(self.routes)
        S_sol = self.evaluate_solution(routes, range(len(routes)))
        r_feas = self.get_feasibility(routes, range(len(routes)))
        bestSol = float('inf')
        saveMove = None
        savef = False

        total_try = 0
        len_routes = 0
        for i, route_i in enumerate(routes):
            len_routes += len(route_i) - 2
        total_try += len_routes * (len_routes - 1) // 2
        try_num = min(10 * len_routes, total_try)
        do_or_not = [0] * (total_try - try_num) + [1] * try_num
        random.shuffle(do_or_not)
        count = -1

        for i in range(len(routes)):
            route_i = routes[i].copy()
            S_i = self.evaluate_solution([route_i], [i])
            for k in range(1, len(route_i) - 1):
                for l in range(k + 1, len(route_i) - 1):
                    count += 1
                    if do_or_not[count] == 0:
                        continue
                    if route_i[k] != route_i[l]:
                        route_i[k], route_i[l] = route_i[l], route_i[k]
                        obj = self.evaluate_solution([route_i], [i])
                        obj = obj + S_sol - S_i
                        obj = round(obj, 1)
                        r_feas_temp = r_feas.copy()
                        r_feas_update = self.get_feasibility([route_i], [i])
                        r_feas_temp[i] = r_feas_update[0]
                        route_i[k], route_i[l] = route_i[l], route_i[k]
                        f = all(r_feas_temp)
                        if obj not in self.banList:
                            if not f:
                                obj = obj * self.infFactor
                            if obj < bestSol:
                                bestSol = obj
                                saveMove = (i, k, i, l)
                                savef = f

                for j in range(i + 1, len(routes)):
                    route_j = routes[j].copy()
                    S_j = self.evaluate_solution([route_j], [j])
                    for l in range(1, len(route_j) - 1):
                        count += 1
                        if do_or_not[count] == 0:
                            continue
                        if route_i[k] != route_j[l]:
                            route_i[k], route_j[l] = route_j[l], route_i[k]
                            obj = self.evaluate_solution([route_i], [i])
                            obj = obj + self.evaluate_solution([route_j], [j])
                            obj = obj + S_sol - S_i - S_j
                            obj = round(obj, 1)
                            r_feas_temp = r_feas.copy()
                            r_feas_update = self.get_feasibility([route_i, route_j], [i, j])
                            r_feas_temp[i] = r_feas_update[0]
                            r_feas_temp[j] = r_feas_update[1]
                            route_i[k], route_j[l] = route_j[l], route_i[k]
                            f = all(r_feas_temp)
                            if obj not in self.banList:
                                if not f:
                                    obj = obj * self.infFactor
                                if obj < bestSol:
                                    bestSol = obj
                                    saveMove = (i, k, j, l)
                                    savef = f
        if count != len(do_or_not) - 1:
            raise Exception('exchange_random_error')
        if saveMove:
            i = saveMove[0]
            k = saveMove[1]
            j = saveMove[2]
            l = saveMove[3]
            routes[i][k], routes[j][l] = routes[j][l], routes[i][k]
        return routes, savef, bestSol < self.last_sol_obj

    def intra_route_2opt(self):
        routes = self.copy_routes(self.routes)
        r_feas = self.get_feasibility(routes, range(len(routes)))
        S_sol = self.evaluate_solution(routes, range(len(routes)))
        bestSol = float('inf')
        saveMove = None
        savef = False

        for i in range(len(routes)):
            route_i = routes[i]
            S_i = self.evaluate_solution([route_i], [i])

            len_route_i = len(route_i) - 2
            total_try = len_route_i * (len_route_i - 1) // 2 - max((len_route_i - 1), 0)
            try_num = min(10 * len_route_i, total_try)
            do_or_not = [0] * (total_try - try_num) + [1] * try_num
            random.shuffle(do_or_not)
            count = -1

            for k in range(1, len(route_i) - 3):
                for l in range(k + 2, len(route_i) - 1):  # 交换的是点k到点l,且把这种方式与swap区分开来了

                    count += 1
                    if do_or_not[count] == 0:
                        continue

                    sigma_1 = route_i[0:k]
                    sigma_2 = route_i[k:l + 1]
                    sigma_2.reverse()
                    sigma_3 = route_i[l + 1:]
                    obj = self.evaluate_solution([sigma_1 + sigma_2 + sigma_3], [i])
                    obj = obj + S_sol - S_i
                    obj = round(obj, 1)
                    r_feas_temp = r_feas.copy()
                    r_feas_update = self.get_feasibility([sigma_1 + sigma_2 + sigma_3], [i])
                    r_feas_temp[i] = r_feas_update[0]
                    f = all(r_feas_temp)
                    if obj not in self.banList:
                        if not f:
                            obj = obj * self.infFactor
                        if obj < bestSol:
                            bestSol = obj
                            saveMove = (i, k, l)
                            savef = f
            if count != len(do_or_not) - 1:
                raise Exception('intra_2opt_random_error')
        if saveMove:
            i = saveMove[0]
            k = saveMove[1]
            l = saveMove[2]
            sigma_1 = routes[i][0:k]
            sigma_2 = routes[i][k:l + 1]
            sigma_2.reverse()
            sigma_3 = routes[i][l + 1:]
            routes[i] = sigma_1 + sigma_2 + sigma_3
        return routes, savef, bestSol < self.last_sol_obj

    def inter_routes_2opt(self):
        routes = self.copy_routes(self.routes)
        r_feas = self.get_feasibility(routes, range(len(routes)))
        S_sol = self.evaluate_solution(routes, range(len(routes)))
        bestSol = float('inf')
        saveMove = None
        savef = False

        total_try = 0
        len_routes = 0
        for i, route_i in enumerate(routes):
            len_route_i = len(route_i) - 1
            len_routes += len_route_i
            total_try -= len_route_i * (len_route_i - 1) // 2
        total_try += len_routes * (len_routes - 1) // 2
        try_num = min(10 * len_routes, total_try)
        do_or_not = [0] * (total_try - try_num) + [1] * try_num
        random.shuffle(do_or_not)
        count = -1

        for i in range(len(routes)):
            route_i = routes[i]
            S_i = self.evaluate_solution([route_i], [i])
            for j in range(i + 1, len(routes)):
                route_j = routes[j]
                S_j = self.evaluate_solution([route_j], [j])
                for k in range(len(route_i) - 1):
                    sigma_1 = route_i[0:k + 1]
                    sigma_2 = route_i[k + 1:]
                    for l in range(len(route_j) - 1):

                        count += 1
                        if do_or_not[count] == 0:
                            continue

                        sigma_3 = route_j[0:l + 1]
                        sigma_4 = route_j[l + 1:]
                        obj = self.evaluate_solution([sigma_1 + sigma_4], [i])
                        obj = obj + self.evaluate_solution([sigma_3 + sigma_2], [j])
                        obj = obj + S_sol - S_i - S_j
                        obj = round(obj, 1)
                        r_feas_temp = r_feas.copy()
                        r_feas_update = self.get_feasibility([sigma_1 + sigma_4, sigma_3 + sigma_2], [i, j])
                        r_feas_temp[i] = r_feas_update[0]
                        r_feas_temp[j] = r_feas_update[1]
                        f = all(r_feas_temp)
                        if obj not in self.banList:
                            if not f:
                                obj = obj * self.infFactor
                            if obj < bestSol:
                                bestSol = obj
                                saveMove = (i, k, j, l)
                                savef = f
        if count != len(do_or_not) - 1:
            raise Exception('inter_2opt_random_error')
        if saveMove:
            i = saveMove[0]
            k = saveMove[1]
            j = saveMove[2]
            l = saveMove[3]
            sigma_1 = routes[i][0:k + 1]
            sigma_2 = routes[i][k + 1:]
            sigma_3 = routes[j][0:l + 1]
            sigma_4 = routes[j][l + 1:]
            routes[i] = sigma_1 + sigma_4
            routes[j] = sigma_3 + sigma_2
        return routes, savef, bestSol < self.last_sol_obj

    def get_neighborhood(self, neighborhood_index):
        if neighborhood_index == 'node_relocation':
            return self.node_relocation()
        elif neighborhood_index == 'inter_routes_2opt':
            return self.inter_routes_2opt()
        elif neighborhood_index == 'intra_route_2opt':
            return self.intra_route_2opt()
        elif neighborhood_index == 'nodes_swap':
            return self.nodes_swap()
        else:
            print('~~')
            return self.routes, True, False

    def multiple_neighborhood_search(self):
        self.reset_inf_factor()
        NonImpIter = 0
        self.best_feas_sol = self.copy_routes(self.routes)

        for it in range(maxIter):
            ImpIter = False
            for nh in neighborhoods:
                # print('nh', nh)
                NbNonImpIter = 0
                for Nbit in range(maxNbIter):
                    neighbor = self.get_neighborhood(nh)  # neighbor的组成为路径、是否可行、是否有所改善
                    self.routes = neighbor[0]
                    # update banlist
                    self.banList.append(self.evaluate_solution(self.routes, range(len(self.routes))))
                    # update infFactor
                    self.last_sol_obj = self.evaluate_solution(self.routes, range(len(self.routes))) if neighbor[
                        1] else round(self.evaluate_solution(self.routes, range(len(self.routes))) * self.infFactor, 1)
                    # print('++', self.last_sol_obj, self.infFactor)
                    if neighbor[1]:
                        self.update_inf_factor(-1)
                    else:
                        self.update_inf_factor(1)
                    if neighbor[2]:
                        if neighbor[1]:
                            if self.evaluate_solution(self.routes, range(len(self.routes))) <= self.best_feas_obj - 0.1:
                                # print('Found A Better Solution')
                                self.best_feas_sol = self.copy_routes(self.routes)
                                self.best_feas_obj = self.evaluate_solution(self.best_feas_sol,
                                                                            range(len(self.best_feas_sol)))
                                if self.unassigned_tasks:
                                    if self.add_unassigned_tasks():
                                        self.reset_inf_factor()
                                        self.multiple_neighborhood_search()
                                        return
                                ImpIter = True
                            if all(self.get_feasibility(self.best_feas_sol, range(25))) is False:
                                print('The New Best Solution is Infeasible')
                    else:
                        NbNonImpIter = NbNonImpIter + 1
                        # print('NbNonImpIter', NbNonImpIter)
                    if NbNonImpIter >= maxNbNonImpIter:
                        self.routes = self.copy_routes(self.best_feas_sol)
                        self.last_sol_obj = self.best_feas_obj
                        self.reset_inf_factor()
                        break
                self.reset_inf_factor()
                self.routes = self.copy_routes(self.best_feas_sol)
                self.last_sol_obj = self.best_feas_obj
            if ImpIter is False:
                NonImpIter = NonImpIter + 1
                # print('NonImpIter', NonImpIter)
            if NonImpIter >= maxNonImpIter:
                break
            # print('Iter', it)

    def print_sol(self):
        """
        To do:
        是否打印fixed routes。
        """
        for r in self.best_feas_sol:
            if len(r) > 2:
                print([self.trans_key_to_station(i) for i in r])
        return

    def get_sol_schedule(self):
        _tasks = []
        for i, r in enumerate(self.best_feas_sol):
            s_jr = np.zeros(len(r))
            w_jr = np.zeros(len(r))
            s_jr[0] = self.routes_info[i]['current_time'] - self.get_service_time(r[0])
            for j in range(1, len(r)):
                s_jr[j] = s_jr[j - 1] + self.t_ij[
                    self.trans_key_to_station(r[j - 1]), self.trans_key_to_station(r[j])] + self.get_service_time(
                    r[j - 1])
                if s_jr[j] <= self.get_tw_lb(r[j]):
                    w_jr[j] = self.get_tw_lb(r[j]) - s_jr[j]
                    s_jr[j] = self.get_tw_lb(r[j])
                else:
                    w_jr[j] = 0
            for j in range(1, len(r) - 1):
                if r[j] in self.activeKeys:
                    _tasks.append(
                        (self.trans_key_to_station(r[j]), s_jr[j], s_jr[j] + w_jr[j + 1] + self.get_service_time(r[j]),
                         self.key2Task[r[j]].demand))
        return _tasks

    def fix_sol(self, lp):
        if any(self.primary_sol) and self.get_feasibility(self.primary_sol, list(range(len(self.primary_sol)))):
            if self.evaluate_solution(self.primary_sol, list(range(len(self.primary_sol)))) < self.best_feas_obj:
                self.pass_initial_solution(self.primary_sol)
        _res = []
        for i, r in enumerate(self.best_feas_sol):
            s_jr = np.zeros(len(r))
            w_jr = np.zeros(len(r))
            s_jr[0] = self.routes_info[i]['current_time'] - self.get_service_time(r[0])
            for j in range(1, len(r)):
                s_jr[j] = s_jr[j - 1] + self.t_ij[
                    self.trans_key_to_station(r[j - 1]), self.trans_key_to_station(r[j])] + self.get_service_time(
                    r[j - 1])
                if s_jr[j] <= self.get_tw_lb(r[j]):
                    w_jr[j] = self.get_tw_lb(r[j]) - s_jr[j]
                    s_jr[j] = self.get_tw_lb(r[j])
                else:
                    w_jr[j] = 0
            cur = 0
            for j in range(1, len(r) - 1):
                if s_jr[j] - self.t_ij[self.trans_key_to_station(r[j - 1]), self.trans_key_to_station(r[j])] < lp:
                    cur = j
                else:
                    break
            # current_load是经过cur点以后的， current_time是服务过cur以后的
            self.routes_info[i]['current_time'] = s_jr[cur] + self.get_service_time(r[cur])
            load = self.routes_info[i]['current_load']
            for j in r[1:cur + 1]:
                if j < self.c_ij.shape[0]:
                    load = 0
                else:
                    load = load + self.key2Task[j].demand
            self.routes_info[i]['current_load'] = load
            for k, j in enumerate(r[1:cur + 1]):
                if j > self.c_ij.shape[0]:
                    _res.append(
                        (
                            self.trans_key_to_station(j), s_jr[k + 1],
                            s_jr[k + 1] + w_jr[k + 2] + self.get_service_time(j),
                            self.key2Task[j].demand))
                    self.fixedKeys.add(j)
                    self.activeKeys.remove(j)
                self.fixedRoutes[i].append(j)
            '''
            for _, j in enumerate(r[cur + 1:-1]):
                del self.key2Task[j]
            '''
            self.primary_sol[i] = r[cur:]
            temp_r = [r[cur], r[-1]]
            self.best_feas_sol[i] = temp_r
        self.routes = self.copy_routes(self.best_feas_sol)
        return _res


class RouteBuilderByDistance(RouteBuilder):
    def __init__(self, distance_matrix, time_matrix):
        RouteBuilder.__init__(self, distance_matrix, time_matrix)

    def evaluate_solution(self, routes, tour_id):
        return self.evaluate_solution_by_total_distance(routes)


class RouteBuilderForSoloman(RouteBuilder):
    def __init__(self, distance_matrix, time_matrix):
        RouteBuilder.__init__(self, distance_matrix, time_matrix)

    def evaluate_solution(self, routes, tour_id):
        return self.evaluate_solution_by_total_distance(routes)

    def get_feasibility(self, routes, tour_id):
        return [all(f_col) for f_col in
                zip(self.get_load_feasibility(routes, tour_id),
                    self.get_time_window_feasibility(routes, tour_id))]


if __name__ == '__main__':
    print('Main Function')
