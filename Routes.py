import numpy as np

# 定义宏观变量
satellite = -1
upper_limit = 7200
lower_limit = 0
#maxIter = 100
maxIter = 100
maxNbIter = 30
maxNonImpIter = 15
maxNbNonImpIter = 7
neighborhoods = ['nodes_swap']
#neighborhoods = ['node_relocation', 'inter_routes_2opt', 'intra_route_2opt', 'nodes_swap']


class Task:
    def __init__(self, location, start_time, end_time, task_demand, service_time):
        self.location = location
        self.startTime = start_time
        self.endTime = end_time
        self.demand = task_demand
        self.onRoute = None
        self.serviceTime = service_time


'''
class Route:
    def __init__(self, time_lb, time_ub, head='head', tail='tail'):
        self.link = {head: (None, tail), tail: (head, None)}
        self.currentHead = head
        self.timeLB = time_lb
        self.timeUB = time_ub
'''


class RouteBuilder:
    def __init__(self, distance_matrix, time_matrix):
        self.c_ij = distance_matrix
        self.t_ij = time_matrix
        self.fixedKeys = []
        self.activeKeys = []
        self.key2Task = {}
        self.routes = []
        self.fixedRoutes = []
        self.routes_info = []
        self.infFactor = 1.1
        self.banList = []
        self.best_feas_sol = None
        self.best_feas_obj = None

    def add_empty_route(self, heads, tails, time_lbs, time_ubs):
        for head, tail, time_lb, time_ub in zip(heads, tails, time_lbs, time_ubs):
            self.routes.append([head, tail])
            self.fixedRoutes.append([])
            self.routes_info.append({'current_time': time_lb, 'time_ub': time_ub, 'current_load': 0, 'capacity': 200})

    def add_tasks(self, locations, start_times, end_times, demands, service_times):
        if self.fixedKeys:
            start_key = max(max(self.fixedKeys), self.c_ij.shape[0])
        else:
            start_key = self.c_ij.shape[0]

        for location, start_time, end_time, task_demand, service_time in zip(locations, start_times, end_times, demands,
                                                                             service_times):
            start_key = start_key + 1
            task = Task(location, start_time, end_time, task_demand, service_time)
            self.activeKeys.append(start_key)
            self.key2Task[start_key] = task

    def trans_key_to_station(self, key):
        if key < 0:
            return 0
        elif key < self.c_ij.shape[0]:
            return key
        else:
            return self.key2Task[key].location

    def build_initial_solution(self):
        unassigned_nodes = self.activeKeys.copy()
        add_move = None
        num_tour = 0
        sol = self.routes.copy()
        while num_tour < len(sol) and unassigned_nodes:
            insertion_allowed = True
            while insertion_allowed and unassigned_nodes:
                best_sol = float('inf')
                move = None
                for i in unassigned_nodes:
                    for j in range(1, len(sol[num_tour])):
                        sigma_1 = sol[num_tour][:j]
                        sigma_2 = sol[num_tour][j:]
                        obj = self.evaluate_solution([sigma_1 + [i] + sigma_2])
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
                            if sol[num_tour][j - 1] != satellite and sol[num_tour][j] != satellite:
                                sigma_1 = sol[num_tour][:j]
                                sigma_2 = sol[num_tour][j:]
                                add_obj = self.evaluate_solution([sigma_1 + [satellite] + sigma_2])
                                add_f = all(self.get_time_window_feasibility([sigma_1 + [satellite] + sigma_2], [num_tour]))
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
        self.routes = sol.copy()
        self.banList.append(self.evaluate_solution(self.routes))
        self.best_feas_sol = self.routes.copy()
        self.best_feas_obj = self.evaluate_solution(self.best_feas_sol)

    def evaluate_solution(self, routes):
        total_dis = 0
        for r in routes:
            for i in range(0, len(r) - 1):
                total_dis = total_dis + self.c_ij[self.trans_key_to_station(r[i]), self.trans_key_to_station(r[i + 1])]
        return total_dis

    def get_feasibility(self, routes, tour_id):
        return [all(f_col) for f_col in
                zip(self.get_load_feasibility(routes, tour_id), self.get_time_window_feasibility(routes, tour_id))]

    def get_time_feasibility(self, routes, tour_id):
        for i, r in enumerate(routes):
            period = self.routes_info[tour_id[i]]['time_ub']
            total_time = self.routes_info[tour_id[i]]['current_time']
            for j in range(0, len(r) - 1):
                total_time = total_time + self.t_ij[
                    self.trans_key_to_station(r[j]), self.trans_key_to_station(r[j + 1])]
            if total_time > period:
                return False
        return True

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
            for j, _ in enumerate(r):
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
        if task_key < self.c_ij.shape[0]:
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

    # 关于infFactor
    def reset_inf_factor(self, init_val=1.1):
        self.infFactor = init_val

    def update_inf_factor(self, symbol, inf_step_up=0.05, inf_step_down=0.02):
        if symbol > 0:
            self.infFactor = self.infFactor + inf_step_up
        else:
            self.infFactor = self.infFactor - inf_step_down
        if self.infFactor < 1:
            self.infFactor = 1

    def node_relocation(self):
        routes = self.routes.copy()
        S_sol = self.evaluate_solution(routes)
        r_feas = self.get_feasibility(routes, range(len(routes)))
        bestSol = float('inf')
        saveMove = None
        savef = False

        for i in range(len(routes)):
            route_i = routes[i]
            S_i = self.evaluate_solution([route_i])
            for j in range(len(routes)):
                route_j = routes[j]
                S_j = self.evaluate_solution([route_j])
                for k in range(1, len(route_i) - 1):
                    sigma_1 = route_i[0:k]
                    sigma_2 = route_i[k:k + 1]
                    sigma_3 = route_i[k + 1:]
                    for l in range(len(route_j) - 1):
                        sigma_4 = route_j[0:l + 1]
                        sigma_5 = route_j[l + 1:]
                        if i != j:
                            obj = self.evaluate_solution([sigma_1 + sigma_3])
                            obj = obj + self.evaluate_solution([sigma_4 + sigma_2 + sigma_5])
                            obj = obj + S_sol - S_i - S_j
                            r_feas_temp = r_feas.copy()
                            r_feas_update = self.get_feasibility([sigma_1 + sigma_3, sigma_4 + sigma_2 + sigma_5], [i, j])
                            r_feas_temp[i] = r_feas_update[0]
                            r_feas_temp[j] = r_feas_update[1]
                            f = all(r_feas_temp)
                        else:
                            if l < k:
                                sigma_5.remove(route_i[k])
                            else:
                                sigma_4.remove(route_i[k])
                            obj = self.evaluate_solution([sigma_4 + sigma_2 + sigma_5])
                            obj = obj + S_sol - S_i
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
                if l <= k:
                    sigma_5.remove(routes[i][k])
                else:
                    sigma_4.remove(routes[i][k])
                routes[i] = sigma_4 + sigma_2 + sigma_5
        return routes, savef, bestSol < S_sol

    def nodes_swap(self):
        routes = self.routes.copy()
        S_sol = self.evaluate_solution(routes)
        r_feas = self.get_feasibility(routes, range(len(routes)))
        bestSol = float('inf')
        saveMove = None
        savef = False

        for i in range(len(routes)):
            route_i = routes[i].copy()
            S_i = self.evaluate_solution([route_i])
            for j in range(i, len(routes)):
                route_j = routes[j].copy()
                S_j = self.evaluate_solution([route_j])
                for k in range(1, len(route_i) - 1):
                    for l in range(1, len(route_j) - 1):
                        if route_i[k] != route_j[l]:
                            route_i[k], route_j[l] = route_j[l], route_i[k]
                            obj = self.evaluate_solution([route_i])
                            obj = obj + self.evaluate_solution([route_j])
                            obj = obj + S_sol - S_i - S_j
                            r_feas_temp = r_feas.copy()
                            r_feas_update = self.get_feasibility([route_i, route_j], [i, j])
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
        if saveMove:
            i = saveMove[0]
            k = saveMove[1]
            j = saveMove[2]
            l = saveMove[3]
            routes[i][k], routes[j][l] = routes[j][l], routes[i][k]
        return routes, savef, bestSol < S_sol

    def intra_route_2opt(self):
        routes = self.routes.copy()
        r_feas = self.get_feasibility(routes, range(len(routes)))
        S_sol = self.evaluate_solution(routes)
        bestSol = float('inf')
        saveMove = None
        savef = False

        for i in range(len(routes)):
            route_i = routes[i]
            S_i = self.evaluate_solution([route_i])
            for k in range(1, len(route_i) - 3):
                for l in range(k + 2, len(route_i) - 1):  # 交换的是点k到点l,且把这种方式与swap区分开来了
                    sigma_1 = route_i[0:k]
                    sigma_2 = route_i[k:l + 1]
                    sigma_2.reverse()
                    sigma_3 = route_i[l + 1:]
                    obj = self.evaluate_solution([sigma_1 + sigma_2 + sigma_3])
                    obj = obj + S_sol - S_i
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
        if saveMove:
            i = saveMove[0]
            k = saveMove[1]
            l = saveMove[2]
            sigma_1 = routes[i][0:k]
            sigma_2 = routes[i][k:l + 1]
            sigma_2.reverse()
            sigma_3 = routes[i][l + 1:]
            routes[i] = sigma_1 + sigma_2 + sigma_3
        return routes, savef, bestSol < S_sol

    def inter_routes_2opt(self):
        routes = self.routes.copy()
        r_feas = self.get_feasibility(routes, range(len(routes)))
        S_sol = self.evaluate_solution(routes)
        bestSol = float('inf')
        saveMove = None
        savef = False

        for i in range(len(routes)):
            route_i = routes[i]
            S_i = self.evaluate_solution([route_i])
            for j in range(i + 1, len(routes)):
                route_j = routes[j]
                S_j = self.evaluate_solution([route_j])
                for k in range(len(route_i) - 1):
                    sigma_1 = route_i[0:k + 1]
                    sigma_2 = route_i[k + 1:]
                    for l in range(len(route_j) - 1):
                        sigma_3 = route_j[0:l + 1]
                        sigma_4 = route_j[l + 1:]
                        obj = self.evaluate_solution([sigma_1 + sigma_4])
                        obj = obj + self.evaluate_solution([sigma_3 + sigma_2])
                        obj = obj + S_sol - S_i - S_j
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
        return routes, savef, bestSol < S_sol

    def print_sol(self):
        for r in self.best_feas_sol:
            if len(r) > 2:
                print([self.trans_key_to_station(i) for i in r])

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
        self.best_feas_sol = self.routes.copy()

        for it in range(maxIter):
            ImpIter = False
            for nh in neighborhoods:
                NbNonImpIter = 0
                for Nbit in range(maxNbIter):
                    neighbor = self.get_neighborhood(nh)  # neighbor的组成为路径、是否可行、是否有所改善
                    self.routes = neighbor[0]
                    # update banlist
                    self.banList.append(self.evaluate_solution(self.routes))
                    # update infFactor
                    if neighbor[1]:
                        self.update_inf_factor(-1)
                    else:
                        self.update_inf_factor(1)
                    if neighbor[2]:
                        if neighbor[1]:
                            print('++',self.evaluate_solution(self.routes))
                            if self.evaluate_solution(self.routes) < self.best_feas_obj -0.1:
                                print('++++')
                                self.best_feas_sol = self.routes.copy()
                                self.best_feas_obj = self.evaluate_solution(self.best_feas_sol)
                            if all(self.get_feasibility(self.best_feas_sol, range(25))) is False:
                                print('+++', nh, neighbor[1])
                            ImpIter = True
                    else:
                        NbNonImpIter = NbNonImpIter + 1
                    if NbNonImpIter >= maxNbNonImpIter:
                        self.routes = self.best_feas_sol.copy()
                        self.reset_inf_factor()
                        break
                self.reset_inf_factor()
                self.routes = self.best_feas_sol.copy()
            if ImpIter is False:
                NonImpIter = NonImpIter + 1
            if NonImpIter >= maxNonImpIter:
                break


if __name__ == '__main__':
    print('Main Function')
