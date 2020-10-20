import numpy as np

station_num = 10

distance_matrix = np.load('dm.npy')
distance_matrix = distance_matrix[1:station_num + 1, 1:station_num + 1]
speed_ev = 240
time_matrix = distance_matrix / speed_ev * 60
time_matrix = np.around(time_matrix, 0).astype(int)

# 通过这里改变站点间关系的强弱变化范围
relation_coeff = np.random.randint(1, 4, (station_num, station_num))
i = list(range(station_num))
relation_coeff[i, i] = 0

# 电量最低消耗设置为10%
battery_consumption = 10 + np.around((time_matrix - np.min(time_matrix[time_matrix > 0])) / (
            np.max(time_matrix) - np.min(time_matrix[time_matrix > 0])) * 20, 0).astype(int)
# 产生需求的时间上限
T = 3600
limit_t = T - np.max(time_matrix)

demands = []
# 通过这里设置时间内需求上限
max_num_d = 3
for i in range(len(distance_matrix)):
    lamda = T / max_num_d
    interval = np.around(np.random.exponential(lamda, max_num_d), 0).astype(int)
    t = 0
    j = 0
    while j < max_num_d:
        t = t + interval[j]
        j = j + 1
        k = np.random.choice(list(range(station_num)), p=relation_coeff[i] / sum(relation_coeff[i]))
        if t + time_matrix[i, k] <= T:
            # 同一趟骑行的需求最小为1， 最大为4
            demands.append(
                [i + 1, k + 1, t, t + time_matrix[i, k], battery_consumption[i, k], int(np.random.randint(1, 4, 1))])

demands = np.array(demands)
np.save('demands_1', demands)
