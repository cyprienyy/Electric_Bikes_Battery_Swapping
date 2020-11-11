import numpy as np

station_inventory = []
station_num = 40
for i in range(station_num):
    percent = np.concatenate((np.random.randint(1, 6, 7), np.random.randint(1, 3, 4)))
    percent = percent/sum(percent)
    bikes_num = 25*percent
    bikes_num = np.around(bikes_num).astype(np.int32)
    station_inventory.append(bikes_num.tolist())
print(station_inventory)
np.save('station_inventory', station_inventory)
