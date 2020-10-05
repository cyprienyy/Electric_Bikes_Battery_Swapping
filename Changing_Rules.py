from Stimulation import *
import numpy as np


class ChangingRules:
    def __init__(self):
        self.eventList = EventList()
        self.stations = [Station([20] * 5) for _ in range(10)]
        demands = np.load('demands.npy')
        demands[:, 4] = np.ceil(demands[:, 4] / 10) * 10
        self.demands = demands.tolist()
        self.charge_schedule = []

    def prepare_event_list(self):
        for d in self.demands:
            self.eventList.add_event(DemandEvent(d[2], d[0], d[3], d[1], d[4], d[5]))
        for c in self.charge_schedule:
            self.eventList.add_event(ChargeEvent(c[0], c[1], c[2], c[3]))

    def stimulate(self):
        while self.eventList.next_event() is not None:
            next_event = self.eventList.next_event()
            self.stations[next_event.startPosition - 1].process_event(next_event, self.eventList)
            self.eventList.remove_event()

    def get_station_info(self, lp, up):
        station_info = []
        for station in self.stations:
            time_labels = list(station.bikesRecord.keys())
            time_labels.sort()
            l_index = 0
            u_index = len(time_labels)
            for i, t in enumerate(time_labels):
                if t <= lp:
                    l_index = i
                if t <= up:
                    u_index = i
            time_labels = time_labels[l_index:u_index+1]
            if time_labels[-1] < up:
                time_labels.append(time_labels[-1])
            bikes_dist = [[station.bikesRecord[t][bl * 10] for bl in range(11)] for t in time_labels]
            loss = [station.loss[t] for t in time_labels]
            if time_labels[0] < lp:
                time_labels[0] = lp
            if time_labels[-1] < up:
                time_labels[-1] = up
            station_info.append([time_labels, bikes_dist, loss])
        return station_info


