import random
from collections import Counter, defaultdict
import numpy as np
import matplotlib.pyplot as plt

BATTERY_LEVEL = list(range(0, 110, 10))


class Event:
    def __init__(self, start_time, start_position):
        self.startTime = start_time
        self.startPosition = start_position
        self.finished = False

    def __eq__(self, other):
        return self.startTime == other.startTime

    def __gt__(self, other):
        return self.startTime > other.startTime


class DemandEvent(Event):
    def __init__(self, start_time, start_position, end_time, end_position, battery_demand, num_trips):
        Event.__init__(self, start_time, start_position)
        self.endTime = end_time
        self.endPosition = end_position
        self.batteryDemand = battery_demand
        self.numTrips = num_trips


class ArrivalEvent(Event):
    def __init__(self, start_time, start_position, battery_surplus, num_trips):
        Event.__init__(self, start_time, start_position)
        self.batterySurplus = battery_surplus
        self.numTrips = num_trips


class ChargeEvent(Event):
    def __init__(self, start_time, start_position, end_time, battery_available):
        Event.__init__(self, start_time, start_position)
        self.endTime = end_time
        self.batteryAvailable = battery_available


class EventList:
    def __init__(self):
        self.listedEvents = []

    def add_event(self, event):
        self.listedEvents.append(event)
        self.listedEvents.sort()

    def prt(self):
        for event in self.listedEvents:
            print(event.startTime, event.startPosition)

    def next_event(self):
        if self.listedEvents:
            return self.listedEvents[0]
        else:
            return None

    def remove_event(self):
        del self.listedEvents[0]


class Station:
    def __init__(self, num_bikes):
        """
        :type self.num_bikes: Counter or List
        """
        self.numBikes = Counter(num_bikes)
        self.loss = Counter()
        self.onsiteVehicles = Counter()
        self.bikesRecord = defaultdict(Counter)
        self.record(0)

    def process_event(self, undefined_event, event_list):
        if type(undefined_event) == DemandEvent:
            self.process_demand_event(undefined_event, event_list)
        elif type(undefined_event) == ChargeEvent:
            self.process_charge_event(undefined_event)
        elif type(undefined_event) == ArrivalEvent:
            self.process_arrival_event(undefined_event)
        self.record(undefined_event.startTime)

    def process_demand_event(self, demand_event, event_list):
        """
        :type event_list: EventList
        :type demand_event: DemandEvent
        """
        bikes_to_choose = []
        for bl, num in self.numBikes.items():
            if bl >= demand_event.batteryDemand:
                bikes_to_choose.extend([bl] * num)
        if len(bikes_to_choose) < demand_event.numTrips:
            self.loss[demand_event.startTime] += demand_event.numTrips - len(bikes_to_choose)
            bikes_counter = Counter(bikes_to_choose)
            for bl, num in bikes_counter.items():
                event_list.add_event(
                    ArrivalEvent(demand_event.endTime, demand_event.endPosition, bl - demand_event.batteryDemand, num))
            self.numBikes.subtract(bikes_counter)
        else:
            bikes_chosen = random.sample(bikes_to_choose, demand_event.numTrips)
            bikes_counter = Counter(bikes_chosen)
            for bl, num in bikes_counter.items():
                event_list.add_event(
                    ArrivalEvent(demand_event.endTime, demand_event.endPosition, bl - demand_event.batteryDemand, num))
            self.numBikes.subtract(bikes_counter)

    def process_charge_event(self, charge_event):
        """
        :type charge_event: ChargeEvent
        """
        self.onsiteVehicles[charge_event.endTime] += charge_event.batteryAvailable
        self.change_batteries_of_bikes(charge_event.startTime)

    def process_arrival_event(self, arrival_event):
        """
        :type arrival_event: ArrivalEvent
        """
        self.numBikes[arrival_event.batterySurplus] += arrival_event.numTrips
        self.change_batteries_of_bikes(arrival_event.startTime)

    def change_batteries_of_bikes(self, current_time):
        bikes = list(self.numBikes.elements())
        out_bikes = [x for x in bikes if x <= 60]
        batteries = list(self.onsiteVehicles.elements())
        available_batteries = [x for x in batteries if x >= current_time]
        out_bikes.sort()
        available_batteries.sort()
        num_change = min(len(out_bikes), len(available_batteries))
        battery_counter = Counter(available_batteries[0:num_change])
        bikes_counter = Counter(out_bikes[0:num_change])
        self.numBikes.subtract(bikes_counter)
        self.onsiteVehicles.subtract(battery_counter)
        self.numBikes[BATTERY_LEVEL[-1]] += num_change

    def record(self, current_time):
        self.bikesRecord[current_time] = Counter(self.numBikes)

    def show_record(self):
        time_labels = list(self.bikesRecord.keys())
        time_labels.sort()
        bikes_dist = [[self.bikesRecord[t][bl] for bl in BATTERY_LEVEL] for t in time_labels]
        bikes_dist = np.array(bikes_dist)
        plt_label = -10
        plt.rcParams['font.sans-serif'] = ['SimHei']
        # marks = ['-o', '-s', '-^', '-p', '-^', '-v', '-p', '-d', '-h', '-8', '-2']
        fig, axes = plt.subplots(2, 5, sharey=True)
        for i in range(2):
            for j in range(5):
                _ax = axes[i, j]
                # _ax.set_xticks(time_labels)
                plt_label += 10
                # plt.step(time_labels, bikes_dist[:, i], marks[i], label=str(plt_label) + '%电量车辆')
                _ax.step(time_labels, bikes_dist[:, j + 5 * i], label=str(plt_label) + '%电量车辆')
                _ax.legend()
        plt.show()

    def clear_state(self):
        self.loss = Counter()
        self.onsiteVehicles = Counter()
        self.numBikes = Counter(self.bikesRecord[0])
        self.bikesRecord = defaultdict(Counter)
        self.record(0)

    def count_loss(self):
        return sum(self.loss.values())


if __name__ == '__main__':
    print('Stimulation Main')
    '''
    eventList = EventList()
    Stations = [Station([20] * 5) for _ in range(10)]
    demands = np.load('demands.npy')
    demands[:, 4] = np.ceil(demands[:, 4] / 10) * 10
    demands = demands.tolist()
    for d in demands:
        eventList.add_event(DemandEvent(d[2], d[0], d[3], d[1], d[4], d[5]))
    while eventList.next_event() is not None:
        nextEvent = eventList.next_event()
        Stations[nextEvent.startPosition - 1].process_event(nextEvent, eventList)
        eventList.remove_event()
    Stations[5].show_record()
    numBikes = [20, 10, 20, 60]
    station = Station(numBikes)
    eventList = EventList()
    # demandEvent = DemandEvent(50, '1', 100, '2', 20, 4)
    chargeEvent = ChargeEvent(50, '1', 100, 1)
    chargeEvent_2 = ChargeEvent(60, '1', 70, 1)
    arrivalEvent = ArrivalEvent(80, '1', 60, 2)
    # station.process_demand_event(demandEvent, eventList)
    station.process_event(chargeEvent, eventList)
    station.process_event(chargeEvent_2, eventList)
    station.process_event(arrivalEvent, eventList)
    print(station.numBikes)
    print(station.onsiteVehicles)
    print(station.bikesRecord)
    print(station.numBikes[100])
    arrivalEvent = ArrivalEvent(100, '4', 7)
    chargeEvent = ChargeEvent(110, '3', 120, 5)
    eventList = EventList()
    eventList.add_event(demandEvent)
    eventList.add_event(arrivalEvent)
    eventList.add_event(chargeEvent)
    eventList.prt()
    print(eventList.next_event())
    eventList.remove_event()
    print(eventList.next_event())
    '''
