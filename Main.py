from Changing_Rules import ChangingRules


changingRules = ChangingRules()
changingRules.prepare_event_list()
changingRules.stimulate()
print(changingRules.get_station_info(0, 3600))