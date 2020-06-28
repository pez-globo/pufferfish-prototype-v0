cmd[0] == 2: actuate the currently selected valve, payload: 3 bytes - number of microsteps; command will only be executed when flag_valve_doing_cyclic_motion==false

cmd[0] == 3: close the selected valve, payload: 1 byte - selected valve to close; command will only be executed when flag_valve_doing_cyclic_motion==false

cmd[0] == 4: valve cycling (all the valves), payload: 1 byte - 1: start, 0: stop

cmd[0] == 5: set the current active valve (if cmd[1]==0) - cmd[2] sets the valve ID (for cmd[0]==2) or eneble the cycling of valve selection (if cmd[1]==1) - cmd[2-3] sets the duration for each valve
