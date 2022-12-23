
def create_data_model():
    """Stores the data for the problem."""

    data={}

    data['time_matrix']= [
        [0, 4, 9, 5, 100, 100 ],
        [4, 0, 5, 11, 100, 100],
        [9, 5, 0, 6, 6, 100],
        [5, 20, 20, 0, 10, 100],
        [100, 100, 10, 10, 0, 2],
        [100, 100, 6, 6, 100, 0],
    ]

    data['energy_matrix']= [
        [0, 4, 9, 5, 100, 100],
        [4, 0, 5, 11, 100, 100],
        [9, 5, 0, 6, 6, 100],
        [5, 11, 6, 0, 3, 100],
        [100, 100, 10, 10, 0, -30],
        [100, 100, 6, 6, 100, 0],
    ]



    data['time_windows'] = [
        (0, 0),  # depot
        (3, 20),  # 1
        (6, 20),  # 2
        (10, 20),  # 3
        (0, 40),  # 4
        (0,60)    #5
    ]

    data['num_vehicles'] = 1

    # battery capacities correspond to maximum energy consumption by the respective ev before it needs to recharge
    data['battery_capacities'] = [30]

    data['vehicle_max_time']= 200

    # list of evse locations from the initial matrix, used to generate the modified energy and time matrices with duplicate evse nodes
    data['evse_nodes'] = [4] #Which means that the evse is represented by nodes 8 and 9 in the modified matrix

    # data['demands'] = [0, 1, 1, 2, 4, 2, 4, 8, 8, 1, 2, 1, 2, 4, 4, 8, 8]

    data['depot'] = 0

    # [START initial_routes]
    data['initial_routes'] = [
        [1,2,4,5,3]
    ]
    # [END initial_routes]
    return data