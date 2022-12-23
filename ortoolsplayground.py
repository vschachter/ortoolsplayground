# OR_tools playground



from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from functools import partial
import evrp_data2 as evrpd


"""Electrical Vehicles Routing Problem (VRP) with Time Windows"""



###########################
# Problem Data Definition #
###########################

"""Stores the data for the problem"""
def create_data_model():
    """Stores the data for the problem"""
    return evrpd.create_data_model()


#######################
# Problem Constraints #
#######################


# Create and register a time evaluator

def time_evaluator(manager, data, from_index, to_index):
    """Returns the travel time between the two nodes."""
    # Convert from routing variable Index to time matrix NodeIndex.
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data['time_matrix'][from_node][to_node]



def add_time_window_constraints(routing, manager, data, time_evaluator):
    """Add Time windows constraint"""
    time = 'Time'
    max_time = data['vehicle_max_time']
    routing.AddDimension(
        time_evaluator,
        30,  # allow waiting time
        max_time,  # maximum time per vehicle
        False,  # don't force start cumul to zero since we are giving TW to start nodes
        time)
    time_dimension = routing.GetDimensionOrDie(time)
    time_dimension.SetGlobalSpanCostCoefficient(1000)
    # Add time window constraints for each location except depot
    # and 'copy' the slack var in the solution object (aka Assignment) to print it
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == 0:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        routing.AddToAssignment(time_dimension.SlackVar(index))
    # Add time window constraints for each vehicle start node
    # and 'copy' the slack var in the solution object (aka Assignment) to print it
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data['time_windows'][0][0],
                                                data['time_windows'][0][1])
        routing.AddToAssignment(time_dimension.SlackVar(index))
        # Warning: Slack var is not defined for vehicle's end node
        #routing.AddToAssignment(time_dimension.SlackVar(self.routing.End(vehicle_id)))

# # Add Time Windows constraint.
# time = 'Time'
# routing.AddDimension(
#     time_transit_callback_index,
#     30,  # allow waiting time
#     30,  # maximum time per vehicle
#     False,  # Don't force start cumul to zero.
#     time)
# time_dimension = routing.GetDimensionOrDie(time)

   # Create and register an energy consumption transit callback.

def energy_evaluator(manager, data, from_index, to_index):
    """Returns the energy consumed by the travel between the two nodes."""
    # Convert from routing variable Index to energy matrix NodeIndex.
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data['energy_matrix'][from_node][to_node]



def add_energy_dimension(routing, manager, data, energy_transit_callback_index):
    # Add energy consumption & battery charging
    energy_tracker = 'Energy'
    routing.AddDimension(energy_transit_callback_index,
                                                      max(data['battery_capacities']), # max capacity slack is equal to max vehicle capacity
                                                      # 0,
                                                      max(data['battery_capacities']),  # vehicle maximum energy per route
                                                      True,  # start cumul to zero
                                                      energy_tracker)
    energy_dimension = routing.GetDimensionOrDie(energy_tracker)
    energy_dimension.SetGlobalSpanCostCoefficient(10000)



###########
# Print & store #
###########

def get_cumul_data(solution, routing, dimension):
    """Get cumulative data from a dimension and store it in an array."""
    # Returns an array cumul_data whose i,j entry contains the minimum and
    # maximum of CumulVar for the dimension at the jth node on route :
    # - cumul_data[i][j][0] is the minimum.
    # - cumul_data[i][j][1] is the maximum.

    cumul_data = []
    for route_nbr in range(routing.vehicles()):
        route_data = []
        index = routing.Start(route_nbr)
        dim_var = dimension.CumulVar(index)
        route_data.append([solution.Min(dim_var), solution.Max(dim_var)])
        while not routing.IsEnd(index):
            index = solution.Value(routing.NextVar(index))
            dim_var = dimension.CumulVar(index)
            route_data.append([solution.Min(dim_var), solution.Max(dim_var)])
        cumul_data.append(route_data)
    return cumul_data


def print_solution(routes, cumul_data):
    """Print the solution."""
    total_time = 0
    route_str = ''
    for i, route in enumerate(routes):
        route_str += 'Route ' + str(i) + ':\n'
        start_time = cumul_data[i][0][0]
        end_time = cumul_data[i][0][1]
        route_str += '  ' + str(route[0]) + \
                     ' Time(' + str(start_time) + ', ' + str(end_time) + ')'
        for j in range(1, len(route)):
            start_time = cumul_data[i][j][0]
            end_time = cumul_data[i][j][1]
            route_str += ' -> ' + str(route[j]) + \
                         ' Time(' + str(start_time) + ', ' + str(end_time) + ')'
        route_str += '\n  Route time: {} min\n\n'.format(start_time)
        total_time += cumul_data[i][len(route) - 1][0]
    route_str += 'Total time: {} min'.format(total_time)
    print(route_str)


# Print solution with time and energy consumption
# Note that this prints the energy consumption at the end of routes
# which is not the same as energy consumption if the vehicle got recharged

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    time_dimension = routing.GetDimensionOrDie('Time')
    energy_dimension = routing.GetDimensionOrDie('Energy')
    total_time = 0
    sum_of_times = 0
    total_energy = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            energy_var = energy_dimension.CumulVar(index)
            plan_output += '{0} Time({1},{2}) Energy({3},{4}) -> '.format(manager.IndexToNode(index),
                                                                        solution.Min(time_var), solution.Max(time_var),
                                                                        solution.Min(energy_var), solution.Max(energy_var))
            index = solution.Value(routing.NextVar(index))
        time_var = time_dimension.CumulVar(index)
        energy_var = energy_dimension.CumulVar(index)
        plan_output += '{0} Time({1},{2}) Energy({3},{4})\n'.format(manager.IndexToNode(index),
                                                    solution.Min(time_var), solution.Max(time_var),
                                                    solution.Min(energy_var), solution.Max(energy_var))
        plan_output += 'Time of the route: {}min\n'.format(solution.Min(time_var))
        plan_output += 'Net energy spent: {} energy units\n'.format(solution.Min(energy_var))
        print(plan_output)
        total_time = max(total_time, solution.Min(time_var))
        sum_of_times += solution.Min(time_var)
        total_energy += solution.Min(energy_var)
    print('Total time of all routes: {} min'.format(total_time))
    print('Total energy of all routes: {} energy units'.format(total_energy))


def ormain():
    """Solve the VRP with time windows."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.

    # Set model parameters
    model_parameters = pywrapcp.DefaultRoutingModelParameters()


    # Make the routing model instance.
    routing = pywrapcp.RoutingModel(manager, model_parameters)


    # Add time constraints
    time_evaluator_index = routing.RegisterTransitCallback(partial(time_evaluator, manager,data))
    add_time_window_constraints(routing, manager, data, time_evaluator_index)

    # Add energy constraints
    energy_evaluator_index = routing.RegisterTransitCallback(partial(energy_evaluator, manager,data))
    add_energy_dimension(routing, manager, data, energy_evaluator_index)


    # Define cost of each arc.
    # For now the cost is only dependent on time
    routing.SetArcCostEvaluatorOfAllVehicles(time_evaluator_index)



    # Make some evse nodes optional
    penalty=500000
    routing.AddDisjunction([manager.NodeToIndex(4)], penalty)
    routing.AddDisjunction([manager.NodeToIndex(5)], penalty)
    # routing.AddDisjunction([manager.NodeToIndex(8)], penalty)
    # routing.AddDisjunction([manager.NodeToIndex(9)], penalty)
    #
    # routing.AddDisjunction([manager.NodeToIndex(3)], penalty)
    # routing.AddDisjunction([manager.NodeToIndex(4)], penalty)
    # routing.AddDisjunction([manager.NodeToIndex(7)], penalty)


    # Link evse nodes: if a vehicle goes to neither or both
    # evse_start_index = manager.NodeToIndex(8)
    # evse_end_index = manager.NodeToIndex(9)
    # routing.solver().Add(routing.VehicleVar(evse_start_index) == routing.VehicleVar(evse_end_index))


   # [START print_initial_solution]
   #  initial_solution = routing.ReadAssignmentFromRoutes(data['initial_routes'],True)
   #  print('Initial solution:')
   #  print_solution(data, manager, routing, initial_solution)
   #  print('\n')


    # Setting search parameters

    parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Setting first solution heuristic (cheapest addition).
    parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC
    parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    # Routing: forbids use of TSPOpt neighborhood, (this is the default behaviour)
    # parameters.local_search_operators.use_tsp_opt = pywrapcp.BOOL_FALSE
    # Disabling Large Neighborhood Search, (this is the default behaviour)
    # parameters.local_search_operators.use_path_lns = pywrapcp.BOOL_FALSE
    # parameters.local_search_operators.use_inactive_lns = pywrapcp.BOOL_FALSE

    parameters.time_limit.seconds = 5
    parameters.use_full_propagation = True
    parameters.log_search = True

    # The solver parameters can be accessed from the model parameters. For example :
    model_parameters.solver_parameters.CopyFrom(pywrapcp.Solver.DefaultSolverParameters())
    model_parameters.solver_parameters.trace_propagation = True



    # Solve the problem with or without an initial solution

    solution = routing.SolveWithParameters(parameters)
    # solution = routing.SolveFromAssignmentWithParameters(initial_solution, parameters)
    #
    # Print solution on console.
    if solution:
        # cumul_data = get_cumul_data(solution, routing, time_dimension)
        print_solution(data, manager, routing, solution)
        # print(cumul_data)


if __name__ == '__ormain__':
    ormain()
