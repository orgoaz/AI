from heuristics import Heuristic
from ways.tools import compute_distance

# TODO : Implement as explained in the instructions
class TSPCustomHeuristic(Heuristic):
    _roads = None

    # TODO : You can add parameters if you need them
    def __init__(self, roads, initialState):
        super().__init__()
        self._roads = roads

    # Estimate heuristically the minimal cost from the given state to the problem's goal
    def estimate(self, problem, state):
        idx = state.junctionIdx
        orders = problem.orders
        
        # h_e
        h_a = max(orders.map(lambda o: self._cost(o[0], o[1])))
        h_d = max(orders.map(lambda o: self._cost(idx, o[0])))
        return h_a + h_d

    def _cost(self, source, target):
        coord1 = self._roads[source].coordinates
        coord2 = self._roads[target].coordinates

        return compute_distance(coord1, coord2)