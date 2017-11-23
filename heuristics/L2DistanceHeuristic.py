from . import Heuristic
from ways.tools import compute_distance

# Use the L2 aerial distance (in meters)
class L2DistanceHeuristic(Heuristic):
    def estimate(self, problem, state):
        # TODO : Return the correct distance
        return compute_distance(problem.target, state)

