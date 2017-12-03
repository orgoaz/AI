import os, sys
sys.path.insert(0, os.getcwd())

from consts import Consts
from astar import AStar
from ways import load_map_from_csv
from busSolvers import GreedyBestFirstSolver, GreedyStochasticSolver
from problems import BusProblem
from costs import L2DistanceCost
from heuristics import L2DistanceHeuristic
import numpy as np

REPEATS = 150

# Load the files
roads = load_map_from_csv(Consts.getDataFilePath("israel.csv"))
prob = BusProblem.load(Consts.getDataFilePath("HAIFA_100.in"))

mapAstar = AStar(L2DistanceHeuristic(), shouldCache=True)

scorer = L2DistanceCost(roads)

# Run the greedy solver
pickingPath = GreedyBestFirstSolver(roads, mapAstar, scorer).solve(prob)
greedyDistance = pickingPath.getDistance() / 1000
print("Greedy solution: {:.2f}km".format(greedyDistance))

# Run the stochastic solver #REPATS times
solver = GreedyStochasticSolver(roads, mapAstar, scorer,
                                Consts.STOCH_INITIAL_TEMPERATURE,
                                Consts.STOCH_TEMPERATURE_DECAY_FUNCTION,
                                Consts.STOCH_TOP_SCORES_TO_CONSIDER)
results = np.zeros((REPEATS,))
print("Stochastic repeats:")
for i in range(REPEATS):
    print("{}..".format(i+1), end=" ", flush=True)
    temp=solver.solve(prob).getDistance() / 1000
    if i == 0  or temp <= results[i -1]:
        results[i] = temp
    else:
        results[i]=results[i-1]

print("\nDone!")

# TODO : Part1 - Plot the diagram required in the instructions
from matplotlib import pyplot as plt
plt.plot(results)
axes = plt.gca()
axes.set_xlabel("iteration")
axes.set_ylabel("distance")
axes.set_xlim([1,len(results)+1])
axes.set_ylim([np.min(results),np.max(results)])
axes.set_xticks(range(1,len(results)+1))
plt.show()


# TODO : Part2 - Remove the exit and perform the t-test
raise NotImplementedError