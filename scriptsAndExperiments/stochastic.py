import os, sys
from scipy import stats
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
minVal = greedyDistance
print("Stochastic repeats:")
for i in range(REPEATS):
    print("{}..".format(i+1), end=" ", flush=True)
    results[i] = solver.solve(prob).getDistance() / 1000

print("\nDone!")
greedyDistance_line = [greedyDistance for i in range(REPEATS)]
# TODO : Part1 - Plot the diagram required in the instructions
from matplotlib import pyplot as plt
monoton_reults = np.minimum.accumulate(results)
plt.plot(monoton_reults)
plt.plot(greedyDistance_line)
plt.title("Stochastic Greedy VS Deterministic Greedy")
axes = plt.gca()
plt.legend(['Stochastic Greedy','Deterministic Greedy'], loc=0)
axes.set_xlabel("iteration")
axes.set_ylabel("distance")
axes.set_xticks(range(0,REPEATS+1, int(REPEATS/5)))
axes.grid(True)
plt.show()


# TODO : Part2 - Remove the exit and perform the t-test
mean = np.mean(results)
sd = np.std(results)
p_val = stats.ttest_1samp(results,greedyDistance)
print(" The mean value is ", mean)
print("The standard deviation is ", sd)
print("The p-value is ", p_val.pvalue)