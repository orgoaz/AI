from . import GreedySolver
import numpy as np

class GreedyStochasticSolver(GreedySolver):
    _TEMPERATURE_DECAY_FACTOR = None
    _INITIAL_TEMPERATURE = None
    _N = None
    _T = None

    def __init__(self, roads, astar, scorer, initialTemperature, temperatureDecayFactor, topNumToConsider):
        super().__init__(roads, astar, scorer)

        self._T = self._INITIAL_TEMPERATURE = initialTemperature
        self._TEMPERATURE_DECAY_FACTOR = temperatureDecayFactor
        self._N = topNumToConsider

    def _getSuccessorsProbabilities(self, currState, successors):
        # Get the scores
        X = np.array([self._scorer.compute(currState, target) for target in successors])
        N = min(len(X), self._N)

        inds = X.argsort()[:N]
        X = X[inds]

        # Initialize an all-zeros vector for the distribution
        P = np.zeros((len(successors),))

        # TODO: Fill the distribution in P as explained in the instructions.
        # TODO : No changes in the rest of the code are needed

        alpha = np.min(X)
        Xh = X / alpha
        EXP = np.tile(-1 / self._T, [N, 1]).transpose()

        P[inds] = np.power(Xh, EXP)
        denom = np.sum(P, axis=0)
        P /= denom

        # Update the temperature
        self._T *= self._TEMPERATURE_DECAY_FACTOR

        return P

    # Find the next state to develop
    def _getNextState(self, problem, currState):
        successors = list(problem.expand(currState))
        P = self._getSuccessorsProbabilities(currState, successors)

        # TODO : Choose the next state stochastically according to the calculated distribution.
        # You should look for a suitable function in npumpy.random.
        nextIdx = np.random.choice(len(successors), p=P)

        return successors[nextIdx]

    # Override the base solve method to initialize the temperature
    def solve(self, initialState):
        self._T = self._INITIAL_TEMPERATURE
        return super().solve(initialState)