import numpy as np
import sys

class AStar:
    cost = None
    heuristic = None
    _cache = None
    shouldCache = None

    def __init__(self, heuristic, cost=None, shouldCache=False):
        self.heuristic = heuristic
        self.shouldCache = shouldCache
        self.cost = cost

        # Handles the cache. No reason to change this code.
        if self.shouldCache:
            self._cache = {}

    # Get's from the cache. No reason to change this code.
    def _getFromCache(self, problem):
        if self.shouldCache:
            return self._cache.get(problem)

        return None

    # Get's from the cache. No reason to change this code.
    def _storeInCache(self, problem, value):
        if not self.shouldCache:
            return

        self._cache[problem] = value

    # Run A*
    def run(self, problem):
        # Check if we already have this problem in the cache.
        # No reason to change this code.
        source = problem.initialState
        if self.shouldCache:
            res = self._getFromCache(problem)

            if res is not None:
                return res

        # Initializes the required sets
        closed_set = set()  # The set of nodes already evaluated.
        parents = {}  # The map of navigated nodes.

        # Save the g_score and f_score for the open nodes
        g_score = {source: 0}
        open_set = {source: self.heuristic.estimate(problem, problem.initialState)}

        developed = 0
        while open_set != {}:
            next = self._getOpenStateWithLowest_f_score(open_set)[0]
            closed_set.add(next)
            if problem.isGoal(next):
                result = (self._reconstructPath(parents, next) , g_score[next], source, developed)
                self._storeInCache(problem, result)
                return result
            for s,c in problem.expandWithCosts(next, self.cost):
                developed += 1
                new_g = g_score[next] + c
                new_h = self.heuristic.estimate(problem, s)
                new_f = new_g + new_h
                old_node = s if open_set.__contains__(s) else None

                if old_node:
                    if new_g < g_score[old_node]:
                        g_score[old_node] = new_g
                        parents[old_node] = next
                        open_set[old_node] = new_f
                else:
                    old_node = s if closed_set.__contains__(s) else None
                    if old_node:
                        if new_g <  g_score[old_node]:
                            g_score[old_node] = new_g
                            parents[old_node] = next
                            open_set[old_node] = new_f
                            
                            closed_set.remove(old_node)
                    else:
                        g_score[s] = new_g
                        parents[s] = next
                        open_set[s] = new_f

        raise ValueError('No Path Found - I worked for nothing :(')

        # - Don't forget to cache your result between returning it - TODO

    def _getOpenStateWithLowest_f_score(self, open_set):
        stateWithLowestF = min(open_set.items(), key=lambda x: x[1])
        open_set.pop(stateWithLowestF[0])
        return stateWithLowestF

    # Reconstruct the path from a given goal by its parent and so on
    def _reconstructPath(self, parents, goal):
        path = []
        currentStep = goal

        while currentStep is not None:
            path.insert(0, currentStep)
            currentStep = parents.get(currentStep)

        return path
