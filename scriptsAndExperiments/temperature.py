import os, sys
sys.path.insert(0, os.getcwd())

import numpy as np
import random
from matplotlib import pyplot as plt

N = 5
X = np.array([400,900,390,1000,550])

alpha = np.min(X)
Xh = X / alpha
Xh = np.tile(Xh, [1, 1])
T = np.linspace(0.01, 5, 100)
EXP = np.tile(-1 / T, [N, 1]).transpose()

P = np.power(Xh, EXP)
denom = np.sum(P, axis=1)
P /= denom[:,None]


plt.plot(P)
axes = plt.gca()
axes.set_xticklabels(range(N+1))
axes.set_xlabel("T")
axes.set_ylabel("P")
axes.set_xlim([0,100])
axes.set_ylim([0,1])
plt.legend(X, loc="upper right")
plt.show()
