import numpy as np
import matplotlib.pyplot as plt

forest_density = [0.1, 0.2, 0.3, 0.4]
p_values = np.arange(0, 1.01, 0.05)
colors = ["#0072B2", "#E69F00", "#009E73", "#CC79A7"]

forest_density_values = []

for fdx in forest_density:
    means = []
    minimums = []
    maximums = []

    for p in p_values:
        samples = np.random.binomial(n=int(fdx * 100), p=p, size=50)
        means.append(samples.mean())
        minimums.append(samples.min())
        maximums.append(samples.max())

    forest_density_values.append((means, minimums, maximums))

for fdx, (means, minimums, maximums), color in zip(
    forest_density, forest_density_values, colors
):
    plt.fill_between(
        p_values, minimums, maximums,
        color=color, alpha=0.2
    )
    plt.plot(
        p_values, means,
        color=color, linewidth=2.5, marker="o", markersize=5,
        label=f"{fdx} Trees/m²"
    )

plt.xlabel("Probability")
plt.ylabel("Tree Count")
plt.xlim(0, 1)
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()