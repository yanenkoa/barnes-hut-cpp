import csv
import matplotlib.pyplot as plt
import numpy as np


from_csv = []
with open("data/measurements.csv") as file:
    from_csv = list(csv.reader(file))

arr = np.array(from_csv)
line1, = plt.plot(arr[:, 0], arr[:, 1], label="Naive")
line2, = plt.plot(arr[:, 0], arr[:, 2], label="Barnes-Hut")

plt.xlabel("Размер задачи")
plt.ylabel("Время работы, с")

plt.legend(handles=[line1, line2])

plt.savefig("data/plot.png")
