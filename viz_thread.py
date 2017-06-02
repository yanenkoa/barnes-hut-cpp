import csv
import matplotlib.pyplot as plt
import numpy as np


from_csv = []
with open("data/measurements_parallel.csv") as file:
    from_csv = list(csv.reader(file))

arr = np.array(from_csv)
line1, = plt.plot(arr[:, 0], arr[:, 1], label="1 поток")
line2, = plt.plot(arr[:, 0], arr[:, 2], label="2 потока")
line3, = plt.plot(arr[:, 0], arr[:, 3], label="3 потока")
line4, = plt.plot(arr[:, 0], arr[:, 4], label="4 потока")

plt.xlabel("Размер задачи")
plt.ylabel("Время работы, с")

plt.legend(handles=[line1, line2, line3, line4])

plt.savefig("data/plot_parallel.png")
