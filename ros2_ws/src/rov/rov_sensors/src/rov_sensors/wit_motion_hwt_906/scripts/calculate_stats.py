import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import os
import csv
from scipy.stats import norm  # Import for Gaussian distribution fitting

Path(__file__).parent
dir = Path(__file__).parent.parent
print(dir)
angular_position = []
angular_velocity = []
with open(os.path.join(dir, 'data', 'data__1.csv'), newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    print(reader.fieldnames)
    for row in reader:
        angular_velocity.append([float(row['Angular velocity X(°/s)']), float(row['Angular velocity Y(°/s)']), float(row['Angular velocity Z(°/s)'])])
        angular_position.append([float(row['Angle X(°)']), float(row['Angle Y(°)']), float(row['Angle Z(°)'])])

angular_velocity = np.array(angular_velocity)
angular_position = np.array(angular_position)
print(angular_velocity.shape)

plt.figure()

angular_position_x = angular_position[:, 0]
mean, std_dev = norm.fit(angular_position_x)
x = np.linspace(min(angular_position_x), max(angular_position_x), 100)
pdf = norm.pdf(x, mean, std_dev)

plt.hist(angular_position_x, bins=50, density=True, alpha=0.6, label='X Histogram')
plt.plot(x, pdf, 'k', linewidth=2, label=f'X Gaussian Fit\nMean: {mean:.2f}, Std Dev: {std_dev:.2f}')

angular_position_y = angular_position[:, 1]
mean, std_dev = norm.fit(angular_position_y)
x = np.linspace(min(angular_position_y), max(angular_position_y), 100)
pdf = norm.pdf(x, mean, std_dev)

plt.hist(angular_position_y, bins=50, density=True, alpha=0.6, label='Y Histogram')
plt.plot(x, pdf, 'k', linewidth=2, label=f'Y Gaussian Fit\nMean: {mean:.2f}, Std Dev: {std_dev:.2f}')

angular_position_z = angular_position[:, 2] - np.mean(angular_position[:, 2])
mean, std_dev = norm.fit(angular_position_z)
x = np.linspace(min(angular_position_z), max(angular_position_z), 100)
pdf = norm.pdf(x, mean, std_dev)

plt.hist(angular_position_z, bins=50, density=True, alpha=0.6, label='Z Histogram')
plt.plot(x, pdf, 'k', linewidth=2, label=f'Z Gaussian Fit\nMean: {mean:.2f}, Std Dev: {std_dev:.2f}')

plt.xlabel('°')
plt.ylabel('Density')
plt.legend()

plt.figure()
plt.plot(angular_position[:, 1])
plt.show()