import matplotlib.pyplot as plt
import csv
x = []
y = []

with open("../files/gridmapbylaser.txt") as f:
    coords = csv.reader(f, delimiter='\t', quotechar='|')
    for line in coords:
        if int(line[2]) > 50:
            if line:
                x.append(float(line[0]))
                y.append(float(line[1]))
plt.plot(x, y, 'r.')


x_gt = []
y_gt = []
x_od = []
y_od = []

with open("../files/gt.txt") as f:
    coords = csv.reader(f, delimiter='\t', quotechar='|')
    for line in coords:
        if line:
            x_gt.append(float(line[0]))
            y_gt.append(float(line[1]))
            x_od.append(float(line[2]))
            y_od.append(float(line[3]))

plt.plot(x_gt, y_gt, 'b.')
plt.plot(x_od, y_od, 'g.')

plt.axis()
plt.show()
