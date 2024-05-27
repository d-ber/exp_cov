import matplotlib.pyplot as plt

time_deltas = [53, 75, 255, 116, -15, 248, 337, 69, 103, 401]
area_deltas = [-338, 1052, 7750, -5277, -10287, -1158, 1142, 974, 10963, 7228]
fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2, figsize=(9, 4))
# rectangular box plot
bplot1 = ax1.boxplot(time_deltas,vert=True, patch_artist=True)
ax1.set_title('time box plot')
bplot2 = ax2.boxplot(area_deltas,vert=True, patch_artist=True)
ax2.set_title('area box plot')
plt.show()
