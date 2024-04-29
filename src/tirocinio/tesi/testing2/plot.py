import matplotlib.pyplot as plt

time_deltas = [186, 334, 322, 341, 59, 486, 641, 290, 79, -26, 1056]
area_deltas = [-3496, -3291, -1975, 8944, 1372, 2774, 1378, -4221, -1899, -5418, 1792]
fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2, figsize=(9, 4))
# rectangular box plot
bplot1 = ax1.boxplot(time_deltas,vert=True, patch_artist=True)
ax1.set_title('time box plot')
bplot2 = ax2.boxplot(area_deltas,vert=True, patch_artist=True)
ax2.set_title('area box plot')
plt.show()
