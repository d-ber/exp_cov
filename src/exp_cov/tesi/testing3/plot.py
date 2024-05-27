import matplotlib.pyplot as plt 

time_deltas = [156, 172, 122, 287, 244, 68, 315, 200, 152, 179]
area_deltas = [1500, 10443, 973, 978, 2122, 972, 228, 957, 972, 856]
fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2, figsize=(9, 4))
# rectangular box plot
bplot1 = ax1.boxplot(time_deltas,vert=True, patch_artist=True)
ax1.set_title('time box plot')
bplot2 = ax2.boxplot(area_deltas,vert=True, patch_artist=True)
ax2.set_title('area box plot')
plt.show()
