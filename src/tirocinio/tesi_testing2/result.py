from math import e
import matplotlib.pyplot as plt
import os
import re
import pandas as pd

cov_times_tot = list()
exp_times_tot = list()
cov_areas_tot = list()
exp_areas_tot = list()
cov_dist_tot = list()
exp_dist_tot = list()

tot_ambienti = 0

directories = [d for d in os.listdir(os.getcwd()) if os.path.isdir(os.path.join(os.getcwd(), d))]
for i, dir_i in enumerate(directories):
    cov_times = list()
    exp_times = list()
    cov_areas = list()
    exp_areas = list()
    cov_dist = list()
    exp_dist = list()
    for root, dirs, files in os.walk(os.path.join(os.getcwd(), dir_i)):
        for d in dirs:
            d = os.path.join(root, d)
            files_in_dir = os.listdir(d)
            for f in files_in_dir:
                if os.stat(os.path.join(d,f)).st_size != 0 and os.path.basename(f) == "result.log":
                    f = os.path.join(d,f)
                    with open(f, "r") as file:
                        print(f)
                        line = file.readline()
                        nums = re.findall(r"\d+", line)
                        #print("cov time:", nums[6+0])
                        cov_times.append(int(nums[6+0]))
                        #print("exp time:", nums[6+1])
                        exp_times.append(int(nums[6+1]))
                        line = file.readline()
                        nums = re.findall(r"\d+", line)
                        #print("cov area:", nums[6+0])
                        cov_areas.append(int(nums[6+0]))
                        #print("exp area:", nums[6+1])
                        exp_areas.append(int(nums[6+1]))
                elif os.stat(os.path.join(d,f)).st_size != 0 and os.path.basename(f) == "explore.log" or os.path.basename(f) == "coverage.log" :
                    f = os.path.join(d,f)
                    with open(f, "r") as file:
                        print(f)
                        for j in range(3):
                            line = file.readline()
                        line = file.readline()
                        num = line.strip().split()[len(line.strip().split())-1]
                        if os.path.basename(f) == "explore.log":
                            exp_dist.append(float(num))
                        else:
                            cov_dist.append(float(num))
                    
    df_cov = pd.DataFrame(list(zip(cov_times, cov_areas, cov_dist)), columns =['Time', 'Area', "Distance"])
    df_exp = pd.DataFrame(list(zip(exp_times, exp_areas, exp_dist)), columns =['Time', 'Area', "Distance"])
    if len(df_cov) > 1:
        tot_ambienti += 1
    print(len(df_cov))
    df_cov.sort_values(by=['Time'])
    df_exp.sort_values(by=['Time'])

    fig, axs = plt.subplots(2, 2, figsize=(12, 12))
    axs[0,0].boxplot([df_cov["Distance"], df_exp["Distance"]])
    axs[0,0].set_xticklabels(['Coverage', 'Exploration'])
    axs[0,0].set_ylabel('Distance')
    axs[0,0].set_title(f"Distance Traveled Comparison {i}")

    axs[0,1].boxplot([df_cov["Time"], df_exp["Time"]])
    axs[0,1].set_xticklabels(['Coverage', 'Exploration'])
    axs[0,1].set_ylabel('Time')
    axs[0,1].set_title(f"Time Comparison {i}")

    axs[1,0].scatter(df_cov['Distance'].values, df_cov['Area'].values, marker='x', linestyle='-', label='Coverage')
    axs[1,0].scatter(df_exp['Distance'].values, df_exp['Area'].values, marker='+', linestyle='--', label='Exploration')
    axs[1,0].set_title(f"Distance Traveled VS Area Mapped {i}")
    axs[1,0].set_xlabel('Distance Traveled')
    axs[1,0].set_ylabel('Area')
    axs[1,0].grid(False)
    axs[1,0].legend()

    axs[1,1].scatter(df_cov['Time'].values, df_cov['Area'].values, marker='x', linestyle='-', label='Coverage')
    axs[1,1].scatter(df_exp['Time'].values, df_exp['Area'].values, marker='+', linestyle='--', label='Exploration')
    axs[1,1].set_title(f"Time vs Area {i}")
    axs[1,1].set_xlabel('Time')
    axs[1,1].set_ylabel('Area')
    axs[1,1].grid(False)
    axs[1,1].legend()
    plt.savefig(f"ambiente{i}.png")
    plt.show()
    i += 1 
    cov_times_tot += cov_times
    exp_times_tot += exp_times
    cov_areas_tot += cov_areas
    exp_areas_tot += exp_areas
    cov_dist_tot += cov_dist
    exp_dist_tot += exp_dist

df_cov = pd.DataFrame(list(zip(cov_times_tot, cov_areas_tot, cov_dist_tot)), columns =['Time', 'Area', "Distance"])
df_exp = pd.DataFrame(list(zip(exp_times_tot, exp_areas_tot, exp_dist_tot)), columns =['Time', 'Area', "Distance"])
print(f"Eseguite {len(df_cov)} run in {tot_ambienti} ambienti")


fig, axs = plt.subplots(2, 2, figsize=(12, 12))
axs[0,0].boxplot([df_cov["Distance"], df_exp["Distance"]])
axs[0,0].set_xticklabels(['Coverage', 'Exploration'])
axs[0,0].set_ylabel('Distance')
axs[0,0].set_title(f"Distance Traveled Comparison Totale")

axs[0,1].boxplot([df_cov["Time"], df_exp["Time"]])
axs[0,1].set_xticklabels(['Coverage', 'Exploration'])
axs[0,1].set_ylabel('Time')
axs[0,1].set_title(f"Time Comparison Totale")

axs[1,0].scatter(df_cov['Distance'].values, df_cov['Area'].values, marker='o', linestyle='-', label='Coverage')
axs[1,0].scatter(df_exp['Distance'].values, df_exp['Area'].values, marker='s', linestyle='--', label='Exploration')
axs[1,0].set_title(f"Distance Traveled VS Area Mapped")
axs[1,0].set_xlabel('Distance Traveled')
axs[1,0].set_ylabel('Area')
axs[1,0].grid(False)
axs[1,0].legend()

axs[1,1].scatter(df_cov['Time'].values, df_cov['Area'].values, marker='o', linestyle='-', label='Coverage')
axs[1,1].scatter(df_exp['Time'].values, df_exp['Area'].values, marker='s', linestyle='--', label='Exploration')
axs[1,1].set_title(f"Time vs Area")
axs[1,1].set_xlabel('Time')
axs[1,1].set_ylabel('Area')
axs[1,1].grid(False)
axs[1,1].legend()

plt.savefig(f"generale.png")
plt.show()

fig, axs = plt.subplots(2, 2, figsize=(12, 12))
axs[0,0].boxplot([df_cov["Distance"], df_exp["Distance"]])
axs[0,0].set_xticklabels(['Coverage', 'Exploration'])
axs[0,0].set_ylabel('Distance')
axs[0,0].set_title(f"Distance Traveled Comparison Totale")
axs[0,0].set_ylim(0, max(max(df_cov['Distance']), max(df_exp['Distance'])) * 1.1)

axs[0,1].boxplot([df_cov["Time"], df_exp["Time"]])
axs[0,1].set_xticklabels(['Coverage', 'Exploration'])
axs[0,1].set_ylabel('Time')
axs[0,1].set_title(f"Time Comparison Totale")
axs[0,1].set_ylim(0, max(max(df_cov['Time']), max(df_exp['Time'])) * 1.1)

axs[1,0].scatter(df_cov['Distance'].values, df_cov['Area'].values, marker='o', linestyle='-', label='Coverage')
axs[1,0].scatter(df_exp['Distance'].values, df_exp['Area'].values, marker='s', linestyle='--', label='Exploration')
axs[1,0].set_title(f"Distance Traveled VS Area Mapped")
axs[1,0].set_xlabel('Distance Traveled')
axs[1,0].set_ylabel('Area')
axs[1,0].grid(False)
axs[1,0].set_xlim(0, max(max(df_cov['Distance']), max(df_exp['Distance'])) * 1.1)
axs[1,0].set_ylim(0,  max(max(df_cov['Area']), max(df_exp['Area'])) * 1.1)
axs[1,0].legend()

axs[1,1].scatter(df_cov['Time'].values, df_cov['Area'].values, marker='o', linestyle='-', label='Coverage')
axs[1,1].scatter(df_exp['Time'].values, df_exp['Area'].values, marker='s', linestyle='--', label='Exploration')
axs[1,1].set_title(f"Time vs Area")
axs[1,1].set_xlabel('Time')
axs[1,1].set_ylabel('Area')
axs[1,1].grid(False)
axs[1,1].set_xlim(0, max(max(df_cov['Time']), max(df_exp['Time'])) * 1.1)
axs[1,1].set_ylim(0,  max(max(df_cov['Area']), max(df_exp['Area'])) * 1.1)
axs[1,1].legend()
plt.savefig(f"generale_zero_based.png")
plt.show()


exit()

# plot
fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2, figsize=(9, 4))
# rectangular box plot
p1 = ax1.scatter(cov_times, cov_areas, linewidth=2.0)
ax1.set_title('cov')
p2 = ax2.scatter(exp_times, exp_areas, linewidth=2.0)
ax2.set_title('exp')
plt.show()
exit()

#1:
time_deltas = [53, 75, 255, 116, -15, 248, 337, 69, 103, 401]
area_deltas = [-338, 1052, 7750, -5277, -10287, -1158, 1142, 974, 10963, 7228]
#2
time_deltas += [186, 334, 322, 341, 59, 486, 641, 290, 79, -26, 1056]
area_deltas += [-3496, -3291, -1975, 8944, 1372, 2774, 1378, -4221, -1899, -5418, 1792]
#3:
time_deltas = [156, 172, 122, 287, 244, 68, 315, 200, 152, 179]
area_deltas += [1500, 10443, 973, 978, 2122, 972, 228, 957, 972, 856]
#4:
time_deltas += [-22, 337, 26, 91, 37, 65, 187, 105, -93, 313]
area_deltas += [8158, 8302, -3649, 1974, 8623, 8123, 431, 1943, -3028, 4178]
#5:
time_deltas += [66, 155, 39, 266, 260, 451, 142, 172, 115, -199]
area_deltas += [7599, 10339, 1322, 1168, 1342, 817, 1171, 1516, 1330, -3462]
#6
time_deltas += [318, 303, 104, 88, 287, 376, -81, 569, 404, 495]
area_deltas += [2833, 6940, 4787, 6612, 699, 6814, 604, 10395, 710, 7801]
#7
time_deltas += [-21, 304, 332, 369, 251, 346, 286, 347, 234, 293]
area_deltas += [2210, 1463, 7998, 7348, 1067, 1646, 1143, 7367, 6981, 7836]
#8
time_deltas += [409, -15, -211, 42, 95, 196, 424, 226, 170, -146]
area_deltas += [-1916, -1523, -8448, -371, 6548, 8060, 167, 5265, 7933, -3434]
#9
time_deltas += [739, 388, 328, 407, 613, 676, 331, 166, 16, 486]
area_deltas += [1881, 7621, 7255, 7228, 6994, 6836, 7333, 7379, 334, 924]
#10
time_deltas += [244, 80, 207, -4, 273, 251, 285, 213, 216, 308]
area_deltas += [12020, 3009, 8921, 7466, 8462, 3102, 8101, 1446, 1434, -113]

fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2, figsize=(9, 4))
# rectangular box plot
bplot1 = ax1.boxplot(time_deltas,vert=True, patch_artist=True)
ax1.set_title('time box plot (seconds)')
bplot2 = ax2.boxplot(area_deltas,vert=True, patch_artist=True)
ax2.set_title('area box plot (map unit, 0.05 m)')
plt.show()


