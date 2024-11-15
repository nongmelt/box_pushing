import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

left = pd.read_csv("data/box_push_left_baseline_1.csv", sep=",")

right = pd.read_csv("data/box_push_right_baseline_1.csv", sep=",")

plt.figure(figsize=(12, 8))
bplot = sns.boxplot(y="y", x="x", data=left, width=0.5)
bplot.set_title("Measurements for Each Distance from Obstacle")

# Step 3: Melt the DataFrame to have a format suitable for seaborn
# This will convert the data from wide format to long format
# left_melted = pd.melt(
#     left,
#     id_vars="Timestamp",
#     value_vars=["Left", "Right"],
#     var_name="Side",
#     value_name="Value",
# )

# right_melted = pd.melt(
#     right,
#     id_vars="Timestamp",
#     value_vars=["Left", "Right"],
#     var_name="Side",
#     value_name="Value",
# )

# # Step 4: Plot the data
# fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
# sns.lineplot(data=left_melted, x="Timestamp", y="Value", hue="Side", ax=ax1)


# plt.subplot(2, 1, 2)
# sns.lineplot(data=right_melted, x="Timestamp", y="Value", hue="Side", ax=ax2)
# plt.tight_layout()

# plt.show()
