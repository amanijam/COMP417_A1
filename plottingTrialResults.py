import numpy as np
import matplotlib.pyplot as plt

# Point Robot

step_size = [4, 6, 10, 12, 15, 20, 25, 50, 75, 100]
point_avg_num_rrt_iterations = [
    5835.5,
    3564.2,
    1352.8,
    994.6,
    635.2,
    513.1,
    310.4,
    82.9,
    50.5,
    21.3,
]
point_avg_rrt_path_length = [
    1319.2,
    1293.6,
    1253.2,
    1219.2,
    1239,
    1294,
    1252,
    1215,
    1267.5,
    1080,
]

fig = plt.figure(figsize=(12, 12))
# fig, axes = plt.subplots(nrows=1, ncols=2)

plt.subplot(2, 2, 1)
plt.bar(step_size, point_avg_num_rrt_iterations, color="maroon", width=1)
plt.xlabel("Step Size")
plt.ylabel("Number of RRT iterations (avg of 10 trials)")
plt.title("Part 2C (a) num-rrt-iterations vs step-size")

plt.subplot(2, 2, 2)
plt.bar(step_size, point_avg_rrt_path_length, color="blue", width=1)
plt.xlabel("Step Size")
plt.ylabel("RRT Path length (avg of 10 trials)")
plt.title("Part 2C (b) rrt-path-length vs step-size")

robot_length = [5, 10, 15, 20, 25, 30, 35, 40, 45, 50]
line_avg_num_rrt_iterations = [
    3142.7,
    2436.6,
    2060.9,
    2759,
    1509.4,
    1187.5,
    1253.5,
    1668,
    1594.8,
    2042.2,
]
line_avg_rrt_path_length = [
    1266,
    1312.2,
    1341,
    1330.8,
    1295.4,
    1282.2,
    1290.6,
    1311.6,
    1282.2,
    1255.2,
]

plt.subplot(2, 2, 3)
plt.bar(robot_length, line_avg_num_rrt_iterations, color="green", width=1)
plt.xlabel("Robot Length")
plt.ylabel("Number of RRT iterations (avg of 10 trials)")
plt.title("Part 3B (a) num-rrt-iterations vs robot-length")

plt.subplot(2, 2, 4)
plt.bar(robot_length, line_avg_rrt_path_length, color="blue", width=1)
plt.xlabel("Robot Length")
plt.ylabel("RRT Path length (avg of 10 trials)")
plt.title("Part 3B (b) rrt-path-length vs robot-length")

plt.show()
