import numpy as np
from build.simulator import StoneSimulator

from pprint import pprint
import json

stone_simulator = StoneSimulator()

with open("data.json", "r") as read_file:
    data = json.load(read_file)

team0_position: list = data["team0_positions"]
team1_position: list = data["team1_positions"]
np_team0_position = np.array(team0_position)
np_team1_position = np.array(team1_position)
shot = data["shot"]
shot_per_team = data["shot_per_team"]
team_id = data["team_id"]
x_velocities: list = data["x_velocities"]
y_velocities: list = data["y_velocities"]
angular_velocities: list = data["angular_velocities"]
np_x_velocities = np.array(x_velocities)
np_y_velocities = np.array(y_velocities)
np_angular_velocities = np.array(angular_velocities)

simulated_stones_position = stone_simulator.simulator(np_team0_position, np_team1_position, shot, np_x_velocities, np_y_velocities, np_angular_velocities, team_id, shot_per_team, 1)

# print(result)
# print(flag)
for i in range(len(simulated_stones_position)):
    pprint(simulated_stones_position[i])
# 1.9991474151611328 39.983848571777344