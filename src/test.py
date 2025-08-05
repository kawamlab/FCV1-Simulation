import numpy as np
from build.simulator import StoneSimulator
import json

stone_simulator = StoneSimulator()

with open("data.json", "r") as read_file:
    data = json.load(read_file)

position: list = data["position"]
np_position = np.array(position)
shot = data["shot"]
x_velocities: list = data["x_velocities"]
y_velocities: list = data["y_velocities"]
angular_velocities: list = data["angular_velocities"]
np_x_velocities = np.array(x_velocities)
np_y_velocities = np.array(y_velocities)
np_angular_velocities = np.array(angular_velocities)

result, flag = stone_simulator.simulator(np_position, shot, np_x_velocities, np_y_velocities, np_angular_velocities)

# print(result)
# print(flag)
print(result)
