from utils import put_object
import random

with open("objects_list.txt") as objects_list_file:
    objects_list = objects_list_file.read().split("\n")
    objects_list = [i[8:] for i in objects_list]

number = [5, 0, 0]

objects_to_place = [random.sample(objects_list, i) for i in number]
zones = [
    [[0.5, 1.5],
    [1.7, 1.9],
    [0.4, 0.4]],

    [[-0.1, 0.1],
    [1.7, 1.9],
    [0.6, 0.6]],

    [[-0.3, 1.7],
    [1.2, 1.5],
    [0, 0]]
]

# for zone in zones:
#     for i in range(2):
#         put_object(random.sample(objects_list, 1)[0],
#             zone[0][i],
#             zone[1][i],
#             zone[2][i])

for i in range(len(zones)):
    zone = zones[i]
    for object_to_place in objects_to_place[i]:
        put_object(object_to_place,
            zone[0][0] + random.uniform(0, zone[0][1] - zone[0][0]),
            zone[1][0] + random.uniform(0, zone[1][1] - zone[1][0]),
            zone[2][0] + random.uniform(0, zone[2][1] - zone[2][0]))
